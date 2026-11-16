/*
 * Xylon logiCVC-ML display controller, as configured in the VEX V5 brain PL.
 *
 * Config (MainLoop table @ 0x010B5D9C): IP 4.1.2, regs at 0x43C00000,
 * 5 layers, 24 bpp stored as 32-bit XRGB, row stride 512 pixels, layer
 * buffer addresses programmed through each layer's ADDRESS register.
 *
 * Registers are 8 bytes apart (Linux drivers/gpu/drm/logicvc). Layer
 * positions count from the bottom-right corner: x = HRES - 1 - HPOSITION.
 * Layer 0 is on top: while a program runs, VEXos shows the user layer (2,
 * a 480x240 window under the header) and its status screen (3) at full
 * alpha, and the user's drawing is what the brain displays.
 *
 * With "skin" set (a binary PPM, sim/skins/make_brain_skin.py) the console
 * shows that picture of the brain with the live screen inside it. The PPM
 * header comment "# v5skin screen=X,Y button=X,Y,HALF" says where: the
 * screen's top-left and the front button's centre and half-side. The
 * button is drawn lit while "button-gpio"'s in1 has the "button-mask" bits
 * low. The geometry is readable as properties for the touch device.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_LOGICVC "logicvc"
OBJECT_DECLARE_SIMPLE_TYPE(LogiCvcState, LOGICVC)

#define R_HRES          0x18
#define R_VRES          0x38
#define R_BG_COLOR      0x50
#define R_INT_STAT      0x68
#define R_INT_MASK      0x70
#define R_IP_VERSION    0xF8

#define INT_V_SYNC      (1 << 5)

#define LAYER_BASE      0x100
#define LAYER_STRIDE    0x80
#define NUM_LAYERS      5
/* Per-layer register offsets. */
#define L_ADDRESS       0x00
#define L_HPOSITION     0x10
#define L_VPOSITION     0x18
#define L_WIDTH         0x20
#define L_HEIGHT        0x28
#define L_ALPHA         0x30
#define L_CTRL          0x38
#define L_TRANSPARENT   0x40

#define L_CTRL_ENABLE           (1 << 0)
#define L_CTRL_COLOR_KEY_DIS    (1 << 1)

#define NUM_REGS        (0x400 / 8)
#define IP_VERSION_4_1_2  ((4 << 11) | (1 << 5) | 2)

#define DEFAULT_WIDTH   480
#define DEFAULT_HEIGHT  272
#define VSYNC_NS        (NANOSECONDS_PER_SECOND / 60)

struct LogiCvcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QemuConsole *con;
    QEMUTimer *vsync_timer;

    uint32_t regs[NUM_REGS];
    uint32_t row_stride;   /* pixels */
    uint32_t *line;

    char *skin_path;
    uint32_t *skin;         /* skin_w x skin_h, 0xRRGGBB */
    uint32_t *screen;       /* the composed screen, when skinned */
    int screen_px;
    uint32_t skin_w, skin_h, screen_x, screen_y;
    uint32_t button_x, button_y, button_half;
    Object *button_gpio;
    uint32_t button_mask;
};

static uint32_t lcvc_reg(LogiCvcState *s, hwaddr off)
{
    return s->regs[off / 8];
}

static uint32_t lcvc_layer_reg(LogiCvcState *s, int n, hwaddr off)
{
    return lcvc_reg(s, LAYER_BASE + n * LAYER_STRIDE + off);
}

static int lcvc_width(LogiCvcState *s)
{
    return (lcvc_reg(s, R_HRES) & 0xFFF) + 1;
}

static int lcvc_height(LogiCvcState *s)
{
    return (lcvc_reg(s, R_VRES) & 0xFFF) + 1;
}

static void lcvc_update_irq(LogiCvcState *s)
{
    qemu_set_irq(s->irq, !!(lcvc_reg(s, R_INT_STAT) & ~lcvc_reg(s, R_INT_MASK)));
}

static void lcvc_vsync(void *opaque)
{
    LogiCvcState *s = opaque;

    s->regs[R_INT_STAT / 8] |= INT_V_SYNC;
    lcvc_update_irq(s);
    timer_mod(s->vsync_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VSYNC_NS);
}

static uint32_t lcvc_blend(uint32_t dst, uint32_t src, uint32_t alpha)
{
    uint32_t out = 0;
    int shift;

    for (shift = 0; shift < 24; shift += 8) {
        uint32_t d = (dst >> shift) & 0xFF;
        uint32_t c = (src >> shift) & 0xFF;
        out |= ((c * alpha + d * (255 - alpha)) / 255) << shift;
    }
    return out;
}

static void lcvc_draw_layer(LogiCvcState *s, int n, uint32_t *fb,
                            int width, int height)
{
    uint32_t ctrl = lcvc_layer_reg(s, n, L_CTRL);
    uint32_t alpha = lcvc_layer_reg(s, n, L_ALPHA) & 0xFF;
    uint32_t key = lcvc_layer_reg(s, n, L_TRANSPARENT) & 0xFFFFFF;
    bool use_key = !(ctrl & L_CTRL_COLOR_KEY_DIS);
    hwaddr base = lcvc_layer_reg(s, n, L_ADDRESS);
    int x0 = width - 1 - (int)(lcvc_layer_reg(s, n, L_HPOSITION) & 0xFFF);
    int y0 = height - 1 - (int)(lcvc_layer_reg(s, n, L_VPOSITION) & 0xFFF);
    int lw = (lcvc_layer_reg(s, n, L_WIDTH) & 0xFFF) + 1;
    int lh = (lcvc_layer_reg(s, n, L_HEIGHT) & 0xFFF) + 1;
    int x, y;

    if (!(ctrl & L_CTRL_ENABLE) || alpha == 0) {
        return;
    }
    x0 = MAX(x0, 0);
    y0 = MAX(y0, 0);
    lw = MIN(lw, width - x0);
    lw = MIN(lw, (int)s->row_stride);   /* s->line holds row_stride pixels */
    lh = MIN(lh, height - y0);

    for (y = 0; y < lh; y++) {
        uint32_t *dst = fb + (y0 + y) * width + x0;

        if (dma_memory_read(&address_space_memory,
                            base + (hwaddr)y * s->row_stride * 4,
                            s->line, lw * 4,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            continue;   /* unmapped layer memory: leave the row as is */
        }
        for (x = 0; x < lw; x++) {
            uint32_t px = le32_to_cpu(s->line[x]) & 0xFFFFFF;

            if (use_key && px == key) {
                continue;
            }
            dst[x] = alpha == 0xFF ? px : lcvc_blend(dst[x], px, alpha);
        }
    }
}

#define BTN_LIT      0x5FD16B

static bool lcvc_button_lit(LogiCvcState *s)
{
    return s->button_gpio && s->button_half &&
           !(object_property_get_uint(s->button_gpio, "in1", NULL) & s->button_mask);
}

/* A pressed button: a green wash and outline over the drawn button. */
static void lcvc_draw_lit_button(LogiCvcState *s, uint32_t *fb, int stride)
{
    int h = s->button_half, x, y;

    for (y = -h; y <= h; y++) {
        for (x = -h; x <= h; x++) {
            int px = s->button_x + x, py = s->button_y + y;
            bool edge = abs(x) >= h - 2 || abs(y) >= h - 2;

            if (px < 0 || py < 0 || px >= (int)s->skin_w || py >= (int)s->skin_h) {
                continue;
            }
            uint32_t *d = &fb[py * stride + px];
            *d = edge ? BTN_LIT : lcvc_blend(*d, BTN_LIT, 90);
        }
    }
}

static bool lcvc_gfx_update(void *opaque)
{
    LogiCvcState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int width = lcvc_width(s);
    int height = lcvc_height(s);
    uint32_t bg = lcvc_reg(s, R_BG_COLOR) & 0xFFFFFF;
    int cw = s->skin ? (int)s->skin_w : width;
    int ch = s->skin ? (int)s->skin_h : height;
    uint32_t *fb, *scr;
    int i, y;

    if (width < 16 || height < 16) {
        return true;
    }
    if (!surface || surface_width(surface) != cw || surface_height(surface) != ch) {
        qemu_console_resize(s->con, cw, ch);
        surface = qemu_console_surface(s->con);
    }
    fb = surface_data(surface);
    scr = fb;
    if (s->skin) {
        if (s->screen_px != width * height) {
            g_free(s->screen);
            s->screen = g_new(uint32_t, width * height);
            s->screen_px = width * height;
        }
        scr = s->screen;
    }
    for (i = 0; i < width * height; i++) {
        scr[i] = bg;
    }
    for (i = NUM_LAYERS - 1; i >= 0; i--) {
        lcvc_draw_layer(s, i, scr, width, height);
    }
    if (s->skin) {
        /* the surface's rows are padded: step by its stride */
        int stride = surface_stride(surface) / 4;
        int w = MIN(width, cw - (int)s->screen_x);
        int h = MIN(height, ch - (int)s->screen_y);

        for (y = 0; y < ch; y++) {
            memcpy(&fb[y * stride], &s->skin[y * cw], (size_t)cw * 4);
        }
        for (y = 0; y < h; y++) {
            memcpy(&fb[(s->screen_y + y) * stride + s->screen_x], &scr[y * width],
                   (size_t)w * 4);
        }
        if (lcvc_button_lit(s)) {
            lcvc_draw_lit_button(s, fb, stride);
        }
    }
    qemu_console_update(s->con, 0, 0, cw, ch);
    return true;
}

static const GraphicHwOps lcvc_gfx_ops = {
    .gfx_update = lcvc_gfx_update,
};

static uint64_t lcvc_read(void *opaque, hwaddr offset, unsigned size)
{
    LogiCvcState *s = opaque;

    if (offset == R_IP_VERSION) {
        return IP_VERSION_4_1_2;
    }
    if (offset / 8 < NUM_REGS && !(offset & 7)) {
        return s->regs[offset / 8];
    }
    qemu_log_mask(LOG_UNIMP, "logicvc: read 0x%" HWADDR_PRIx "\n", offset);
    return 0;
}

static void lcvc_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    LogiCvcState *s = opaque;

    if (offset / 8 >= NUM_REGS || (offset & 7)) {
        qemu_log_mask(LOG_UNIMP, "logicvc: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", offset, value);
        return;
    }
    if (offset == R_INT_STAT) {
        s->regs[R_INT_STAT / 8] &= ~value;
    } else {
        s->regs[offset / 8] = value;
    }
    if (offset == R_INT_STAT || offset == R_INT_MASK) {
        lcvc_update_irq(s);
    }
}

static const MemoryRegionOps lcvc_ops = {
    .read = lcvc_read,
    .write = lcvc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void lcvc_reset(DeviceState *dev)
{
    LogiCvcState *s = LOGICVC(dev);
    int n;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_HRES / 8] = DEFAULT_WIDTH - 1;
    s->regs[R_VRES / 8] = DEFAULT_HEIGHT - 1;
    s->regs[R_INT_MASK / 8] = 0xFFFF;
    /* Layers not repositioned by the firmware cover the whole screen. */
    for (n = 0; n < NUM_LAYERS; n++) {
        hwaddr b = (LAYER_BASE + n * LAYER_STRIDE) / 8;

        s->regs[b + L_HPOSITION / 8] = DEFAULT_WIDTH - 1;
        s->regs[b + L_VPOSITION / 8] = DEFAULT_HEIGHT - 1;
        s->regs[b + L_WIDTH / 8] = DEFAULT_WIDTH - 1;
        s->regs[b + L_HEIGHT / 8] = DEFAULT_HEIGHT - 1;
        s->regs[b + L_ALPHA / 8] = 0xFF;
    }
    lcvc_update_irq(s);
    timer_mod(s->vsync_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VSYNC_NS);
}

/* Next header token of a PPM; "v5skin" comments fill in the geometry. */
static const char *ppm_token(LogiCvcState *s, const char *p, const char *end)
{
    while (p < end) {
        if (*p == '#') {
            const char *eol = memchr(p, '\n', end - p);
            g_autofree char *line = g_strndup(p, (eol ? eol : end) - p);
            const char *tag = strstr(line, "v5skin");

            if (tag) {
                sscanf(tag, "v5skin screen=%u,%u button=%u,%u,%u", &s->screen_x,
                       &s->screen_y, &s->button_x, &s->button_y, &s->button_half);
            }
            p = eol ? eol + 1 : end;
        } else if (g_ascii_isspace(*p)) {
            p++;
        } else {
            return p;
        }
    }
    return NULL;
}

static bool lcvc_load_skin(LogiCvcState *s, Error **errp)
{
    g_autofree char *buf = NULL;
    g_autoptr(GError) gerr = NULL;
    gsize len;
    const char *p, *end;
    unsigned long v[3];
    char *next;
    int i;

    if (!g_file_get_contents(s->skin_path, &buf, &len, &gerr)) {
        error_setg(errp, "logicvc: skin %s: %s", s->skin_path, gerr->message);
        return false;
    }
    end = buf + len;
    if (len < 2 || buf[0] != 'P' || buf[1] != '6') {
        error_setg(errp, "logicvc: skin %s is not a binary PPM (P6)", s->skin_path);
        return false;
    }
    p = buf + 2;
    for (i = 0; i < 3; i++) {             /* width, height, maxval */
        p = ppm_token(s, p, end);
        if (!p) {
            break;
        }
        v[i] = strtoul(p, &next, 10);
        p = next;
    }
    if (i < 3 || v[2] != 255 || v[0] < DEFAULT_WIDTH || v[1] < DEFAULT_HEIGHT ||
        v[0] > 4096 || v[1] > 4096 || (gsize)(end - p) < v[0] * v[1] * 3 + 1) {
        error_setg(errp, "logicvc: skin %s: need an 8-bit PPM at least %dx%d",
                   s->skin_path, DEFAULT_WIDTH, DEFAULT_HEIGHT);
        return false;
    }
    p++;                                  /* the single whitespace byte */
    if (s->screen_x + DEFAULT_WIDTH > v[0] || s->screen_y + DEFAULT_HEIGHT > v[1]) {
        error_setg(errp, "logicvc: skin %s: the screen (v5skin screen=X,Y) "
                   "doesn't fit the picture", s->skin_path);
        return false;
    }
    s->skin_w = v[0];
    s->skin_h = v[1];
    s->skin = g_new(uint32_t, s->skin_w * s->skin_h);
    for (i = 0; i < (int)(s->skin_w * s->skin_h); i++, p += 3) {
        s->skin[i] = (uint8_t)p[0] << 16 | (uint8_t)p[1] << 8 | (uint8_t)p[2];
    }
    return true;
}

static void lcvc_realize(DeviceState *dev, Error **errp)
{
    LogiCvcState *s = LOGICVC(dev);

    if (s->row_stride == 0 || s->row_stride > 4096) {
        error_setg(errp, "logicvc: row-stride must be 1..4096");
        return;
    }
    s->line = g_new0(uint32_t, s->row_stride);
    s->vsync_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lcvc_vsync, s);
    s->skin_w = DEFAULT_WIDTH;
    s->skin_h = DEFAULT_HEIGHT;
    if (s->skin_path && *s->skin_path && !lcvc_load_skin(s, errp)) {
        return;
    }
    s->con = qemu_graphic_console_create(dev, 0, &lcvc_gfx_ops, s);
    qemu_console_resize(s->con, s->skin_w, s->skin_h);
}

static void lcvc_init(Object *obj)
{
    LogiCvcState *s = LOGICVC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &lcvc_ops, s, TYPE_LOGICVC,
                          0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    /* the skin's geometry, for the touch device (read-only) */
    object_property_add_uint32_ptr(obj, "canvas-w", &s->skin_w, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "canvas-h", &s->skin_h, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "screen-x", &s->screen_x, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "screen-y", &s->screen_y, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "button-x", &s->button_x, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "button-y", &s->button_y, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "button-half", &s->button_half, OBJ_PROP_FLAG_READ);
}

static const Property lcvc_props[] = {
    DEFINE_PROP_UINT32("row-stride", LogiCvcState, row_stride, 512),
    DEFINE_PROP_STRING("skin", LogiCvcState, skin_path),
    DEFINE_PROP_LINK("button-gpio", LogiCvcState, button_gpio, TYPE_OBJECT, Object *),
    DEFINE_PROP_UINT32("button-mask", LogiCvcState, button_mask, 0),
};

static void lcvc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lcvc_realize;
    device_class_set_legacy_reset(dc, lcvc_reset);
    device_class_set_props(dc, lcvc_props);
    dc->desc = "Xylon logiCVC-ML display controller";
}

static const TypeInfo lcvc_info = {
    .name = TYPE_LOGICVC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LogiCvcState),
    .instance_init = lcvc_init,
    .class_init = lcvc_class_init,
};

static void lcvc_register_types(void)
{
    type_register_static(&lcvc_info);
}

type_init(lcvc_register_types)
