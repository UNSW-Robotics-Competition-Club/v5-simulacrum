/*
 * VEX V5 brain touchscreen, as seen by MainLoop.
 *
 * On hardware a PL soft core samples the resistive panel, writes a sample
 * block into OCM and raises AXI INTC0 source 23; CPU0 acks by writing the
 * register at 0x43520000. This device does the same from QEMU's absolute
 * pointer (mouse / tablet): left button = finger down.
 *
 * Sample block at 0xFFFF12D6 (MainLoop g_pTouchAdcData; byte offsets from
 * the disassembly of devUpdateStatusFlags / touchReadCalibratedPoint, the
 * floats are unaligned):
 *   +0x00 u16   0 = sample ready; CPU0 sets it back to 1 when consumed
 *   +0x04 u16   bit 0 finger down, bit 1 new sample
 *   +0x06 f32   Y raw     +0x0E f32  Y ref
 *   +0x0A f32   X raw     +0x12 f32  X ref
 * touchReadCalibratedPoint: x = -14 + 1.06 * Xraw/Xref * 480,
 * y = -10 + 1.101 * Yraw/Yref * 272, then uiTouchPointProcess y = 272 - y.
 *
 * With a skin (logicvc "skin": a picture of the brain around the screen)
 * the pointer covers the whole picture: "canvas-w/h" is its size,
 * "screen-x/y" where the screen sits, "button-x/y/half" the front button.
 * A press that starts on the button raises the named GPIO output "button";
 * one that starts elsewhere off the screen does nothing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "ui/input.h"
#include "qom/object.h"

#define TYPE_VEX_V5_TOUCH "vex-v5-touch"
OBJECT_DECLARE_SIMPLE_TYPE(VexV5TouchState, VEX_V5_TOUCH)

#define TOUCH_BLOCK        0xFFFF12D6
#define TB_READY           0x00
#define TB_FLAGS           0x04
#define TB_Y_RAW           0x06
#define TB_X_RAW           0x0A
#define TB_Y_REF           0x0E
#define TB_X_REF           0x12
#define TB_FLAG_DOWN       (1 << 0)
#define TB_FLAG_NEW        (1 << 1)

#define SCREEN_W           480
#define SCREEN_H           272
#define CAL_X_OFFSET       (-14.0f)
#define CAL_Y_OFFSET       (-10.0f)
#define CAL_X_GAIN         1.06f
#define CAL_Y_GAIN         1.101f

/*
 * The panel is scanned continuously, finger or not: MainLoop counts six
 * "up" samples (uiTouchStateTick) before it reports a release.
 */
#define SAMPLE_NS          (10 * SCALE_MS)

struct VexV5TouchState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    QemuInputHandlerState *input;

    int x;          /* screen pixels */
    int y;
    bool down;

    /* pointer position on the canvas (the skin, or just the screen) */
    int cx, cy;
    uint32_t canvas_w, canvas_h, screen_x, screen_y;
    uint32_t button_x, button_y, button_half;
    bool button_press;      /* the current press began on the button */
    bool off_screen_press;  /* ... or elsewhere off the screen */
    qemu_irq button;
};

static void touch_put_u16(hwaddr off, uint16_t v)
{
    uint16_t le = cpu_to_le16(v);

    address_space_write(&address_space_memory, TOUCH_BLOCK + off,
                        MEMTXATTRS_UNSPECIFIED, &le, sizeof(le));
}

static void touch_put_f32(hwaddr off, float v)
{
    uint32_t le;

    memcpy(&le, &v, sizeof(le));
    le = cpu_to_le32(le);
    address_space_write(&address_space_memory, TOUCH_BLOCK + off,
                        MEMTXATTRS_UNSPECIFIED, &le, sizeof(le));
}

static void touch_send_sample(VexV5TouchState *s)
{
    /* Invert the firmware's calibration with ref = 1.0. */
    float xraw = (s->x - CAL_X_OFFSET) / (CAL_X_GAIN * SCREEN_W);
    float yraw = ((SCREEN_H - s->y) - CAL_Y_OFFSET) / (CAL_Y_GAIN * SCREEN_H);

    touch_put_f32(TB_X_RAW, xraw);
    touch_put_f32(TB_X_REF, 1.0f);
    touch_put_f32(TB_Y_RAW, yraw);
    touch_put_f32(TB_Y_REF, 1.0f);
    touch_put_u16(TB_FLAGS, s->down ? TB_FLAG_DOWN | TB_FLAG_NEW : 0);
    touch_put_u16(TB_READY, 0);
    qemu_irq_raise(s->irq);
}

static void touch_tick(void *opaque)
{
    VexV5TouchState *s = opaque;

    touch_send_sample(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SAMPLE_NS);
}

static void touch_input_event(DeviceState *dev, QemuConsole *src,
                              QemuInputEvent *evt)
{
    VexV5TouchState *s = VEX_V5_TOUCH(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->cx = (int64_t)evt->abs.value * s->canvas_w / (INPUT_EVENT_ABS_MAX + 1);
            s->x = MAX(0, MIN(SCREEN_W - 1, s->cx - (int)s->screen_x));
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            s->cy = (int64_t)evt->abs.value * s->canvas_h / (INPUT_EVENT_ABS_MAX + 1);
            s->y = MAX(0, MIN(SCREEN_H - 1, s->cy - (int)s->screen_y));
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button != INPUT_BUTTON_LEFT) {
            break;
        }
        if (evt->btn.down) {
            bool on_screen = s->cx >= (int)s->screen_x && s->cy >= (int)s->screen_y &&
                             s->cx < (int)s->screen_x + SCREEN_W &&
                             s->cy < (int)s->screen_y + SCREEN_H;

            if (s->button_half && abs(s->cx - (int)s->button_x) <= (int)s->button_half &&
                abs(s->cy - (int)s->button_y) <= (int)s->button_half) {
                s->button_press = true;
                qemu_set_irq(s->button, 1);
            } else if (on_screen) {
                s->down = true;
            } else {
                s->off_screen_press = true;
            }
        } else if (s->button_press) {
            s->button_press = false;
            qemu_set_irq(s->button, 0);
        } else if (s->off_screen_press) {
            s->off_screen_press = false;
        } else {
            s->down = false;
        }
        break;
    default:
        break;
    }
}

static void touch_input_sync(DeviceState *dev)
{
    touch_send_sample(VEX_V5_TOUCH(dev));
}

static const QemuInputHandler touch_input_handler = {
    .name  = "VEX V5 touchscreen",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = touch_input_event,
    .sync  = touch_input_sync,
};

static uint64_t touch_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0;
}

static void touch_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    VexV5TouchState *s = opaque;

    /* Any write acknowledges the interrupt. */
    qemu_irq_lower(s->irq);
}

static const MemoryRegionOps touch_ops = {
    .read = touch_read,
    .write = touch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void touch_reset(DeviceState *dev)
{
    VexV5TouchState *s = VEX_V5_TOUCH(dev);

    s->x = 0;
    s->y = 0;
    s->down = false;
    qemu_irq_lower(s->irq);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SAMPLE_NS);
}

static void touch_realize(DeviceState *dev, Error **errp)
{
    VexV5TouchState *s = VEX_V5_TOUCH(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, touch_tick, s);
    s->input = qemu_input_handler_register(dev, &touch_input_handler);
    qemu_input_handler_activate(s->input);
}

static void touch_init(Object *obj)
{
    VexV5TouchState *s = VEX_V5_TOUCH(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &touch_ops, s, TYPE_VEX_V5_TOUCH,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->button, "button", 1);
    s->canvas_w = SCREEN_W;
    s->canvas_h = SCREEN_H;
    object_property_add_uint32_ptr(obj, "canvas-w", &s->canvas_w, OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "canvas-h", &s->canvas_h, OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "screen-x", &s->screen_x, OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "screen-y", &s->screen_y, OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "button-x", &s->button_x, OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "button-y", &s->button_y, OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "button-half", &s->button_half,
                                   OBJ_PROP_FLAG_READWRITE);
}

static void touch_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = touch_realize;
    device_class_set_legacy_reset(dc, touch_reset);
    dc->desc = "VEX V5 brain touchscreen";
}

static const TypeInfo touch_info = {
    .name = TYPE_VEX_V5_TOUCH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VexV5TouchState),
    .instance_init = touch_init,
    .class_init = touch_class_init,
};

static void touch_register_types(void)
{
    type_register_static(&touch_info);
}

type_init(touch_register_types)
