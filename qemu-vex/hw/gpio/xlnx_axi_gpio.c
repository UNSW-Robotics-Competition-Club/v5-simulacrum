/*
 * Xilinx AXI GPIO (PG144), minimal: two channels of DATA/TRI, no interrupts.
 *
 * Inputs are the "in1"/"in2" words, settable at any time (QMP qom-set). On
 * the VEX V5 brain, 0x41210000 bit 7 is the front button (active low, so
 * idle reads 1). With "button-key" set (a QKeyCode name, e.g. "b"), that
 * key held in the display window pulls the "button-mask" bits of in1 low;
 * so does the named GPIO input "button" (e.g. a click on the display's
 * button bar).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qapi/util.h"
#include "qapi/qapi-types-ui.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "ui/input.h"
#include "qom/object.h"

#define TYPE_XLNX_AXI_GPIO "xlnx.axi-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(XlnxAxiGpioState, XLNX_AXI_GPIO)

#define R_DATA   0x00
#define R_TRI    0x04
#define R_DATA2  0x08
#define R_TRI2   0x0C
#define R_GIER   0x11C
#define R_IPISR  0x120
#define R_IPIER  0x128

struct XlnxAxiGpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t out[2];
    uint32_t tri[2];
    uint32_t in[2];
    uint32_t gier;
    uint32_t ipier;

    char *button_key;
    uint32_t button_mask;
    unsigned int button_linux;  /* QemuInputKeyEvent.key: a Linux keycode */
    QemuInputHandlerState *input;
};

static uint32_t axi_gpio_data(XlnxAxiGpioState *s, int ch)
{
    return (s->in[ch] & s->tri[ch]) | (s->out[ch] & ~s->tri[ch]);
}

static uint64_t axi_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    XlnxAxiGpioState *s = opaque;

    switch (offset) {
    case R_DATA:
        return axi_gpio_data(s, 0);
    case R_TRI:
        return s->tri[0];
    case R_DATA2:
        return axi_gpio_data(s, 1);
    case R_TRI2:
        return s->tri[1];
    case R_GIER:
        return s->gier;
    case R_IPISR:
        return 0;
    case R_IPIER:
        return s->ipier;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "axi-gpio: bad read 0x%" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
}

static void axi_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    XlnxAxiGpioState *s = opaque;

    switch (offset) {
    case R_DATA:
        s->out[0] = value;
        break;
    case R_TRI:
        s->tri[0] = value;
        break;
    case R_DATA2:
        s->out[1] = value;
        break;
    case R_TRI2:
        s->tri[1] = value;
        break;
    case R_GIER:
        s->gier = value;
        break;
    case R_IPISR:
        break;
    case R_IPIER:
        s->ipier = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "axi-gpio: bad write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", offset, value);
        break;
    }
}

static const MemoryRegionOps axi_gpio_ops = {
    .read = axi_gpio_read,
    .write = axi_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void axi_gpio_key_event(DeviceState *dev, QemuConsole *src,
                               QemuInputEvent *evt)
{
    XlnxAxiGpioState *s = XLNX_AXI_GPIO(dev);

    if (evt->type != INPUT_EVENT_KIND_KEY || evt->key.key != s->button_linux) {
        return;
    }
    if (evt->key.down) {
        s->in[0] &= ~s->button_mask;      /* active low */
    } else {
        s->in[0] |= s->button_mask;
    }
}

static void axi_gpio_button_in(void *opaque, int n, int level)
{
    XlnxAxiGpioState *s = opaque;

    if (level) {
        s->in[0] &= ~s->button_mask;      /* active low */
    } else {
        s->in[0] |= s->button_mask;
    }
}

static const QemuInputHandler axi_gpio_key_handler = {
    .name  = "AXI GPIO button",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = axi_gpio_key_event,
};

static void axi_gpio_reset(DeviceState *dev)
{
    XlnxAxiGpioState *s = XLNX_AXI_GPIO(dev);

    s->out[0] = s->out[1] = 0;
    /* Reset value: all pins inputs. */
    s->tri[0] = s->tri[1] = 0xFFFFFFFF;
    s->gier = 0;
    s->ipier = 0;
}

static void axi_gpio_realize(DeviceState *dev, Error **errp)
{
    XlnxAxiGpioState *s = XLNX_AXI_GPIO(dev);

    if (!s->button_key || !*s->button_key) {
        return;
    }
    int qcode = qapi_enum_parse(&QKeyCode_lookup, s->button_key, -1, errp);

    if (qcode < 0) {
        return;
    }
    s->button_linux = qemu_input_map_qcode_to_linux[qcode];
    s->input = qemu_input_handler_register(dev, &axi_gpio_key_handler);
    qemu_input_handler_activate(s->input);
}

static void axi_gpio_init(Object *obj)
{
    XlnxAxiGpioState *s = XLNX_AXI_GPIO(obj);

    memory_region_init_io(&s->iomem, obj, &axi_gpio_ops, s,
                          TYPE_XLNX_AXI_GPIO, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    object_property_add_uint32_ptr(obj, "in1", &s->in[0], OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "in2", &s->in[1], OBJ_PROP_FLAG_READWRITE);
    qdev_init_gpio_in_named(DEVICE(obj), axi_gpio_button_in, "button", 1);
}

static const Property axi_gpio_props[] = {
    DEFINE_PROP_STRING("button-key", XlnxAxiGpioState, button_key),
    DEFINE_PROP_UINT32("button-mask", XlnxAxiGpioState, button_mask, 0),
};

static void axi_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, axi_gpio_reset);
    device_class_set_props(dc, axi_gpio_props);
    dc->realize = axi_gpio_realize;
    dc->desc = "Xilinx AXI GPIO";
}

static const TypeInfo axi_gpio_info = {
    .name = TYPE_XLNX_AXI_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XlnxAxiGpioState),
    .instance_init = axi_gpio_init,
    .class_init = axi_gpio_class_init,
};

static void axi_gpio_register_types(void)
{
    type_register_static(&axi_gpio_info);
}

type_init(axi_gpio_register_types)
