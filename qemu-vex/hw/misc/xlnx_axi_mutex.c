/*
 * Xilinx AXI Mutex (PG115): 32 mutexes, one 0x100-byte slot each.
 *
 *   +0x0 MUTEX  bit 0 locked, bits 8:1 owner CPU id. Writing (id<<1)|1
 *               takes a free mutex; writing (id<<1) releases one owned by id.
 *               The XMutex driver reads back to see whether it won.
 *   +0x4 USER   free 32-bit data word.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_XLNX_AXI_MUTEX "xlnx.axi-mutex"
OBJECT_DECLARE_SIMPLE_TYPE(XlnxAxiMutexState, XLNX_AXI_MUTEX)

#define NUM_MUTEX     32
#define SLOT_SHIFT    8
#define R_MUTEX       0x0
#define R_USER        0x4
#define MUTEX_LOCKED  1
#define MUTEX_OWNER   0x1FE

struct XlnxAxiMutexState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t mutex[NUM_MUTEX];
    uint32_t user[NUM_MUTEX];
};

static uint64_t axi_mutex_read(void *opaque, hwaddr offset, unsigned size)
{
    XlnxAxiMutexState *s = opaque;
    unsigned n = offset >> SLOT_SHIFT;

    if (n < NUM_MUTEX) {
        switch (offset & 0xFF) {
        case R_MUTEX:
            return s->mutex[n];
        case R_USER:
            return s->user[n];
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "axi-mutex: bad read 0x%" HWADDR_PRIx "\n",
                  offset);
    return 0;
}

static void axi_mutex_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    XlnxAxiMutexState *s = opaque;
    unsigned n = offset >> SLOT_SHIFT;

    if (n >= NUM_MUTEX) {
        goto bad;
    }
    switch (offset & 0xFF) {
    case R_MUTEX:
        if (value & MUTEX_LOCKED) {
            if (!(s->mutex[n] & MUTEX_LOCKED)) {
                s->mutex[n] = value & (MUTEX_OWNER | MUTEX_LOCKED);
            }
        } else if ((s->mutex[n] & MUTEX_OWNER) == (value & MUTEX_OWNER)) {
            s->mutex[n] = 0;
        }
        return;
    case R_USER:
        s->user[n] = value;
        return;
    }
bad:
    qemu_log_mask(LOG_GUEST_ERROR, "axi-mutex: bad write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", offset, value);
}

static const MemoryRegionOps axi_mutex_ops = {
    .read = axi_mutex_read,
    .write = axi_mutex_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void axi_mutex_reset(DeviceState *dev)
{
    XlnxAxiMutexState *s = XLNX_AXI_MUTEX(dev);

    memset(s->mutex, 0, sizeof(s->mutex));
    memset(s->user, 0, sizeof(s->user));
}

static void axi_mutex_init(Object *obj)
{
    XlnxAxiMutexState *s = XLNX_AXI_MUTEX(obj);

    memory_region_init_io(&s->iomem, obj, &axi_mutex_ops, s,
                          TYPE_XLNX_AXI_MUTEX, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void axi_mutex_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, axi_mutex_reset);
    dc->desc = "Xilinx AXI Mutex";
}

static const TypeInfo axi_mutex_info = {
    .name = TYPE_XLNX_AXI_MUTEX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XlnxAxiMutexState),
    .instance_init = axi_mutex_init,
    .class_init = axi_mutex_class_init,
};

static void axi_mutex_register_types(void)
{
    type_register_static(&axi_mutex_info);
}

type_init(axi_mutex_register_types)
