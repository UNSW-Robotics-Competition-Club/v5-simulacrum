/*
 * Cadence I2C controller (Zynq-7000 PS I2C, UG585 ch. 20), polled-mode model.
 *
 * Transfers complete synchronously: a write to ADDR runs the transfer on the
 * QEMU I2C bus at once, so status bits the driver polls for are already set
 * when it looks. Enough for the Xilinx XIicPs polled driver.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_CADENCE_I2C "cadence.i2c"
OBJECT_DECLARE_SIMPLE_TYPE(CadenceI2CState, CADENCE_I2C)

#define R_CR          0x00
#define R_SR          0x04
#define R_ADDR        0x08
#define R_DATA        0x0C
#define R_ISR         0x10
#define R_TRANS_SIZE  0x14
#define R_SLV_PAUSE   0x18
#define R_TIME_OUT    0x1C
#define R_IMR         0x20
#define R_IER         0x24
#define R_IDR         0x28

#define CR_RW         (1 << 0)
#define CR_HOLD       (1 << 4)
#define CR_CLR_FIFO   (1 << 6)

#define SR_RXDV       (1 << 5)
#define SR_TXDV       (1 << 6)
#define SR_BA         (1 << 8)

#define ISR_COMP      (1 << 0)
#define ISR_DATA      (1 << 1)
#define ISR_NACK      (1 << 2)
#define ISR_MASK      0x2FF

#define FIFO_DEPTH    16

struct CadenceI2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;

    uint32_t cr;
    uint32_t addr;
    uint32_t isr;
    uint32_t imr;
    uint32_t trans_size;
    uint32_t time_out;
    uint32_t slv_pause;

    /* Bytes written to DATA before the transfer starts. */
    uint8_t tx_fifo[FIFO_DEPTH];
    uint32_t tx_len;
    /* A send transfer is open on the bus and further DATA writes go out. */
    bool tx_open;
    /* A receive is open; DATA reads pull TRANS_SIZE more bytes. */
    bool rx_open;
};

static void cadence_i2c_update_irq(CadenceI2CState *s)
{
    qemu_set_irq(s->irq, !!(s->isr & ~s->imr & ISR_MASK));
}

static void cadence_i2c_close(CadenceI2CState *s)
{
    if (s->tx_open || s->rx_open) {
        i2c_end_transfer(s->bus);
        s->tx_open = false;
        s->rx_open = false;
    }
}

static void cadence_i2c_start(CadenceI2CState *s)
{
    uint8_t slave = s->addr & 0x7F;
    uint32_t i;

    cadence_i2c_close(s);

    if (s->cr & CR_RW) {
        /*
         * The driver writes TRANS_SIZE after ADDR, so bytes are pulled from
         * the slave on demand as DATA is read.
         */
        if (i2c_start_recv(s->bus, slave)) {
            s->isr |= ISR_NACK;
        } else {
            s->rx_open = true;
        }
    } else {
        if (i2c_start_send(s->bus, slave)) {
            s->tx_len = 0;
            s->isr |= ISR_NACK;
            cadence_i2c_update_irq(s);
            return;
        }
        for (i = 0; i < s->tx_len; i++) {
            i2c_send(s->bus, s->tx_fifo[i]);
        }
        s->tx_len = 0;
        /*
         * Keep the transfer open even without HOLD: the driver may refill
         * the FIFO after starting. It closes on the next ADDR or CR write.
         */
        s->tx_open = true;
        s->isr |= ISR_COMP;
    }
    cadence_i2c_update_irq(s);
}

static uint8_t cadence_i2c_recv_byte(CadenceI2CState *s)
{
    uint8_t v;

    if (!s->rx_open || !s->trans_size) {
        return 0;
    }
    v = i2c_recv(s->bus);
    if (--s->trans_size == 0) {
        i2c_nack(s->bus);
        i2c_end_transfer(s->bus);
        s->rx_open = false;
        s->isr |= ISR_COMP;
        cadence_i2c_update_irq(s);
    }
    return v;
}

static uint64_t cadence_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    CadenceI2CState *s = opaque;
    uint32_t v = 0;

    switch (offset) {
    case R_CR:
        return s->cr;
    case R_SR:
        if (s->rx_open && s->trans_size) {
            v |= SR_RXDV;
        }
        if (s->tx_len) {
            v |= SR_TXDV;
        }
        return v;
    case R_ADDR:
        return s->addr;
    case R_DATA:
        return cadence_i2c_recv_byte(s);
    case R_ISR:
        return s->isr;
    case R_TRANS_SIZE:
        return s->trans_size;
    case R_SLV_PAUSE:
        return s->slv_pause;
    case R_TIME_OUT:
        return s->time_out;
    case R_IMR:
        return s->imr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "cadence_i2c: bad read 0x%" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
}

static void cadence_i2c_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    CadenceI2CState *s = opaque;

    switch (offset) {
    case R_CR:
        if (value & CR_CLR_FIFO) {
            s->tx_len = 0;
            s->trans_size = 0;
        }
        s->cr = value & ~CR_CLR_FIFO;
        /*
         * Dropping HOLD ends an open send. An open receive runs on until
         * TRANS_SIZE bytes are read: the driver drops HOLD mid-receive.
         */
        if (!(s->cr & CR_HOLD) && s->tx_open) {
            cadence_i2c_close(s);
        }
        break;
    case R_ADDR:
        s->addr = value & 0x3FF;
        cadence_i2c_start(s);
        break;
    case R_DATA:
        if (s->tx_open) {
            i2c_send(s->bus, value);
        } else if (s->tx_len < FIFO_DEPTH) {
            s->tx_fifo[s->tx_len++] = value;
        }
        break;
    case R_ISR:
        s->isr &= ~(value & ISR_MASK);
        cadence_i2c_update_irq(s);
        break;
    case R_TRANS_SIZE:
        s->trans_size = value & 0xFF;
        break;
    case R_SLV_PAUSE:
        s->slv_pause = value & 0xF;
        break;
    case R_TIME_OUT:
        s->time_out = value & 0xFF;
        break;
    case R_IER:
        s->imr &= ~(value & ISR_MASK);
        cadence_i2c_update_irq(s);
        break;
    case R_IDR:
        s->imr |= value & ISR_MASK;
        cadence_i2c_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "cadence_i2c: bad write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", offset, value);
        break;
    }
}

static const MemoryRegionOps cadence_i2c_ops = {
    .read = cadence_i2c_read,
    .write = cadence_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void cadence_i2c_reset(DeviceState *dev)
{
    CadenceI2CState *s = CADENCE_I2C(dev);

    cadence_i2c_close(s);
    s->cr = 0;
    s->addr = 0;
    s->isr = 0;
    s->imr = ISR_MASK;
    s->trans_size = 0;
    s->time_out = 0x1F;
    s->slv_pause = 0;
    s->tx_len = 0;
    cadence_i2c_update_irq(s);
}

static void cadence_i2c_init(Object *obj)
{
    CadenceI2CState *s = CADENCE_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &cadence_i2c_ops, s,
                          TYPE_CADENCE_I2C, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void cadence_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, cadence_i2c_reset);
    dc->desc = "Cadence I2C controller (Zynq-7000)";
}

static const TypeInfo cadence_i2c_info = {
    .name = TYPE_CADENCE_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CadenceI2CState),
    .instance_init = cadence_i2c_init,
    .class_init = cadence_i2c_class_init,
};

static void cadence_i2c_register_types(void)
{
    type_register_static(&cadence_i2c_info);
}

type_init(cadence_i2c_register_types)
