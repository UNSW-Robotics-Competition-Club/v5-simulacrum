/*
 * VEX V5 brain NXP co-processor (DEVFRM01), I2C slave at 0x50 on PS I2C0.
 *
 * Modelled as a 256-byte register file, as MainLoop uses it:
 *   0x00  u32 command / status word (nxpReadReg0, i2cWriteCmd0Word)
 *   0x04  u32 smart-port config mask (nxpGetPortConfigMask)
 *   0x08  EE settings, 0x38 bytes read at boot (sysLoadEESettings);
 *         regs 0x08..0xF7 are the byte registers of nxpReadRegByte
 *   0xFC  u32 system id (nxpReadRegFC)
 * The first byte of a write sets the register pointer; later bytes store
 * and advance it. Reads return bytes from the pointer onwards.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/core/qdev-properties.h"
#include "qom/object.h"

#define TYPE_VEX_V5_NXP "vex-v5-nxp"
OBJECT_DECLARE_SIMPLE_TYPE(VexV5NxpState, VEX_V5_NXP)

#define NXP_REG_PORT_MASK  0x04
#define NXP_REG_SYSTEM_ID  0xFC
/* EE setting register r lands at MainLoop RAM 0x0180B33D + r. */
#define NXP_REG_RADIO_TYPE 0x0E   /* Settings > Radio Type: 0 Bluetooth, 1 VEXnet */
#define NXP_REG_LANGUAGE   0x2E   /* g_languageId */
#define NXP_REG_IDLE_MIN   0x34   /* g_idleTimeoutMin */
/*
 * 0x0D (g_eeSettingByte34a) is left 0: sysDefaultEESettings sets it to 1,
 * which flips the UI 180 degrees for the brain's upside-down LCD panel.
 */

struct VexV5NxpState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t ptr;
    bool ptr_set;
    uint32_t system_id;
    uint32_t port_mask;
    uint8_t language;
    uint8_t radio_type;
};

static void vex_v5_nxp_put32(VexV5NxpState *s, uint8_t reg, uint32_t v)
{
    s->regs[reg] = v;
    s->regs[reg + 1] = v >> 8;
    s->regs[reg + 2] = v >> 16;
    s->regs[reg + 3] = v >> 24;
}

static int vex_v5_nxp_event(I2CSlave *i2c, enum i2c_event event)
{
    VexV5NxpState *s = VEX_V5_NXP(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return 0;
}

static int vex_v5_nxp_send(I2CSlave *i2c, uint8_t data)
{
    VexV5NxpState *s = VEX_V5_NXP(i2c);

    if (!s->ptr_set) {
        s->ptr = data;
        s->ptr_set = true;
        return 0;
    }
    qemu_log_mask(LOG_UNIMP, "vex-v5-nxp: reg[0x%02x] = 0x%02x\n", s->ptr, data);
    s->regs[s->ptr++] = data;
    return 0;
}

static uint8_t vex_v5_nxp_recv(I2CSlave *i2c)
{
    VexV5NxpState *s = VEX_V5_NXP(i2c);

    return s->regs[s->ptr++];
}

static void vex_v5_nxp_reset(DeviceState *dev)
{
    VexV5NxpState *s = VEX_V5_NXP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    vex_v5_nxp_put32(s, NXP_REG_PORT_MASK, s->port_mask);
    vex_v5_nxp_put32(s, NXP_REG_SYSTEM_ID, s->system_id);
    s->regs[NXP_REG_RADIO_TYPE] = s->radio_type;
    s->regs[NXP_REG_LANGUAGE] = s->language;
    s->regs[NXP_REG_IDLE_MIN] = 3;
    s->ptr = 0;
    s->ptr_set = false;
}

static const Property vex_v5_nxp_props[] = {
    DEFINE_PROP_UINT32("system-id", VexV5NxpState, system_id, 0x0005E1F0),
    DEFINE_PROP_UINT32("port-mask", VexV5NxpState, port_mask, 0),
    DEFINE_PROP_UINT8("language", VexV5NxpState, language, 2),
    /* VEXnet, so the robot radio links a controller over VEXnet (header "V") */
    DEFINE_PROP_UINT8("radio-type", VexV5NxpState, radio_type, 1),
};

static void vex_v5_nxp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = vex_v5_nxp_event;
    k->send = vex_v5_nxp_send;
    k->recv = vex_v5_nxp_recv;
    device_class_set_legacy_reset(dc, vex_v5_nxp_reset);
    device_class_set_props(dc, vex_v5_nxp_props);
    dc->desc = "VEX V5 brain NXP co-processor";
}

static const TypeInfo vex_v5_nxp_info = {
    .name = TYPE_VEX_V5_NXP,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(VexV5NxpState),
    .class_init = vex_v5_nxp_class_init,
};

static void vex_v5_nxp_register_types(void)
{
    type_register_static(&vex_v5_nxp_info);
}

type_init(vex_v5_nxp_register_types)
