/* Minimal repro: enable generic serial on port 1 and spin. A host client
 * sends bytes to the port socket; the RX interrupt (CPU1 INTC1) then fires. */
#include "v5_sdk.h"
void *memset(void *d, int c, unsigned int n)
{ unsigned char *p = d; while (n--) *p++ = (unsigned char)c; return d; }
static void wait_ms(uint32_t ms)
{ uint32_t e = vexSystemTimeGet() + ms; while (vexSystemTimeGet() < e) vexTasksRun(); }
void user_main(void)
{
    V5_DeviceT dev = vexDeviceGetByIndex(0);
    vexDisplayString(1, "gsrx: generic serial port 1");
    vexDeviceGenericSerialEnable(dev, 0);
    vexDeviceGenericSerialBaudrate(dev, 115200);
    uint8_t buf[64];
    for (;;) {
        int n = vexDeviceGenericSerialReceive(dev, buf, sizeof buf);
        vexDisplayString(2, "rx total tick %d n %d", (int)vexSystemTimeGet(), n);
        wait_ms(50);
    }
}
