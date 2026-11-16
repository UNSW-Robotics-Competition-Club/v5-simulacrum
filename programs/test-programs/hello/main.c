/*
 * Smoke test for user programs on the emulated brain: shows a live clock
 * and the port 1 motor's telemetry, and drives that motor with a voltage
 * that flips sign every 2 s.
 */
#include "v5_sdk.h"

#define MOTOR_PORT_INDEX  0         /* smart port 1 */
#define DRIVE_MV          6000
#define FLIP_MS           2000
#define LOOP_MS           20

/*
 * The program's main context isn't a VEXos task (vexTaskSleep would write
 * through a null current-task pointer), so wait by running the scheduler.
 */
static void wait_ms(uint32_t ms)
{
    uint32_t end = vexSystemTimeGet() + ms;

    while (vexSystemTimeGet() < end) {
        vexTasksRun();
    }
}

void user_main(void)
{
    V5_DeviceT motor = vexDeviceGetByIndex(MOTOR_PORT_INDEX);

    vexDisplayErase();
    for (;;) {
        uint32_t now = vexSystemTimeGet();
        int32_t mv = (now / FLIP_MS) % 2 ? -DRIVE_MV : DRIVE_MV;

        vexDeviceMotorVoltageSet(motor, mv);
        vexDisplayForegroundColor(0xFFFFFF);
        vexDisplayString(1, "Hello from CPU1");
        vexDisplayString(2, "time  %lu ms    ", (unsigned long)now);
        vexDisplayString(3, "cmd   %ld mV    ", (long)mv);
        vexDisplayString(4, "pos   %d deg    ", (int)vexDeviceMotorPositionGet(motor));
        vexDisplayString(5, "vel   %d rpm    ", (int)vexDeviceMotorActualVelocityGet(motor));
        wait_ms(LOOP_MS);
    }
}
