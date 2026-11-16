/* Build check for the generated SDK, compiled as C11 (scripts/sdk_test.sh). */
#include "v5_sdk.h"

void sample_cpp(void);

void sample_main(void) {
    V5_DeviceT motor = vexDeviceGetByIndex(0);
    vexDeviceMotorVelocitySet(motor, 100);
    vexDeviceMotorPositionSet(motor, 1.5);
    double pos = vexDeviceMotorPositionGet(motor);
    vexDeviceMotorBrakeModeSet(motor, kV5MotorBrakeModeHold);

    V5_DeviceMotorPid pid = {0};
    vexDeviceMotorVelocityPidSet(motor, &pid);

    struct time now;
    vexGettime(&now);
    double heading = vexDeviceGpsHeadingGet(motor);  /* VEX-only slot */
    vexDebug("gps %d\n", (int)heading);
    vexDisplayForegroundColor(0xFF0000);
    vexDisplayRectFill(0, 0, 100, 50);
    vexDisplayPrintf(10, 10, 1, "pos %d at %u:%u", (int)pos, now.ti_min, now.ti_sec);
    vex_printf("t=%lu v=%lu\n", (unsigned long)vexSystemTimeGet(), (unsigned long)vexSystemVersion());

    sample_cpp();
    vexTaskSleep(10);
}
