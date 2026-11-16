// Legacy 3-wire analog devices on the brain's ports: gyro A, pot B (on the
// motor 7 shaft), line tracker C, light sensor D, accelerometer E. The
// robot calibrates the gyro, turns right on the spot and moves the pot's
// motor to 100 degrees; each reading is printed in VEXcode's units.
#include "vex.h"

using namespace vex;

brain Brain;
gyro Gyro(Brain.ThreeWirePort.A);
pot Pot(Brain.ThreeWirePort.B);
line Line(Brain.ThreeWirePort.C);
light Light(Brain.ThreeWirePort.D);
accelerometer Accel(Brain.ThreeWirePort.E);
motor Left(PORT1), Right(PORT2, true), Arm(PORT7);

static void show(const char *phase) {
  printf("%s cal=%d gyro=%.1f pot=%.1f line=%ld light=%ld accel=%.2f\n", phase,
         (int)Gyro.isCalibrating(), Gyro.heading(degrees), Pot.angle(degrees),
         (long)Line.reflectivity(), (long)Light.brightness(),
         Accel.acceleration());
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("adi extras: %s", phase);
}

int main() {
  wait(300, msec);
  for (int t = 0; t < 2000; t += 200) { show("cal"); wait(200, msec); }
  Arm.spinToPosition(100, degrees, 60, rpm, false);
  Left.spin(forward, 30, percent);
  Right.spin(reverse, 30, percent);
  for (int t = 0; t < 4000; t += 10) {
    if (Gyro.heading(degrees) > 90 && Gyro.heading(degrees) < 180) break;
    show("turn");
    wait(10, msec);
  }
  Left.stop(brake);
  Right.stop(brake);
  while (true) { show("done"); wait(200, msec); }
}
