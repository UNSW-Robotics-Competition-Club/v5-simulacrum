// VEXcode student-style closed-loop turn with the Inertial Sensor on port 10:
// calibrate, turn to 90 degrees with a P controller, then hold and report.
#include "vex.h"

using namespace vex;

brain Brain;
inertial Imu(PORT10);
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, false);
motor LeftBack(PORT3, ratio6_1, false);
motor RightFront(PORT4, ratio6_1, true);
motor RightMiddle(PORT5, ratio6_1, true);
motor RightBack(PORT6, ratio6_1, true);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);

static const double TARGET_DEG = 90.0;
static const double KP = 0.6;         // percent per degree of error
static const double DONE_DEG = 1.0;

static void show(const char *phase) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("IMU turn test: %s", phase);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("heading  %7.2f", Imu.heading(degrees));
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("rotation %7.2f", Imu.rotation(degrees));
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("gyro z   %7.2f dps", Imu.gyroRate(zaxis, dps));
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("accel z  %7.3f g", Imu.acceleration(zaxis));
}

int main() {
  Imu.calibrate();
  while (Imu.isCalibrating()) {
    show("calibrating");
    wait(50, msec);
  }

  for (;;) {
    double err = TARGET_DEG - Imu.rotation(degrees);
    if (fabs(err) < DONE_DEG && fabs(Imu.gyroRate(zaxis, dps)) < 5) break;
    double out = fmax(-60, fmin(60, KP * err));
    LeftDrive.spin(forward, out, percent);
    RightDrive.spin(forward, -out, percent);
    show("turning");
    wait(20, msec);
  }
  LeftDrive.stop(hold);
  RightDrive.stop(hold);
  while (true) {
    show("done");
    wait(100, msec);
  }
}
