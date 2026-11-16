// The VEXcode V5 competition template: pre_auton calibrates the IMU, the
// autonomous callback drives forward for 1.5 s, and the driver control
// callback runs arcade drive from Controller1. The field (here the sim's
// --match schedule) decides which callback runs and when.
#include "vex.h"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1;
inertial Imu(PORT10);
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, false);
motor LeftBack(PORT3, ratio6_1, false);
motor RightFront(PORT4, ratio6_1, true);
motor RightMiddle(PORT5, ratio6_1, true);
motor RightBack(PORT6, ratio6_1, true);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);

static int auton_runs = 0;
static int driver_runs = 0;

static const char *phase() {
  if (!Competition.isEnabled()) return "disabled";
  return Competition.isAutonomous() ? "autonomous" : "driver";
}

void pre_auton() {
  Imu.calibrate();
  while (Imu.isCalibrating()) wait(50, msec);
}

void autonomous() {
  auton_runs++;
  LeftDrive.spin(forward, 50, percent);
  RightDrive.spin(forward, 50, percent);
  wait(1500, msec);
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

void usercontrol() {
  driver_runs++;
  while (true) {
    int fwd = Controller1.Axis3.position(percent);
    int turn = Controller1.Axis1.position(percent);
    LeftDrive.spin(forward, fwd + turn, percent);
    RightDrive.spin(forward, fwd - turn, percent);
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();

  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("competition: %s%s", phase(),
                       Competition.isCompetitionSwitch() ||
                               Competition.isFieldControl()
                           ? " (field)" : "");
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("autonomous runs %d  driver runs %d", auton_runs,
                       driver_runs);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("heading %7.2f", Imu.heading(degrees));
    wait(100, msec);
  }
}
