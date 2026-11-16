// VEXcode student-style driver control: arcade drive from Controller1, with
// a tracking-wheel Rotation Sensor, a front Distance Sensor and an Inertial
// Sensor shown on the brain screen, and the heading on the controller's screen.
#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1;
inertial Imu(PORT10);
rotation Tracker(PORT12);
distance Front(PORT13);
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, false);
motor LeftBack(PORT3, ratio6_1, false);
motor RightFront(PORT4, ratio6_1, true);
motor RightMiddle(PORT5, ratio6_1, true);
motor RightBack(PORT6, ratio6_1, true);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);

int main() {
  Imu.calibrate();
  while (Imu.isCalibrating()) wait(50, msec);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("driver ready");
  Controller1.rumble(".-");

  for (int n = 0;; n++) {
    int forward = Controller1.Axis3.position(percent);
    int turn = Controller1.Axis1.position(percent);
    LeftDrive.spin(vex::forward, forward + turn, percent);
    RightDrive.spin(vex::forward, forward - turn, percent);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("driver control  A:%d L1:%d", Controller1.ButtonA.pressing(),
                       Controller1.ButtonL1.pressing());
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("axis3 %4d  axis1 %4d", forward, turn);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("tracker %8.1f deg", Tracker.position(degrees));
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("distance %6.0f mm", Front.objectDistance(mm));
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("heading %7.2f", Imu.heading(degrees));
    if (n % 10 == 0) {                 // the controller takes ~1 update / 50 ms
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("hdg %6.1f", Imu.heading(degrees));
    }
    wait(20, msec);
  }
}
