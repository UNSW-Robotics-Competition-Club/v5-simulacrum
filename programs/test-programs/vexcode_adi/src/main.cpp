// VEXcode 3-wire (ADI) test on the brain's built-in ports A..H: drive at the
// wall until the front bumper (A) hits, fire the pneumatic piston (C), then
// back up one tracking-wheel turn on the legacy encoder (E/F) while the
// ultrasonic (G/H) watches the wall recede.
#include "vex.h"

using namespace vex;

brain Brain;
bumper Bump(Brain.ThreeWirePort.A);
pneumatics Piston(Brain.ThreeWirePort.C);
encoder Track(Brain.ThreeWirePort.E);
sonar Sonar(Brain.ThreeWirePort.G);
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, false);
motor LeftBack(PORT3, ratio6_1, false);
motor RightFront(PORT4, ratio6_1, true);
motor RightMiddle(PORT5, ratio6_1, true);
motor RightBack(PORT6, ratio6_1, true);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);

static void show(const char *phase) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("3-wire test: %s", phase);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("bumper %d  piston %d", (int)Bump.pressing(),
                     (int)Piston.value());
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("encoder %8.1f deg", Track.position(degrees));
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("sonar %7.0f mm", Sonar.distance(mm));
}

static void drive(double pct) {
  LeftDrive.spin(forward, pct, percent);
  RightDrive.spin(forward, pct, percent);
}

int main() {
  LeftDrive.setStopping(brake);
  RightDrive.setStopping(brake);
  wait(500, msec);                  // let the 3-wire config reach the device

  drive(40);
  while (!Bump.pressing()) {
    show("to the wall");
    wait(20, msec);
  }
  drive(0);
  Piston.open();

  double start = Track.position(degrees);
  drive(-30);
  while (Track.position(degrees) > start - 360) {
    show("backing up");
    wait(20, msec);
  }
  LeftDrive.stop();
  RightDrive.stop();
  while (true) {
    show("done");
    wait(100, msec);
  }
}
