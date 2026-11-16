// Game-object test (blocks.json): drive forward through a line of three
// blocks with the intake running, turn around, and eject them. A fourth
// block beside the path gets plowed aside, not intaken.
#include "vex.h"

using namespace vex;

brain Brain;
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, false);
motor LeftBack(PORT3, ratio6_1, false);
motor RightFront(PORT4, ratio6_1, true);
motor RightMiddle(PORT5, ratio6_1, true);
motor RightBack(PORT6, ratio6_1, true);
motor Intake(PORT7, ratio18_1, false);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);

static void show(const char *phase) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("intake test: %s", phase);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("intake %6.1f rpm %5.2f A", Intake.velocity(rpm),
                     Intake.current(amp));
}

static void drive(const char *phase, double l, double r, int ms) {
  LeftDrive.spin(forward, l, percent);
  RightDrive.spin(forward, r, percent);
  for (int t = 0; t < ms; t += 50) {
    show(phase);
    wait(50, msec);
  }
}

int main() {
  LeftDrive.setStopping(brake);
  RightDrive.setStopping(brake);
  Intake.spin(forward, 100, percent);
  drive("collect", 30, 30, 1600);
  drive("stop", 0, 0, 300);
  Intake.stop(coast);
  drive("turn", 30, -30, 1100);
  drive("stop", 0, 0, 300);
  Intake.spin(reverse, 100, percent);
  drive("eject", 0, 0, 1200);
  Intake.stop(coast);
  while (true) {
    show("done");
    wait(200, msec);
  }
}
