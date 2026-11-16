// A typical VEXcode V5 C++ student program, built with VEXcode's own SDK
// and toolchain (programs/vexcode_build.sh): a 6-motor drivetrain drives
// forward, turns, drives back, and reports encoder positions on the screen.
#include "vex.h"

using namespace vex;

brain Brain;
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
  Brain.Screen.print("VEXcode drive test: %s", phase);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("left  %8.1f deg %6.1f rpm", LeftDrive.position(degrees),
                     LeftDrive.velocity(rpm));
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("right %8.1f deg %6.1f rpm", RightDrive.position(degrees),
                     RightDrive.velocity(rpm));
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("time  %.1f s", Brain.Timer.time(seconds));
}

static void drive(const char *phase, double left_pct, double right_pct,
                  int ms) {
  LeftDrive.spin(forward, left_pct, percent);
  RightDrive.spin(forward, right_pct, percent);
  for (int t = 0; t < ms; t += 100) {
    show(phase);
    wait(100, msec);
  }
}

int main() {
  LeftDrive.setStopping(brake);
  RightDrive.setStopping(brake);

  drive("forward", 50, 50, 1500);
  drive("turn", 40, -40, 800);
  drive("back", -50, -50, 1500);

  LeftDrive.stop();
  RightDrive.stop();
  while (true) {
    show("done");
    wait(200, msec);
  }
}
