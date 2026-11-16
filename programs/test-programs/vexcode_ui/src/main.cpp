// Starter template for competition UI: an autonomous selector on the brain
// screen (touch) and the controller screen (left/right arrows, A to lock in).
// Test it on a simulated field:  V5Sim ui-test path\to\vexcode_ui.bin
// The robot starts disabled, as on a real field. Pick a routine, then press
// "autonomous" under the field view (http://localhost:8765/).
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

// ---- the routines the selector offers -------------------------------------

static void drive(int left_pct, int right_pct, int ms) {
  LeftDrive.spin(forward, left_pct, percent);
  RightDrive.spin(forward, right_pct, percent);
  wait(ms, msec);
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
}

static void autonForward() { drive(50, 50, 1500); }
static void autonTurn() { drive(40, -40, 1200); }
static void autonNone() {}

struct Routine {
  const char *name;
  void (*run)();
  color tint;
};

static const Routine ROUTINES[] = {
    {"Forward", autonForward, color(30, 110, 200)},
    {"Turn", autonTurn, color(200, 60, 40)},
    {"Do nothing", autonNone, color(90, 90, 90)},
};
static const int ROUTINE_COUNT = sizeof(ROUTINES) / sizeof(ROUTINES[0]);
static int selected = 0;
static bool locked = false;          // A on the controller locks the choice

// ---- brain screen: one button per routine ---------------------------------

static const int BUTTON_W = 150, BUTTON_H = 80, BUTTON_Y = 100, GAP = 10;

static int buttonX(int i) { return GAP + i * (BUTTON_W + GAP); }

static const char *phase() {
  if (!Competition.isEnabled()) return "disabled";
  return Competition.isAutonomous() ? "autonomous" : "driver";
}

static void drawBrain() {
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(10, 30, false, "%s  %s", phase(),
                       Competition.isFieldControl()        ? "(field)"
                       : Competition.isCompetitionSwitch() ? "(switch)"
                                                           : "(no field)");
  Brain.Screen.printAt(10, 60, false, "auton: %s%s", ROUTINES[selected].name,
                       locked ? " (locked)" : "");
  for (int i = 0; i < ROUTINE_COUNT; i++) {
    Brain.Screen.setPenColor(i == selected ? white : black);
    Brain.Screen.setFillColor(ROUTINES[i].tint);
    Brain.Screen.drawRectangle(buttonX(i), BUTTON_Y, BUTTON_W, BUTTON_H);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(buttonX(i) + 10, BUTTON_Y + 45, false, "%s", ROUTINES[i].name);
  }
}

static void onBrainPress() {
  if (locked || Competition.isEnabled()) return;
  int x = Brain.Screen.xPosition(), y = Brain.Screen.yPosition();
  if (y < BUTTON_Y || y > BUTTON_Y + BUTTON_H) return;
  for (int i = 0; i < ROUTINE_COUNT; i++)
    if (x >= buttonX(i) && x <= buttonX(i) + BUTTON_W) selected = i;
}

// ---- controller screen: arrows to choose, A to lock ------------------------

static void drawController() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("%s", phase());
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("< %s >", ROUTINES[selected].name);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print(locked ? "locked" : "A to lock");
}

static void onLeft() {
  if (!locked && !Competition.isEnabled())
    selected = (selected + ROUTINE_COUNT - 1) % ROUTINE_COUNT;
}
static void onRight() {
  if (!locked && !Competition.isEnabled()) selected = (selected + 1) % ROUTINE_COUNT;
}
static void onA() { locked = !locked; }

// ---- competition callbacks ------------------------------------------------

void pre_auton() {
  Imu.calibrate();
  while (Imu.isCalibrating()) wait(50, msec);
}

void autonomous() { ROUTINES[selected].run(); }

void usercontrol() {
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
  Brain.Screen.pressed(onBrainPress);
  Controller1.ButtonLeft.pressed(onLeft);
  Controller1.ButtonRight.pressed(onRight);
  Controller1.ButtonA.pressed(onA);
  pre_auton();
  int ticks = 0;
  while (true) {
    drawBrain();
    if (ticks++ % 5 == 0) drawController();   // the controller screen is slow
    wait(100, msec);
  }
}
