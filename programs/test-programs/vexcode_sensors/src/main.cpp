// Optical + GPS test (blocks.json: one blue block 0.7 m ahead). Drive with
// the intake running until the optical sensor sees an object, name its
// colour from the hue, keep intaking briefly, then report what the GPS says
// about where the robot is.
#include "vex.h"

using namespace vex;

brain Brain;
optical Eye(PORT8);
gps Gps(PORT9);
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, false);
motor LeftBack(PORT3, ratio6_1, false);
motor RightFront(PORT4, ratio6_1, true);
motor RightMiddle(PORT5, ratio6_1, true);
motor RightBack(PORT6, ratio6_1, true);
motor Intake(PORT7, ratio18_1, false);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);

static const char *colour_name(double hue) {
  if (hue < 30 || hue > 330) return "red";
  if (hue > 180 && hue < 260) return "blue";
  return "other";
}

static void show(const char *phase, double seen_hue) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("sensors: %s", phase);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("optical hue %5.1f near %d bright %.2f", Eye.hue(),
                     (int)Eye.isNearObject(), Eye.brightness());
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("saw %s (hue %.1f)", colour_name(seen_hue), seen_hue);
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("gps x %6.0f mm y %6.0f mm h %5.1f q %d",
                     Gps.xPosition(mm), Gps.yPosition(mm), Gps.heading(),
                     (int)Gps.quality());
}

int main() {
  Eye.setLight(ledState::on);
  wait(500, msec);
  Intake.spin(forward, 100, percent);
  LeftDrive.spin(forward, 25, percent);
  RightDrive.spin(forward, 25, percent);
  double seen = -1;
  for (int t = 0; t < 4000 && !Eye.isNearObject(); t += 20) {
    show("looking", seen);
    wait(20, msec);
  }
  seen = Eye.hue();
  wait(400, msec);                    // keep driving into it: intake takes it
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
  Intake.stop(coast);
  while (true) {
    show("done", seen);
    wait(100, msec);
  }
}
