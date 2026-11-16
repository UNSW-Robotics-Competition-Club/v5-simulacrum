// AI Vision test (blocks.json: a red block ahead-left, a blue one ahead-right).
// With a red colour signature, turn until the red block is centred in the
// camera image, then drive at it with the intake running until it's taken.
#include "vex.h"

#include <cstdlib>

using namespace vex;

brain Brain;
aivision::colordesc RedBlock(1, 200, 25, 25, 15, 0.30);
aivision Eye(PORT10, RedBlock);
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, false);
motor LeftBack(PORT3, ratio6_1, false);
motor RightFront(PORT4, ratio6_1, true);
motor RightMiddle(PORT5, ratio6_1, true);
motor RightBack(PORT6, ratio6_1, true);
motor Intake(PORT7, ratio18_1, false);
motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);

static void drive(double l, double r) {
  LeftDrive.spin(forward, l, percent);
  RightDrive.spin(forward, r, percent);
}

static void show(const char *phase, int count) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("aivision: %s", phase);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("red objects %d", count);
  if (count > 0) {
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("centre (%d, %d) size %dx%d id %d", Eye.objects[0].centerX,
                       Eye.objects[0].centerY, Eye.objects[0].width,
                       Eye.objects[0].height, Eye.objects[0].id);
  }
}

int main() {
  wait(1000, msec);                      // let the signature reach the sensor
  int seen = 0;
  // 1: turn left until the red block is centred
  for (int t = 0; t < 6000; t += 20) {
    seen = Eye.takeSnapshot(RedBlock);
    show("aiming", seen);
    if (seen > 0 && std::abs(Eye.objects[0].centerX - 160) < 12) break;
    if (seen > 0) {
      double turn = (Eye.objects[0].centerX - 160) * 0.12;   // + = turn right
      drive(turn, -turn);
    } else {
      drive(-12, 12);                    // search: turn left
    }
    wait(20, msec);
  }
  // 2: drive at it with the intake until it disappears into the robot
  Intake.spin(forward, 100, percent);
  for (int t = 0; t < 4000; t += 20) {
    seen = Eye.takeSnapshot(RedBlock);
    show("collecting", seen);
    if (seen == 0) break;
    double steer = (Eye.objects[0].centerX - 160) * 0.08;
    drive(25 + steer, 25 - steer);
    wait(20, msec);
  }
  wait(300, msec);
  drive(0, 0);
  LeftDrive.stop(brake);
  RightDrive.stop(brake);
  Intake.stop(coast);
  while (true) {
    seen = Eye.takeSnapshot(RedBlock);
    show("done", seen);
    wait(100, msec);
  }
}
