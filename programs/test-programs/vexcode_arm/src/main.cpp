// A loaded arm (sim: --motor 8:red --arm 8:5:12:1.5:-30:-30:90, a 5:1
// reduction to a 12 in arm with 1.5 kg, resting on its -30 degree stop).
// Lift it to level (motor 150 deg = arm 0 deg), hold it there against
// gravity, then let it go: it should fall back to the stop.
#include "vex.h"

using namespace vex;

brain Brain;
motor Arm(PORT8, ratio36_1, false);

static void show(const char *phase) {
  printf("%s pos=%.1f vel=%.1f amps=%.2f volts=%.2f\n", phase, Arm.position(degrees),
         Arm.velocity(rpm), Arm.current(amp), Arm.voltage(volt));
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("arm: %s", phase);
}

int main() {
  wait(500, msec);
  Arm.setStopping(hold);
  Arm.spinToPosition(150, degrees, 60, rpm, true);   // arm level
  for (int t = 0; t < 2000; t += 250) { show("hold"); wait(250, msec); }
  Arm.setStopping(coast);
  Arm.stop();
  for (int t = 0; t < 1500; t += 250) { show("coast"); wait(250, msec); }
  while (true) { show("done"); wait(250, msec); }
}
