// 5.5 W motor check: the brain reports the motor type (1 = 5.5 W), the
// motor spins at 100 rpm, then moves to an absolute 720 degrees and holds.
#include "vex.h"

using namespace vex;

brain Brain;
motor Small(PORT7);

static void show(const char *phase) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("5.5W motor: %s", phase);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("type %d  installed %d", (int)Small.getMotorType(),
                     (int)Small.installed());
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("vel %6.1f rpm  pos %7.1f deg", Small.velocity(rpm),
                     Small.position(degrees));
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("torque %.2f Nm  current %.2f A", Small.torque(Nm),
                     Small.current(amp));
  printf("%s type=%d vel=%.1f pos=%.1f\n", phase, (int)Small.getMotorType(),
         Small.velocity(rpm), Small.position(degrees));
}

int main() {
  wait(500, msec);
  Small.spin(forward, 100, rpm);
  for (int t = 0; t < 2000; t += 100) { show("spin 100 rpm"); wait(100, msec); }
  Small.spinToPosition(720, degrees, 150, rpm, true);
  for (int t = 0; t < 1500; t += 100) { show("at 720"); wait(100, msec); }
  while (true) { show("done"); wait(200, msec); }
}
