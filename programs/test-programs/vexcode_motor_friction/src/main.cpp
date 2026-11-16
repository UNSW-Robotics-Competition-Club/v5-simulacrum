// Gearbox friction, measured on a real motor (sim/v5sim/motor.py
// FREE_CURRENT_A and BREAKAWAY_FACTOR, notes/motor-emulation.md).
// Put one motor on port 1 with nothing on its shaft and run this. It prints
// to the brain screen and to the terminal (VEXcode's output / PROS terminal):
//   free   current and speed spinning free at 12 V, in both directions
//   start  the smallest voltage that gets a stopped motor turning
// To calibrate, run it on a real motor and in the sim (--motor 1:CART,
// with or without --motor-fw 1) and tune FREE_CURRENT_A (free current) and
// BREAKAWAY_FACTOR (start volts) until the sim prints the same. With the
// estimates, an 11 W green in the sim prints about 0.195 A / 195 rpm free
// and 0.85 V to start.
#include <cstdarg>

#include "vex.h"

using namespace vex;

brain Brain;
motor M(PORT1);

static int row = 1;

static void report(const char *fmt, ...) {
  char text[80];
  va_list args;
  va_start(args, fmt);
  vsnprintf(text, sizeof text, fmt, args);
  va_end(args);
  printf("%s\n", text);
  Brain.Screen.setCursor(row++, 1);
  Brain.Screen.print("%s", text);
}

// Spin free at +-12 V; average the second half of 3 s.
static void free_run(double volts) {
  M.spin(forward, volts, voltageUnits::volt);
  wait(1500, msec);
  double amps = 0, speed = 0;
  int n = 0;
  for (int t = 0; t < 1500; t += 20, n++) {
    amps += M.current(amp);
    speed += M.velocity(rpm);
    wait(20, msec);
  }
  M.stop(coast);
  wait(1500, msec);                          // let it run down
  report(volts > 0 ? "free fwd %.3f A %.1f rpm" : "free rev %.3f A %.1f rpm",
         amps / n, speed / n);
}

// Raise the voltage in 25 mV steps, 300 ms each, until it turns.
static void breakaway(double sign) {
  M.stop(coast);
  wait(1000, msec);
  double start = M.position(degrees);
  for (double v = 0.0; v <= 3.0; v += 0.025) {
    M.spin(forward, sign * v, voltageUnits::volt);
    wait(300, msec);
    if (fabs(M.position(degrees) - start) > 2.0 || fabs(M.velocity(rpm)) > 1.0) {
      M.stop(coast);
      report(sign > 0 ? "start fwd %.3f V" : "start rev %.3f V", v);
      return;
    }
  }
  M.stop(coast);
  report("start %s: no movement up to 3 V", sign > 0 ? "fwd" : "rev");
}

int main() {
  Brain.Screen.clearScreen();
  wait(500, msec);
  report("motor type %d (0 = 11 W, 1 = 5.5 W)", (int)M.getMotorType());
  free_run(12.0);
  free_run(-12.0);
  for (int i = 0; i < 3; i++) {              // repeat: it varies a little
    breakaway(1.0);
    breakaway(-1.0);
  }
  report("done");
  while (true) {
    wait(1, seconds);
  }
}
