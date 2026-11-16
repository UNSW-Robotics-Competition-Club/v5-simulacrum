// Program stdio over USB: printf goes to the brain's user serial port (the
// "terminal" in VEXcode / `pros terminal`), and characters typed there come
// back through getchar-style reads and are echoed in upper case.
#include "vex.h"

#include <cctype>
#include <cstdio>

using namespace vex;

brain Brain;

int main() {
  int n = 0;
  while (true) {
    printf("tick %d time %.1f\n", n, Brain.Timer.time(seconds));
    fflush(stdout);
    int c;
    while ((c = vexSerialReadChar(1)) >= 0) {   // channel 1 = user port
      printf("echo %c\n", toupper(c));
      fflush(stdout);
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("printf test: tick %d", n);
    n++;
    wait(500, msec);
  }
}
