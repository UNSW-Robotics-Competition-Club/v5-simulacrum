// Legacy Vision Sensor on port 11 with a Vision Utility style red signature
// (blocks.json: a red block ahead-left, a blue one ahead-right). Turn until
// the red block is centred in the 316x212 image, then stop.
#include "vex.h"

#include <cstdlib>

using namespace vex;

brain Brain;
vision::signature RED(1, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
vision Cam(PORT11, 50, RED);
motor Left(PORT1), Right(PORT2, true);

static int look(const char *phase) {
  int n = Cam.takeSnapshot(RED);
  if (n > 0)
    printf("%s n=%d id=%d centre=(%d,%d) size=%dx%d\n", phase, n,
           (int)Cam.objects[0].id, (int)Cam.objects[0].centerX,
           (int)Cam.objects[0].centerY, (int)Cam.objects[0].width,
           (int)Cam.objects[0].height);
  else
    printf("%s n=0\n", phase);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("vision: %s red %d", phase, n);
  return n;
}

int main() {
  wait(1000, msec);
  for (int t = 0; t < 1500; t += 250) { look("start"); wait(250, msec); }
  for (int t = 0; t < 5000; t += 50) {
    if (look("aim") > 0 && std::abs(Cam.objects[0].centerX - 158) < 6) break;
    Left.spin(reverse, 8, percent);              // turn left toward it
    Right.spin(forward, 8, percent);
    wait(50, msec);
  }
  Left.stop(brake);
  Right.stop(brake);
  while (true) { look("done"); wait(250, msec); }
}
