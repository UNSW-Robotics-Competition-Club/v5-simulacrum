// AI Vision AprilTags and colour codes (tags.json, blocks.json: tag 3 on
// the far wall ahead, a red block left of a blue one in front). Prints the
// tag's id, centre and corners and the red-blue code's box, then turns
// right a little: the tag should move left in the image.
#include "vex.h"

using namespace vex;

brain Brain;
aivision::colordesc Red(1, 200, 25, 25, 15, 0.30);
aivision::colordesc Blue(2, 25, 60, 200, 15, 0.30);
aivision::codedesc RedBlue(1, Red, Blue);
aivision Eye(PORT10, Red, Blue, RedBlue, aivision::ALL_TAGS);
motor Left(PORT1), Right(PORT2, true);

static void show(const char *phase) {
  int tags = Eye.takeSnapshot(aivision::ALL_TAGS);
  if (tags > 0) {
    aivision::object &t = Eye.objects[0];
    printf("%s tags=%d id=%d centre=(%d,%d) size=%dx%d angle=%.1f "
           "bl=(%d,%d) tr=(%d,%d)\n", phase, tags, (int)t.id, (int)t.centerX,
           (int)t.centerY, (int)t.width, (int)t.height, t.angle, t.tag.x[0],
           t.tag.y[0], t.tag.x[2], t.tag.y[2]);
  } else {
    printf("%s tags=0\n", phase);
  }
  int codes = Eye.takeSnapshot(RedBlue);
  if (codes > 0) {
    aivision::object &c = Eye.objects[0];
    printf("%s codes=%d id=%d origin=(%d,%d) size=%dx%d angle=%.1f\n", phase,
           codes, (int)c.id, (int)c.originX, (int)c.originY, (int)c.width,
           (int)c.height, c.angle);
  } else {
    printf("%s codes=0\n", phase);
  }
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("aitags: %s tags %d codes %d", phase, tags, codes);
}

int main() {
  wait(1000, msec);
  for (int t = 0; t < 2000; t += 250) { show("ahead"); wait(250, msec); }
  Left.spin(forward, 10, percent);
  Right.spin(reverse, 10, percent);
  wait(300, msec);
  Left.stop(brake);
  Right.stop(brake);
  while (true) { show("turned"); wait(250, msec); }
}
