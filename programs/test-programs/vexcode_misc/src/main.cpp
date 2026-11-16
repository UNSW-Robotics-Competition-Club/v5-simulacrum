// Misc smart devices: an Electromagnet on port 3, a Light Tower on 4, CTE
// pneumatics on 5 and an LED on 6. Each one is driven through the SDK and
// read back, so the output shows what the brain parsed from the device.
// An Optical Sensor on 8 in gesture mode counts swipes, and the tower's
// button state is printed (both driven from the field view's /input).
#include "vex.h"

using namespace vex;

brain Brain;
cte::signaltower Tower(PORT4);
cte::pneumatic Pneu(PORT5);
device Magnet(PORT3), Led(PORT6);
optical Eye(PORT8);
static int ups, lefts;
static void onUp() { ups++; }
static void onLeft() { lefts++; }

static void show(const char *phase) {
  uint16_t c1, c2, c3, c4;
  vexPneumaticActuationStatusGet(PORT5, &c1, &c2, &c3, &c4);
  printf("%s types=%d/%d/%d/%d magnet power=%ld mA=%.0f temp=%.0f st=%lu "
         "tower rgb=%06lx xyw=%04lx pneu st=%04lx count=%u/%u led=%06lx "
         "pressing=%d ups=%d lefts=%d\n",
         phase, (int)Magnet.type(), (int)Tower.type(), (int)Pneu.type(),
         (int)Led.type(), (long)vexMagnetPowerGet(PORT3),
         vexMagnetCurrentGet(PORT3), vexMagnetTemperatureGet(PORT3),
         (unsigned long)vexMagnetStatusGet(PORT3),
         (unsigned long)vexLightTowerRgbGet(PORT4),
         (unsigned long)vexLightTowerXywGet(PORT4),
         (unsigned long)vexPneumaticStatusGet(PORT5), c1, c2,
         (unsigned long)vexLedRgbGet(PORT6), (int)Tower.pressing(), ups, lefts);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("misc devices: %s", phase);
}

int main() {
  Eye.gestureEnable();
  Eye.gestureUp(onUp);
  Eye.gestureLeft(onLeft);
  for (int t = 0; t < 3000; t += 200) { show("idle"); wait(200, msec); }
  vexMagnetPickup(PORT3, kMagnetDurationLong);       // +50 for 1000 ms
  Tower.setColor(255, 0, 0, 0, 128);                 // red + white
  Tower.setBlink(cte::signaltower::ledId::red);
  Pneu.pumpOn();
  Pneu.extend(cylinderType::cylinder1);
  vexLedRgbSet(PORT6, 0x00FF80);
  for (int t = 0; t < 800; t += 100) { show("on"); wait(100, msec); }
  Pneu.retract(cylinderType::cylinder1);
  Pneu.extend(cylinderType::cylinder2);
  wait(800, msec);
  while (true) { show("done"); wait(100, msec); }
}
