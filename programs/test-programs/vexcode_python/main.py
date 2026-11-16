# VEXcode V5 Python: a student-style drive test. Forward 1.5 s, turn,
# report positions on the screen and over print (the USB user port).
from vex import *

brain = Brain()
left = MotorGroup(Motor(Ports.PORT1, GearSetting.RATIO_6_1, False),
                  Motor(Ports.PORT2, GearSetting.RATIO_6_1, False),
                  Motor(Ports.PORT3, GearSetting.RATIO_6_1, False))
right = MotorGroup(Motor(Ports.PORT4, GearSetting.RATIO_6_1, True),
                   Motor(Ports.PORT5, GearSetting.RATIO_6_1, True),
                   Motor(Ports.PORT6, GearSetting.RATIO_6_1, True))

def drive(phase, l, r, ms):
    left.spin(FORWARD, l, PERCENT)
    right.spin(FORWARD, r, PERCENT)
    wait(ms, MSEC)
    print(phase, left.position(DEGREES), right.position(DEGREES))
    brain.screen.print_at(phase, x=10, y=40)

print("python start")
drive("forward", 50, 50, 1500)
drive("turn", 40, -40, 800)
left.stop(BRAKE)
right.stop(BRAKE)
print("python done")
brain.screen.print_at("python done", x=10, y=80)
