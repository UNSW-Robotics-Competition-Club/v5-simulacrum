// Build check for the generated SDK, compiled as C++17 (scripts/sdk_test.sh).
#include "v5_sdk.h"

namespace {
struct Motor {
    explicit Motor(uint32_t port) : dev_(vexDeviceGetByIndex(port)) {}
    void spin(int32_t rpm) const { vexDeviceMotorVelocitySet(dev_, rpm); }
    double position() const { return vexDeviceMotorPositionGet(dev_); }
    V5_DeviceT dev_;
};

static_assert(sizeof(V5_DeviceMotorPid) == 12, "V5_DeviceMotorPid layout");
static_assert(sizeof(vcodesig) == 16, "vcodesig layout");
}  // namespace

extern "C" void sample_cpp(void) {
    Motor m(1);
    m.spin(200);
    V5_DeviceAiVisionObject obj{};
    vexDeviceAiVisionObjectGet(m.dev_, 0, &obj);
    vexDisplayString(2, "pos %.2f type %u", m.position(), obj.type);
}
