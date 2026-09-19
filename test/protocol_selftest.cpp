// =============================================================================
// protocol_selftest.cpp
//   하드웨어 없이 프로토콜 인코딩이 Python 버전과 같은지 확인.
//   실행: ros2 run cybergear_arm protocol_selftest
// =============================================================================
#include <cmath>
#include <cstdio>

#include "cybergear_arm/cybergear_protocol.hpp"

using namespace cybergear;

static int fails = 0;
#define CHECK(cond)                                              \
  do {                                                           \
    if (!(cond)) { std::printf("FAIL: %s (line %d)\n", #cond, __LINE__); ++fails; } \
  } while (0)

int main() {
  // Python: enable(motor 127, host 0xFD) -> 시리얼 바이트 41 54 18 07 eb fc ...
  //         (0x1807EBFC >> 3) = 0x0300FD7F
  auto en = makeEnable(127, 0xFD);
  CHECK(en.id == 0x0300FD7Fu);
  CHECK(((en.id << 3) | 4) == 0x1807EBFCu);

  // Python: motion_control(0.0) -> 41 54 0b ff fb fc 08 7f ff 7f ff 00 00 00 00
  auto mo = makeMotion(127, 0.0);
  CHECK(((mo.id << 3) | 4) == 0x0BFFFBFCu);
  CHECK(mo.data[0] == 0x7F && mo.data[1] == 0xFF && mo.data[2] == 0x7F && mo.data[3] == 0xFF);

  // 피드백 해석 왕복 테스트
  CanFrame fb;
  fb.id = (2u << 24) | (2u << 22) | (1u << 8) | 0xFD;
  const uint16_t p = floatToUint16(1.0, kPosMin, kPosMax);
  fb.data = {uint8_t(p >> 8), uint8_t(p & 0xFF), 0x7F, 0xFF, 0x80, 0x00, 0x01, 0x2C};
  auto r = parseFeedback(fb);
  CHECK(r.has_value());
  CHECK(r->motor_id == 1 && r->mode == 2 && r->fault == 0);
  CHECK(std::fabs(r->position - 1.0) < 1e-3);
  CHECK(std::fabs(r->temperature - 30.0) < 1e-9);

  std::printf(fails ? "%d test(s) FAILED\n" : "all protocol tests passed\n", fails);
  return fails ? 1 : 0;
}
