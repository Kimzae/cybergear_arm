// =============================================================================
// cybergear_protocol.cpp  —  cybergear_protocol.hpp 구현
// =============================================================================
#include "cybergear_arm/cybergear_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace cybergear {

uint16_t floatToUint16(double x, double x_min, double x_max) {
  x = std::clamp(x, x_min, x_max);
  return static_cast<uint16_t>((x - x_min) * 65535.0 / (x_max - x_min));
}

double uint16ToFloat(uint16_t u, double x_min, double x_max) {
  return static_cast<double>(u) * (x_max - x_min) / 65535.0 + x_min;
}

// 29비트 ID 조립:  타입(5bit) | 데이터영역2(16bit) | 모터ID(8bit)
static uint32_t makeId(uint8_t type, uint16_t data16, uint8_t motor_id) {
  return (static_cast<uint32_t>(type & 0x1F) << 24) |
         (static_cast<uint32_t>(data16) << 8) | motor_id;
}

// 16비트 값을 big-endian(상위 바이트 먼저)으로 기록
static void putU16BE(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}
static uint16_t getU16BE(const uint8_t* p) {
  return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

CanFrame makeEnable(uint8_t motor_id, uint8_t host_id) {
  CanFrame f;
  f.id = makeId(kEnable, host_id, motor_id);
  return f;
}

CanFrame makeStop(uint8_t motor_id, uint8_t host_id, bool clear_fault) {
  CanFrame f;
  f.id = makeId(kStop, host_id, motor_id);
  f.data[0] = clear_fault ? 1 : 0;
  return f;
}

CanFrame makeSetZero(uint8_t motor_id, uint8_t host_id) {
  CanFrame f;
  f.id = makeId(kSetZero, host_id, motor_id);
  f.data[0] = 1;
  return f;
}

CanFrame makeGetId(uint8_t motor_id, uint8_t host_id) {
  CanFrame f;
  f.id = makeId(kGetId, host_id, motor_id);
  return f;
}

// 파라미터 쓰기 데이터:  [index(LE 2B)] [0x0000] [value(LE 4B)]
CanFrame makeWriteParamU8(uint8_t motor_id, uint8_t host_id, uint16_t index, uint8_t value) {
  CanFrame f;
  f.id = makeId(kWriteParam, host_id, motor_id);
  f.data[0] = index & 0xFF;
  f.data[1] = index >> 8;
  f.data[4] = value;
  return f;
}

CanFrame makeWriteParamFloat(uint8_t motor_id, uint8_t host_id, uint16_t index, float value) {
  CanFrame f;
  f.id = makeId(kWriteParam, host_id, motor_id);
  f.data[0] = index & 0xFF;
  f.data[1] = index >> 8;
  std::memcpy(&f.data[4], &value, 4);  // x86/ARM 모두 little-endian
  return f;
}

CanFrame makeMotion(uint8_t motor_id, double torque, double position, double velocity,
                    double kp, double kd) {
  CanFrame f;
  // 토크는 ID의 데이터영역2(16비트)에 들어간다 — CyberGear 프로토콜의 특이한 점
  f.id = makeId(kMotion, floatToUint16(torque, kTorMin, kTorMax), motor_id);
  putU16BE(&f.data[0], floatToUint16(position, kPosMin, kPosMax));
  putU16BE(&f.data[2], floatToUint16(velocity, kVelMin, kVelMax));
  putU16BE(&f.data[4], floatToUint16(kp, kKpMin, kKpMax));
  putU16BE(&f.data[6], floatToUint16(kd, kKdMin, kKdMax));
  return f;
}

std::optional<MotorFeedback> parseFeedback(const CanFrame& f) {
  if (commTypeOf(f.id) != kFeedback || f.len < 8) return std::nullopt;
  MotorFeedback fb;
  fb.motor_id = (f.id >> 8) & 0xFF;   // 응답한 모터의 ID
  fb.fault = (f.id >> 16) & 0x3F;
  fb.mode = (f.id >> 22) & 0x03;
  fb.position = uint16ToFloat(getU16BE(&f.data[0]), kPosMin, kPosMax);
  fb.velocity = uint16ToFloat(getU16BE(&f.data[2]), kVelMin, kVelMax);
  fb.torque = uint16ToFloat(getU16BE(&f.data[4]), kTorMin, kTorMax);
  fb.temperature = getU16BE(&f.data[6]) / 10.0;
  return fb;
}

}  // namespace cybergear
