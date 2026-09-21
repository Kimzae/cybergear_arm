// =============================================================================
// cybergear_protocol.hpp
//   CyberGear "사설 프로토콜" 인코딩/디코딩 (하드웨어·ROS와 무관한 순수 로직)
//   Python 버전 cybergear_protocol.py 를 C++로 옮긴 것.
//
//   CyberGear 29비트 확장 CAN ID 구조
//     [28:24] 통신 타입 | [23:8] 데이터 영역2 (호스트 ID 또는 토크) | [7:0] 목표 모터 ID
// =============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace cybergear {

// ---- 통신 타입 -------------------------------------------------------------
enum CommType : uint8_t {
  kGetId = 0,         // 장치 ID 요청 (모터는 움직이지 않음)
  kMotion = 1,        // 운동 제어: 토크(ID 안) + 위치/속도/Kp/Kd(데이터)
  kFeedback = 2,      // 모터 -> 호스트 상태 피드백
  kEnable = 3,        // 모터 활성화
  kStop = 4,          // 모터 정지 (data[0]=1 이면 에러 해제)
  kSetZero = 6,       // 현재 위치를 기계 원점으로 저장
  kWriteParam = 18,   // 파라미터 쓰기
};

// ---- 파라미터 인덱스 -------------------------------------------------------
constexpr uint16_t kParamRunMode = 0x7005;  // 0:운동제어 1:위치 2:속도 3:전류

// ---- 값 범위 (매뉴얼 기준) -------------------------------------------------
constexpr double kPi = 3.14159265358979323846;
constexpr double kPosMin = -4.0 * kPi, kPosMax = 4.0 * kPi;  // [rad]
constexpr double kVelMin = -30.0, kVelMax = 30.0;            // [rad/s]
constexpr double kTorMin = -12.0, kTorMax = 12.0;            // [Nm]
constexpr double kKpMin = 0.0, kKpMax = 500.0;
constexpr double kKdMin = 0.0, kKdMax = 5.0;

// ---- 전송 계층과 주고받는 "CAN 프레임" 한 개 -------------------------------
//   전송 계층(시리얼 어댑터 / SocketCAN / 시뮬레이터)은 이 구조체만 알면 된다.
struct CanFrame {
  uint32_t id = 0;               // 29비트 확장 ID
  uint8_t len = 8;               // 데이터 길이 (0~8)
  std::array<uint8_t, 8> data{}; // 데이터
};

// ---- 모터가 보내주는 상태 --------------------------------------------------
struct MotorFeedback {
  uint8_t motor_id = 0;
  uint8_t fault = 0;        // 0 이 아니면 에러 (bit 의미는 매뉴얼 참고)
  uint8_t mode = 0;         // 0:리셋 1:캘리브레이션 2:동작 중(Run)
  double position = 0.0;    // [rad]  (모터 출력축 기준)
  double velocity = 0.0;    // [rad/s]
  double torque = 0.0;      // [Nm]
  double temperature = 0.0; // [°C]
};

// ---- 변환 함수 -------------------------------------------------------------
// 실수 -> 0~65535 (범위를 넘으면 잘라냄)
uint16_t floatToUint16(double x, double x_min, double x_max);
// 0~65535 -> 실수
double uint16ToFloat(uint16_t u, double x_min, double x_max);

// ---- 명령 프레임 만들기 ----------------------------------------------------
CanFrame makeEnable(uint8_t motor_id, uint8_t host_id);
CanFrame makeStop(uint8_t motor_id, uint8_t host_id, bool clear_fault);
CanFrame makeSetZero(uint8_t motor_id, uint8_t host_id);
CanFrame makeGetId(uint8_t motor_id, uint8_t host_id);
CanFrame makeWriteParamU8(uint8_t motor_id, uint8_t host_id, uint16_t index, uint8_t value);
CanFrame makeWriteParamFloat(uint8_t motor_id, uint8_t host_id, uint16_t index, float value);

// 운동 제어 명령. 모터 내부에서 계산되는 실제 출력 토크:
//   T = kp*(position - 현재위치) + kd*(velocity - 현재속도) + torque
//   kp = kd = 0  ->  순수 토크 제어
CanFrame makeMotion(uint8_t motor_id, double torque, double position = 0.0,
                    double velocity = 0.0, double kp = 0.0, double kd = 0.0);

// ---- 수신 프레임 해석 ------------------------------------------------------
// 타입 2(피드백) 프레임이면 값을 돌려주고, 아니면 std::nullopt
std::optional<MotorFeedback> parseFeedback(const CanFrame& f);

inline uint8_t commTypeOf(uint32_t id) { return (id >> 24) & 0x1F; }

}  // namespace cybergear
