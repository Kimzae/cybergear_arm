# cybergear_arm

Xiaomi CyberGear만으로 만드는 **다축 로봇팔 제어 패키지**입니다. (ROS2 Jazzy, C++)

- 여러 모터를 한 주기 안에 동시에 제어 (2축 / 4축 설정 제공, 6축까지 확장 가능)
- **DH + 링크 부품 목록(yaml)** 으로 만드는 N축 동역학 모델 `M(q)q̈ + C(q,q̇)q̇ + G(q) + F(q̇) = τ`
- 슬라이딩 모드 제어(SMC) — 선형 피드백을 **모터 내부 PD로 옮겨 채터링 제거** (`smc.inner_loop: motor`)
- 하드웨어 없이 돌려볼 수 있는 **시뮬레이터 모드**
- 기구학 보정 실험용 로그 토픽 (`/arm/debug`)

> 구조와 원리 설명은 [ARCHITECTURE.md](ARCHITECTURE.md)에 있습니다.
> ⚠️ **초안(draft)** 입니다. 시뮬레이터와 오프라인 테스트로만 검증했고, 실제 모터에서는 아직 검증하지 않았습니다.

---

## 1. 로봇 구성 (2축 기준)

| 관절 | 모터 ID | 회전축 | 원점 (q = 0) |
|---|---|---|---|
| joint1 | 1 | 전역 z축 | 전역 좌표계와 일치 |
| joint2 | 2 | 모터 1 위에 수평으로 장착 | 링크 2(x₂)가 전역 z축(위쪽)을 향함 |

DH 파라미터:

| i | θ | d | a | α |
|---|---|---|---|---|
| 1 | q₁ | d₁ | 0 | 90° |
| 2 | **q₂ + 90°** | d₂ | 0 | 0 |

> θ₂에 +90°가 붙는 이유: θ₂ = q₂로 두면 q₂ = 0일 때 x₂가 전역 **x**축을 향합니다.
> "x₂ = 전역 z" 조건을 맞추려면 90° 오프셋이 필요합니다. 그래서 **q₂는 수직(위)에서 잰 각도**입니다.

---

## 2. 설치

```bash
sudo apt install -y libeigen3-dev libyaml-cpp-dev
cd ~/ros2_ws/src
git clone https://github.com/Kimzae/cybergear      # 이 패키지가 들어 있는 저장소
cd ~/ros2_ws
colcon build --packages-select cybergear_arm
source install/setup.bash
```

> `cybergear_arm` 폴더가 `~/ros2_ws/src` 아래 어딘가에 있으면 colcon이 알아서 찾습니다.

---

## 3. 단계별 실행

### 단계 1. 오프라인 테스트 (하드웨어 없이)

```bash
CFG=$(ros2 pkg prefix cybergear_arm)/share/cybergear_arm/config
ros2 run cybergear_arm protocol_selftest            # 통신 프레임이 Python 버전과 같은지
ros2 run cybergear_arm model_test $CFG              # 동역학 모델 검증 (2축 해석해 비교, 4축 성질)
ros2 run cybergear_arm smc_sim_test                 # 지연 없는 이상 조건에서 SMC 강인성
ros2 run cybergear_arm chattering_sim_test $CFG     # 채터링 재현 + 해결 비교 (4 ms 지연)
```

`chattering_sim_test` 결과 (개발 환경, 500 Hz, 피드백 지연 4 ms, 실제 질량 +30%):

| 경우 | 채터링 [Nm] | 최대 오차 [°] |
|---|---|---|
| A) HOST, 이전 이득 | 2.17 | 21.3 |
| B) HOST, φ↑ Kd↓ | 0.11 | 1.2 |
| **C) MOTOR, 이전 이득** | **0.0013** | **0.05** |
| D) MOTOR + 조건부 적분 | 0.0013 | 0.26 |

> 시뮬레이터의 모터 내부 PD는 이상적(노이즈·내부 지연 없음)이라 C는 낙관적인 값입니다. 경향을 보는 용도로 쓰세요.

### 단계 2. 시뮬레이터 모드 (ROS2 전체 흐름 확인)

```bash
# 터미널 1
ros2 launch cybergear_arm arm.launch.py dof:=2 transport:=sim
# 터미널 2
ros2 run cybergear_arm sine_reference.py --ros-args -p amplitude:="[0.3, 0.3]" -p freq:=0.3
# (4축이면 amplitude 를 4개로: "[0.2, 0.2, 0.2, 0.2]")
# 터미널 3
ros2 topic echo /arm/joint_states
```

### 단계 3. 실제 모터

체크리스트:
- [ ] 로봇팔을 **손으로 받칠 수 있는 자세**에서 시작
- [ ] `config/arm_2dof.yaml`의 `torque_limits`를 작게 (예: 1.0)
- [ ] `config/model_2dof.yaml`의 치수·질량을 실측값으로 수정
- [ ] `directions` 확인: 모터 +방향이 DH 축 방향과 같은가? (아래 5절 참고)

```bash
ros2 launch cybergear_arm arm.launch.py dof:=2        # 시작하면 "현재 자세"를 유지
ros2 run cybergear_arm sine_reference.py --ros-args -p amplitude:="[0.1, 0.1]"
```

> ⚠️ 노드를 종료하면 약 0.5초 동안 댐핑을 건 뒤 모터를 풀어 줍니다. **팔이 중력 때문에 내려오니** 받쳐 주세요.

## 3.5 4축으로 확장하기

1. **모터 ID 설정:** 새 모터 2개를 ID 3, 4로 설정하고 `cybergear_scan.py`로 확인합니다. 같은 버스에 같은 ID가 두 개 있으면 안 됩니다.
2. **대역폭 확인:** CH340 어댑터 1개 + 모터 4개는 **500 Hz까지** 가능합니다. 이때 CAN 버스 부하는 약 56%입니다. 1 kHz가 필요하면 어댑터를 2개로 나누세요 (`buses`, `bus_indices`).
3. **모델 작성:** `config/model_4dof.yaml`은 "평면 팔꿈치 + 손목" **예시**입니다. 실제 관절 3, 4의 DH와 부품 위치로 바꾸세요.
   - 관절 3이 생기면 보통 a₂ = 링크2 길이로 둡니다 (frame 2를 팔꿈치로 옮김).
4. **검증 → 시뮬레이터 → 실제** 순서로 진행합니다.
   ```bash
   ros2 run cybergear_arm model_test $CFG                    # 모델 파일 문법·성질 확인
   ros2 launch cybergear_arm arm.launch.py dof:=4 transport:=sim
   ros2 launch cybergear_arm arm.launch.py dof:=4            # 실제 (받칠 준비!)
   ```
5. **한 관절씩 켜기 권장:** 처음에는 새 관절의 `torque_limits`를 0.5 정도로 작게 두고, `directions`부터 확인하세요.

---

## 4. 토픽

| 토픽 | 타입 | 방향 | 내용 |
|---|---|---|---|
| `/arm/reference` | `trajectory_msgs/JointTrajectoryPoint` | 입력 | `positions`, `velocities`, `accelerations` (관절 좌표, rad) |
| `/arm/joint_states` | `sensor_msgs/JointState` | 출력 | position, velocity, effort(모터가 보고한 토크) |
| `/arm/debug` | `std_msgs/Float64MultiArray` | 출력 | `[t, q(n), q̇(n), q_d(n), e(n), s(n), τ(n), 주기ms, 계산ms]` |

- 여러분의 역기구학 코드는 `/arm/reference`로 관절 목표를 보내면 됩니다.
- 기구학 보정 실험 데이터 기록:
  ```bash
  ros2 bag record /arm/debug /arm/joint_states
  ```

---

## 5. 주요 파라미터 (`config/arm_Ndof.yaml`)

| 그룹 | 파라미터 | 설명 |
|---|---|---|
| 통신 | `transport` | `serial`(CH340) / `socketcan` / `sim` |
| | `buses` | 시리얼 포트 목록 또는 CAN 인터페이스 목록 |
| 관절 | `motor_ids`, `bus_indices` | 관절 ↔ (버스, 모터 ID) 연결 |
| | `directions`, `offsets` | `q_joint = direction × q_motor + offset` |
| | `torque_limits` | 관절별 최대 토크 [Nm] |
| 루프 | `rate_hz` | 제어 주기 (CH340 1개 + 모터 2개 → 500 Hz 권장) |
| 안전 | `joint_min/max`, `max_velocity`, `feedback_timeout` | 넘으면 **SAFE 모드**(댐핑만 유지, 재시작 필요) |
| SMC | `smc.inner_loop` | `motor`(권장) / `host`. [ARCHITECTURE.md](ARCHITECTURE.md) 4절 |
| | `smc.lambda`, `smc.k`, `smc.phi`, `smc.kd` | 모터 내부 PD: `kd_int = kd + k/phi`, `kp_int = lambda·kd_int` |
| | `smc.ki`, `smc.i_max` | 조건부 적분 (기본 0. 정지 자세 정상상태 오차가 보일 때만) |
| | `smc.use_model` | `false`면 모델 없이 동작 (모델 효과 비교 실험용) |
| 모델 | `config/model_Ndof.yaml` | DH + 링크 부품(모터 원통, 프로파일, 점질량). **"측정 필요" 값은 실측** |

**`directions` 확인 방법:** 노드를 켜지 않은 상태에서 `cybergear_test`(Python 패키지)로 +0.2 Nm를 줍니다.
- joint1: 위에서 봤을 때 **반시계**로 돌면 `+1`
- joint2: q₁ = 0 자세에서 팔이 전역 **−x 방향**으로 기울면 `+1`
  (DH상 z₁ = 전역 −y이므로, q₂ > 0이면 x₂가 z → −x 쪽으로 돕니다.)
- 반대로 돌면 `-1`로 설정하세요.

---

## 6. 실시간 권한 (선택, 1 kHz 도전 시 권장)

```bash
echo "$USER - rtprio 98" | sudo tee /etc/security/limits.d/99-realtime.conf
echo "$USER - memlock unlimited" | sudo tee -a /etc/security/limits.d/99-realtime.conf
# 로그아웃 후 다시 로그인
```

설정하지 않으면 노드가 경고를 출력하고 일반 우선순위로 동작합니다. 5초마다 로그에 **최대 주기 / 최대 계산 시간**이 출력되니, 목표 주기와 비교해 보세요.

---

## 7. 파일 구조

```
cybergear_arm/
├── CMakeLists.txt, package.xml
├── config/
│   ├── arm_2dof.yaml, arm_4dof.yaml      제어기 설정
│   └── model_2dof.yaml, model_4dof.yaml  동역학 모델 (DH + 링크 부품)
├── launch/arm.launch.py              실행 파일 (dof:=2|4)
├── include/cybergear_arm/
│   ├── cybergear_protocol.hpp        프레임 인코딩/디코딩
│   ├── can_transport.hpp             전송 계층 인터페이스
│   ├── sim_transport.hpp             시뮬레이터 전송 계층
│   ├── cybergear_bus.hpp             다축 동시 제어 (관절 ↔ 모터)
│   ├── arm_dynamics.hpp              동역학 인터페이스 + 2축 해석해 (검증용)
│   ├── dh_dynamics.hpp               N축 범용 동역학 (DH + yaml)
│   └── smc_controller.hpp            슬라이딩 모드 제어기
├── src/                              위 헤더의 구현 + arm_controller_node.cpp
│   ├── serial_at_transport.cpp       CH340 'AT' 시리얼
│   └── socketcan_transport.cpp       SocketCAN
├── scripts/sine_reference.py         테스트용 기준 궤적 발행기
└── test/                             하드웨어 없는 테스트
```
