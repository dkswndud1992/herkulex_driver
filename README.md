# HerkuleX Driver - ROS2 Package

Dongbu HerkuleX DRS-0101 / DRS-0201 스마트 서보 모터를 위한 ROS2 드라이버 패키지입니다.

> 원본 Arduino 라이브러리 (Alessandro Giacomel) 및 HerkuleX C 라이브러리 기반으로  
> Linux 시리얼 통신(termios) + ROS2 인터페이스로 재작성되었습니다.

## 기능

- **시리얼 통신**: Linux termios 기반 시리얼 포트 통신 (`/dev/ttyUSB0` 등)
- **위치 제어**: 서보를 절대 위치(0~1023) 또는 각도(-160°~+160°)로 이동
- **속도 제어**: 연속 회전 모드로 속도 제어 (-1023~1023)
- **상태 모니터링**: 주기적으로 서보 상태(위치/속도/전압/온도/토크/에러)를 토픽으로 발행
- **LED 제어**: 서보 LED 색상 변경
- **토크 제어**: 토크 ON/OFF
- **서보 관리**: ID 변경, 재부팅, 에러 클리어, 레지스트리 읽기/쓰기

## 패키지 구조

```
herkulex_driver/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/herkulex_driver/
│   ├── herkulex_serial.hpp     # 시리얼 통신 라이브러리
│   └── herkulex_node.hpp       # ROS2 노드 헤더
├── src/
│   ├── herkulex_serial.cpp     # HerkuleX 프로토콜 구현
│   └── herkulex_node.cpp       # ROS2 노드 (서비스 + 퍼블리셔)
├── msg/
│   ├── ServoStatus.msg         # 단일 서보 상태
│   └── ServoStatusArray.msg    # 다중 서보 상태
├── srv/
│   ├── SetPosition.srv         # 위치 이동 (0~1023)
│   ├── SetAngle.srv            # 각도 이동 (-160°~+160°)
│   ├── SetSpeed.srv            # 속도 설정 (연속 회전)
│   ├── SetTorque.srv           # 토크 ON/OFF
│   ├── SetLed.srv              # LED 색상 설정
│   ├── GetPosition.srv         # 현재 위치 읽기
│   ├── Reboot.srv              # 서보 재부팅
│   ├── ClearError.srv          # 에러 초기화
│   ├── SetID.srv               # 서보 ID 변경
│   └── WriteRegistry.srv       # 레지스트리 쓰기 (RAM/EEP)
├── launch/
│   └── herkulex_driver.launch.py
└── config/
    └── herkulex_params.yaml
```

## 빌드

```bash
# 워크스페이스에서 빌드
cd ~/catkin_ws  # (또는 사용 중인 ROS2 워크스페이스)
colcon build --packages-select herkulex_driver
source install/setup.bash
```

## 실행

### 기본 실행
```bash
ros2 launch herkulex_driver herkulex_driver.launch.py
```

### 파라미터 지정 실행
```bash
ros2 launch herkulex_driver herkulex_driver.launch.py \
  serial_port:=/dev/ttyUSB1 \
  baud_rate:=57600
```

### 단일 노드 실행
```bash
ros2 run herkulex_driver herkulex_node \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p servo_ids:="[0, 1, 2]" \
  -p status_rate:=10.0 \
  -p auto_initialize:=true
```

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB0` | 시리얼 포트 경로 |
| `baud_rate` | int | `115200` | 통신 속도 |
| `model` | string | `0602` | 기본 서보 모델 (예: "0602", "0201", "0101" 등) |
| `servo_ids` | int[] | `[1, 2, 3]` | 상태 모니터링할 서보 ID 목록 |
| `servo_models` | string[] | `[]` | 서보별 개별 모델 오버라이드 (예: `["1:0602", "2:0602", "3:0201"]`) |
| `status_rate` | double | `20.0` | 상태 발행 주기 (Hz, 0=비활성) |
| `auto_initialize` | bool | `true` | 시작 시 자동 초기화 여부 |
| `auto_torque_on` | bool | `true` | 서보 OFF 감지 시 자동 토크 ON 복구 (핫플러그 지원) |
| `max_sync_packet_age_sec` | double | `0.15` | 실시간 동기 토픽 최대 허용 지연(초). 초과 시 지연 패킷 폐기 |

## 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `herkulex/status` | `herkulex_driver/msg/ServoStatusArray` | 서보 상태 (위치/각도/속도/전압/온도/PWM/토크/에러, 주기적 발행) |
| `herkulex/cmd_sync_angle` | `herkulex_driver/msg/SyncAngleCmd` | **다중 서보 실시간 동시 각도 제어** (`CMD_S_JOG`, 타임스탬프 필터링) |
| `herkulex/cmd_sync_position` | `herkulex_driver/msg/SyncPositionCmd` | **다중 서보 실시간 동시 위치 제어** (`CMD_S_JOG`, 타임스탬프 필터링) |

## 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `herkulex/set_position` | `SetPosition` | 위치 이동 (0~1023) |
| `herkulex/set_angle` | `SetAngle` | 각도 이동 (-160°~+160°) |
| `herkulex/set_speed` | `SetSpeed` | 연속 회전 속도 설정 |
| `herkulex/set_torque` | `SetTorque` | 토크 ON/OFF |
| `herkulex/set_led` | `SetLed` | LED 색상 변경 |
| `herkulex/get_position` | `GetPosition` | 현재 위치 조회 |
| `herkulex/get_speed` | `GetSpeed` | 현재 속도 조회 |
| `herkulex/get_error` | `GetError` | 에러 상태 조회 |
| `herkulex/reboot` | `Reboot` | 서보 재부팅 |
| `herkulex/clear_error` | `ClearError` | 에러 초기화 |
| `herkulex/set_id` | `SetID` | 서보 ID 변경 |
| `herkulex/write_registry` | `WriteRegistry` | RAM/EEP 레지스트리 쓰기 |

## 사용 예시

### 서보 위치 이동
```bash
ros2 service call /herkulex/set_position herkulex_driver/srv/SetPosition \
  "{servo_id: 0, goal_position: 512, playtime_ms: 500, led_color: 1}"
```

### 서보 각도 이동
```bash
ros2 service call /herkulex/set_angle herkulex_driver/srv/SetAngle \
  "{servo_id: 0, goal_angle: 45.0, playtime_ms: 1000, led_color: 0}"
```

### 현재 위치 읽기
```bash
ros2 service call /herkulex/get_position herkulex_driver/srv/GetPosition \
  "{servo_id: 0}"
```

### 토크 OFF
```bash
ros2 service call /herkulex/set_torque herkulex_driver/srv/SetTorque \
  "{servo_id: 0, torque_on: false}"
```

### 상태 토픽 확인
```bash
ros2 topic echo /herkulex/status
```

## 하드웨어 연결

- **USB-to-Serial 변환기** (FTDI, CP2102 등)를 사용하여 PC와 HerkuleX 서보 연결
- HerkuleX 서보의 기본 통신 속도: **115200 bps**
- 시리얼 포트 권한 설정:
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  # 또는 영구적으로:
  sudo usermod -aG dialout $USER
  ```

## 라이선스

LGPL-2.1-or-later (원본 Arduino 라이브러리 라이선스 계승)
