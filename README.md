# 엔코더·IMU 기반 다중 모빌리티 협력 운송 로봇

Raspberry Pi 한 대가 QR을 인식하고 Wi-Fi/UDP로 두 대의 ESP32 메카넘 로봇을 제어하는 프로젝트입니다. UWB 고장 시에는 엔코더 이동거리와 로봇 2의 MPU6050 방향을 결합해 짧은 임무 동안 지도 좌표를 추정합니다.

최근 RPM 보정 결과와 다음 실험 항목은 [CALIBRATION_STATUS.md](CALIBRATION_STATUS.md)에 기록합니다.

## 현재 기능

- QR 종류에 따라 로봇 1대 또는 2대 선택 및 경로 실행
- 전진·후진·횡이동·대각선 이동·제자리 회전 수동 조종
- 로봇 1 제자리 회전 + 로봇 2 원 궤도 주행
- 브라우저 GUI에서 속도 조절 및 실시간 상태 확인
- 엔코더 PID 속도 제어와 RPM/PWM 상태 전송
- 로봇 1: HC-SR04로 전방 20 cm 장애물 정지, 후진 탈출 허용
- 로봇 2: HC-SR04로 로봇 1과의 간격 측정, 8 cm 충돌 방지
- 로봇 2: MPU6050 자세·가속도·자이로 상태 전송
- RPM·1 m 직진·공전 실험과 CSV 기록
- 1초 이상 명령이 끊기면 ESP32가 자동 정지
- UWB 없이 시뮬레이션 좌표 경로를 재현하는 QR 주행

## 파일 구성

```text
firmware/
  robot1_leader_controller/       로봇 1용 ESP32 코드
  robot2_follower_controller/     로봇 2용 ESP32 코드(초음파+MPU6050)
raspberry_pi/
  config.example.json             IP·속도·경로 설정 예시
  qr_dual_robot.py                QR 자동 주행
  qr_coordinate_navigation.py     엔코더+IMU 좌표 기반 2대 QR 주행
  odometry_navigation.py          메카넘 오도메트리·좌표 제어 핵심
  robot_control_gui.py            브라우저 수동 조종/상태 GUI
  manual_drive.py                 터미널 수동 조종
  robot_status_monitor.py         로그가 흐르지 않는 상태 모니터
  robot_experiment_gui.py         주행 보정 실험 GUI/CSV 기록
```

이전 테스트 코드, 중복 펌웨어, 측정 CSV, 보고서와 외부 UWB 예제 전체는 저장소에서 제외했습니다.

## 1. ESP32 업로드

Arduino IDE에서 보드를 `ESP32 Dev Module`, 시리얼 속도를 `115200`으로 설정합니다.

1. 각 펌웨어 폴더의 `secrets.example.h`를 복사해 `secrets.h`로 이름을 바꿉니다.
2. `secrets.h`에 사용할 핫스팟 이름과 비밀번호를 입력합니다.
3. 로봇별 파일을 혼동하지 말고 업로드합니다.
   - 로봇 1: `firmware/robot1_leader_controller/robot1_leader_controller.ino`
   - 로봇 2: `firmware/robot2_follower_controller/robot2_follower_controller.ino`
4. 업로드 후 시리얼 모니터에서 각 ESP32의 IP를 확인합니다.

`secrets.h`는 Git에 올라가지 않습니다. 두 로봇의 모터 방향 보정값이 서로 다르므로 `ROBOT_ID`만 바꿔 같은 펌웨어를 사용하면 안 됩니다.

### 주요 핀 배치

| 장치 | 핀 |
|---|---|
| FL RPWM / LPWM | GPIO 25 / 26 |
| FR RPWM / LPWM | GPIO 27 / 14 |
| RL RPWM / LPWM | GPIO 32 / 33 |
| RR RPWM / LPWM | GPIO 18 / 19 |
| FL 엔코더 A / B | GPIO 13 / 23 |
| FR 엔코더 A / B | GPIO 16 / 17 |
| RL 엔코더 A / B | GPIO 21 / 22 |
| RR 엔코더 A / B | GPIO 34 / 35 |
| HC-SR04 TRIG / ECHO | GPIO 2 / 12 |
| MPU6050 SDA / SCL(로봇 2) | GPIO 4 / 5 |

HC-SR04 ECHO는 5 V이므로 ESP32에 직접 연결하지 말고 `1 kΩ / 2 kΩ` 분압 회로를 사용해야 합니다. GPIO 34/35에는 내부 풀업이 없으므로 엔코더에 외부 풀업이 필요합니다. ESP32, 모터 드라이버, 센서 전원의 GND는 공통으로 연결합니다.

## 2. Raspberry Pi 설치

```bash
sudo apt update
sudo apt install -y git python3-opencv
git clone https://github.com/minkyu6877/dltla.git
cd dltla/raspberry_pi
cp config.example.json config.json
nano config.json
```

`config.json`의 `robot_ips`를 시리얼 모니터에서 확인한 로봇 1, 로봇 2 IP 순서로 입력합니다.

```json
"robot_ips": [
  "로봇 1 IP",
  "로봇 2 IP"
]
```

휴대전화 핫스팟은 재연결할 때 IP가 바뀔 수 있으므로 ESP32를 다시 켠 뒤 시리얼 모니터에서 확인하는 것이 안전합니다.

## 3. 실행

한 번에 아래 프로그램 중 하나만 실행합니다. 모두 UDP 상태 포트 `4212`를 사용합니다.

```bash
cd ~/dltla/raspberry_pi

# 브라우저 GUI: 노트북에서 http://<라즈베리파이-IP>:8080 접속
python3 robot_control_gui.py

# QR 자동 주행(SSH 환경에서는 --headless 권장)
python3 qr_dual_robot.py --headless

# UWB 없이 시뮬레이션 경로를 그대로 실행
python3 qr_coordinate_navigation.py --headless

# 터미널 수동 조종
python3 manual_drive.py

# 고정 화면 상태 모니터
python3 robot_status_monitor.py

# 실험 GUI: http://<라즈베리파이-IP>:8081
python3 robot_experiment_gui.py
```

종료는 `Ctrl+C`입니다. 프로그램 종료 시 두 로봇에 반복해서 `STOP`을 전송합니다.

## 4. QR 데이터와 경로

현재 QR 형식은 다음과 같습니다.

```json
{"destination":"B","cargo_type":"LONG_BOX","weight_kg":4.5}
```

`cargo_robot_counts`에서 화물별 로봇 수를 정하고, `routes`에서 목적지 A~D의 이동 순서를 설정합니다. QR에는 모터 명령을 넣지 않으며 Raspberry Pi가 경로를 선택합니다.

- `vx`: 전진 `+`, 후진 `-`
- `vy`: 좌측 횡이동 `+`, 우측 횡이동 `-`
- `w`: 반시계 회전 `+`, 시계 회전 `-`
- `duration_sec`: 해당 명령을 유지할 시간

## 5. UWB 없는 좌표 주행

`qr_coordinate_navigation.py`는 QR의 `dest_x=3.55`, `dest_y=2.55`를 확인한 뒤 두 로봇으로 시뮬레이션의 코너 경로를 실행합니다. 이 모드에서는 QR을 비추기 전에 로봇을 반드시 아래 바닥 표시에 놓아야 합니다.

- 지도 원점: 4×3 m 작업장의 왼쪽 아래
- 로봇 1 중심: `(0.99, 0.75) m`
- 로봇 2 중심: `(0.46, 0.75) m`
- 두 로봇의 정면: 오른쪽 `+X`, `0°`
- 로봇 몸체 사이 간격: `0.30 m`

실행 전 설정만 검사할 수 있습니다.

```bash
python3 qr_coordinate_navigation.py --config config.json --check-config
```

이 방식의 좌표는 절대좌표가 아니라 엔코더 적분값입니다. 바퀴 미끄러짐과 MPU6050 yaw drift가 누적되므로 매 임무 시작 때 위 위치로 다시 맞춰야 하며, 장시간 반복 운행에는 AprilTag·카메라·LiDAR 같은 외부 절대 위치 보정 수단을 추가해야 합니다.

## 안전 확인

처음에는 바퀴를 바닥에서 띄우고 전진·후진·정지와 네 바퀴 RPM 부호를 확인하세요. 모터 전원은 ESP32에서 공급하지 마세요. 시험 중에는 즉시 전원을 차단할 사람이 로봇 옆에 있어야 합니다.

로봇 2는 저속 시험에서 FL/RR 바퀴가 상대적으로 느린 경향이 확인되어 현재 펌웨어에 보수적인 정마찰 피드포워드(`FL=0.0045`, `RR=0.0055`)가 적용되어 있습니다. 방향 반전값은 바꾸지 말고 펌웨어를 다시 올린 뒤 `robot_experiment_gui.py`로 RPM 시험을 반복해 보정 효과를 확인해야 합니다.
