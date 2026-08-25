# 4앵커 UWB 위치 확인

캘리브레이션을 마친 A1~A4와 로봇에 장착된 UWB 태그로 4 m x 3 m 공간의 위치를 확인합니다.

현재 반영된 실측값:

- 앵커와 태그 안테나 높이: `0.100 m`
- 로봇 중심에서 태그까지: 앞쪽 `0.110 m`, 왼쪽 `0.025 m`
- 앵커 좌표: A1 `(0,0)`, A2 `(4,0)`, A3 `(4,3)`, A4 `(0,3)`

## 1. 앵커 배치

```text
       A4 (0,3) -------- A3 (4,3)
          |                 |
          |                 |
       A1 (0,0) -------- A2 (4,0)  -> +x
```

좌표는 기판 모서리가 아니라 안테나 중심 기준입니다. 실제 변 길이가 정확히 4.000 m, 3.000 m가 아니면 `tag_position_sender.ino`의 `ANCHOR_X_M`, `ANCHOR_Y_M`을 실측값으로 수정해야 합니다.

## 2. 태그 위치 송신 코드 업로드

Arduino IDE에서 다음 파일을 엽니다.

```text
uwb_localization/tag_position_sender/tag_position_sender.ino
```

같은 폴더의 `secrets.example.h`를 `secrets.h`로 복사하고 핫스팟과 Raspberry Pi IP를 입력합니다. 이 작업 PC에는 현재 `Ganglaxy`와 Raspberry Pi `10.44.212.72` 설정의 `secrets.h`가 준비되어 있습니다.

보드는 `ESP32 Dev Module`을 선택하고 로봇의 UWB 태그에 업로드합니다. 이 코드는 캘리브레이션용 `reference_tag.ino`를 대체합니다. 앵커 펌웨어는 바꾸지 않습니다.

## 3. Raspberry Pi 모니터 실행

저장소 최신 파일을 Raspberry Pi에 복사한 뒤 실행합니다.

```bash
cd ~/dltla/raspberry_pi
python3 uwb_position_monitor.py
```

화면이 계속 흐르지 않고 태그 좌표, 로봇 중심 좌표, A1~A4 거리, 위치 RMSE와 수신 속도가 고정 화면에 표시됩니다. CSV도 저장하려면 다음처럼 실행합니다.

```bash
python3 uwb_position_monitor.py --csv uwb_position_log.csv
```

## 4. 첫 위치 검증

로봇 앞을 경기장 `+x` 방향으로 맞추고 정지시킵니다. 처음에는 기본 수동 heading `0도`를 사용합니다.

태그 안테나 중심을 다음 지점에 놓고 각 지점에서 20초 이상 확인합니다.

- `(0.5, 0.5)`
- `(3.5, 0.5)`
- `(2.0, 1.5)`
- `(0.5, 2.5)`
- `(3.5, 2.5)`

목표는 `quality=OK`, RMSE 0.20 m 이하, 정지 상태 좌표 흔들림 0.10 m 이하입니다. 중심 좌표는 다음 오프셋 변환을 사용합니다.

```text
center_x = tag_x - 0.110*cos(yaw) + 0.025*sin(yaw)
center_y = tag_y - 0.110*sin(yaw) - 0.025*cos(yaw)
```

태그의 `(x,y)`는 로봇이 회전해도 유효하지만, 로봇 중심 보정은 실시간 yaw가 필요합니다. 현재 로봇 2만 MPU6050 yaw를 상태 패킷에 포함하므로 태그가 어느 로봇에 달렸는지 확인한 뒤 다음 단계에서 결합합니다.
