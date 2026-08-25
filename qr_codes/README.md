# 배송 QR 카드

이 QR들은 `cargo_fleet_manager/qr_reader.py`가 읽는 배송 명령이다. QR을 찍으면 화물 종류에 따라 1대 또는 2대가 배정되고, `대기소 → 적재소 → 목적지 → 대기소` 순서로 이동한다.

모든 QR의 목적지는 공통 목적지 `(3.20, 2.40) m`다. 화물 조건만 로봇 배정에 사용한다.

| 파일 | 화물 | 배정 로봇 | 목적지 좌표 |
| --- | --- | --- | --- |
| `01_SMALL_DELIVERY.png` | 소형 상자 2 kg | 1대 | (3.20, 2.40) m |
| `02_LONG_DELIVERY.png` | 장형 상자 8 kg | 2대 | (3.20, 2.40) m |

QR 내부 데이터 형식은 다음과 같다. `dest_x`, `dest_y`를 바꾸면 새 목적지를 만들 수 있다.

```json
{"command":"DELIVERY","cargo_type":"LONG_BOX","weight_kg":8.0,"dest_x":3.20,"dest_y":2.40}
```

새 카드는 `python3 tools/generate_delivery_qrs.py`로 다시 생성한다.
