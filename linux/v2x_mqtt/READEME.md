# V2X MQTT 전체 동작 흐름

이 문서는 현재 `v2x_mqtt` 코드의 전체 구조와 실행 흐름을 협업자가 빠르게 이해할 수 있도록 정리한 문서입니다.

## 1. 프로젝트 목적

이 프로젝트는 별도의 중앙 서버 없이 MQTT 브로커의 publish/subscribe 기능만으로 차량들이 서로 상태 정보를 주고받는 V2X 데모입니다.

각 차량 프로그램은 다음 역할을 합니다.

- 실행 인자로 자기 차량의 `vehicle_id`를 지정한다.
- 자기 차량 상태를 `VehicleInfo` 구조체로 만든다.
- `VehicleInfo`를 JSON 문자열로 변환한다.
- MQTT 토픽 `v2x/vehicle/<vehicle_id>/status`로 주기적으로 publish한다.
- 다른 차량의 상태 토픽 `v2x/vehicle/+/status`를 subscribe한다.
- 신호등 토픽 `v2x/trafficlight/+`를 subscribe한다.
- 수신한 JSON payload를 다시 구조체로 변환해서 내부 상태에 저장한다.

현재 코드는 실제 RTOS/CAN/UART 입력 대신 `simulate_ego()` 함수로 차량 움직임을 임시 생성합니다.

## 2. 폴더 구조

```text
v2x_mqtt/
  Makefile
  README.md
  OPERATION_FLOW.md

  common/
    types.h
    mqtt_topics.h
    vehicle_codec.h
    vehicle_codec.c

  vehicle/
    vehicle.c

  broker/
    broker_monitor.c
```

## 3. 파일별 역할

### `common/types.h`

프로젝트에서 주고받는 데이터 구조체를 정의합니다.

- `VehicleInfo`: MQTT로 송수신하는 차량 상태 정보
- `EgoVehicle`: 자기 차량의 원본 상태 정보
- `TrafficLight`: 신호등 상태 정보
- `MSB_Frame`: 향후 64비트 바이너리 프레임 변환용 union

현재 MQTT payload는 JSON입니다. `MSB_Frame`은 나중에 실제 통신 프로토콜 또는 압축된 바이너리 포맷으로 확장할 때 사용할 수 있습니다.

### `common/mqtt_topics.h`

MQTT 토픽 이름과 공통 상수를 정의합니다.

| 용도 | 토픽 | payload |
|---|---|---|
| 차량 상태 송신 | `v2x/vehicle/<vehicle_id>/status` | `VehicleInfo` JSON |
| 차량 상태 수신 | `v2x/vehicle/+/status` | `VehicleInfo` JSON |
| 신호등 상태 수신 | `v2x/trafficlight/+` | `TrafficLight` JSON |

주요 상수는 다음과 같습니다.

- `VEHICLE_ID_MIN`: 차량 ID 최소값, `1`
- `VEHICLE_ID_MAX`: 차량 ID 최대값, `254`
- `VEHICLE_ID_NONE`: 미할당 차량 ID, `0`
- `VEHICLE_PUBLISH_PERIOD_MS`: 차량 상태 publish 주기, `100ms`

### `common/vehicle_codec.h`

구조체와 JSON 간 변환 함수의 선언부입니다.

주요 함수는 다음과 같습니다.

- `vehicle_info_to_json_string()`: `VehicleInfo` -> JSON 문자열
- `vehicle_info_from_json_string()`: JSON 문자열 -> `VehicleInfo`
- `traffic_light_to_json_string()`: `TrafficLight` -> JSON 문자열
- `traffic_light_from_json_string()`: JSON 문자열 -> `TrafficLight`

### `common/vehicle_codec.c`

구조체와 JSON 간 변환 함수의 구현부입니다.

이 파일이 MQTT payload 포맷을 담당합니다. 어떤 필드명이 JSON에 들어가는지, 수신한 JSON에서 어떤 필드를 읽을지 여기서 결정됩니다.

예시 `VehicleInfo` JSON:

```json
{
  "vehicle_id": 1,
  "x": 812,
  "y": 1024,
  "speed": 40,
  "heading": 10,
  "lane": 0,
  "direction": 0,
  "cz_x": 512,
  "cz_y": 1024,
  "linked_tl": 0,
  "timestamp": 1234
}
```

예시 `TrafficLight` JSON:

```json
{
  "type_mask": 1,
  "color": 2,
  "time_left": 10
}
```

### `vehicle/vehicle.c`

차량 클라이언트의 메인 동작을 담당합니다.

현재 버전의 핵심 변경점은 MAC 주소 기반 ID 생성 로직이 제거되었다는 점입니다. 예전에는 MAC 주소를 읽고 CRC-8로 `vehicle_id`를 만들었지만, 현재는 실행 인자로 `vehicle_id`를 직접 받습니다.

주요 역할은 다음과 같습니다.

- 실행 인자에서 MQTT 브로커 주소와 포트를 읽는다.
- 실행 인자 3번째 값에서 `vehicle_id`를 읽는다.
- `vehicle_id`가 없으면 기본값 `1`을 사용한다.
- `vehicle_id`가 `VEHICLE_ID_MIN`부터 `VEHICLE_ID_MAX` 범위 안인지 검사한다.
- Mosquitto MQTT 클라이언트를 생성한다.
- MQTT client ID를 `v2x_vehicle_<vehicle_id>` 형태로 만든다.
- 자기 차량 상태 토픽에 Last Will을 설정한다.
- 브로커에 연결한다.
- 차량 상태 토픽과 신호등 토픽을 subscribe한다.
- 100ms 주기로 자기 차량 상태를 publish한다.
- MQTT 메시지를 수신하면 JSON을 파싱해서 내부 상태를 갱신한다.

### `broker/broker_monitor.c`

WSL 또는 노트북에서 실행하는 브로커 보조 프로그램입니다.

주의: 이 파일은 MQTT 브로커 자체가 아닙니다. 실제 브로커는 Mosquitto이고, `broker_monitor`는 Mosquitto에 접속하는 일반 MQTT 클라이언트입니다.

주요 역할은 다음과 같습니다.

- `v2x/vehicle/+/status`를 구독한다.
- 라즈베리파이 차량들이 publish한 `VehicleInfo` JSON을 파싱해서 로그로 출력한다.
- `v2x/trafficlight/<tl_id>`로 `TrafficLight` JSON을 주기적으로 publish한다.
- 차량 ID를 배열 인덱스로 사용해서 `O(1)`로 차량 정보를 갱신한다.
- 수신 시점마다 내부 `last_updated_ms`를 갱신한다.
- 500ms 이상 갱신되지 않은 차량을 timeout으로 제거한다.
- 빈 payload가 들어오면 Last Will 또는 retained clear 메시지로 보고 차량 목록에서 즉시 제거한다.

## 4. 실행 인자

현재 실행 형식은 다음과 같습니다.

```bash
./bin/vehicle_app [BROKER_IP] [BROKER_PORT] [VEHICLE_ID]
```

예시:

```bash
./bin/vehicle_app 127.0.0.1 1883 1
./bin/vehicle_app 127.0.0.1 1883 2
./bin/vehicle_app 127.0.0.1 1883 3
```

인자별 의미는 다음과 같습니다.

| 인자 | 의미 | 기본값 |
|---|---|---|
| `BROKER_IP` | MQTT 브로커 주소 | `127.0.0.1` |
| `BROKER_PORT` | MQTT 브로커 포트 | `1883` |
| `VEHICLE_ID` | 자기 차량 ID | `1` |

`VEHICLE_ID`는 코드상 `VEHICLE_ID_MIN`부터 `VEHICLE_ID_MAX` 범위까지 허용됩니다. 현재 상수 기준으로는 `1~254`입니다.

차량 ID는 토픽과 payload 안에 모두 들어갑니다. 브로커 보조 프로그램은 이 ID를 배열 인덱스로 사용해서 차량 정보를 관리합니다.

## 5. 전체 실행 흐름

전체 실행 순서는 다음과 같습니다.

```text
프로그램 시작
  |
  v
브로커 주소/포트 확인
  |
  v
실행 인자에서 vehicle_id 확인
  |
  v
vehicle_id 범위 검사
  |
  v
Mosquitto MQTT 클라이언트 생성
  |
  v
Last Will 설정
  |
  v
MQTT 브로커 연결
  |
  v
MQTT loop thread 시작
  |
  v
메인 루프 반복
  |
  +-- simulate_ego()로 자기 차량 상태 생성
  |
  +-- EgoVehicle -> VehicleInfo 변환
  |
  +-- VehicleInfo -> JSON 문자열 변환
  |
  +-- v2x/vehicle/<id>/status publish
  |
  +-- 다른 차량/신호등 상태를 읽어서 로그 출력
  |
  +-- 100ms 대기
```

## 6. MQTT 연결 흐름

### 6.1 연결 성공 시

`on_connect()` 콜백이 호출됩니다.

이 콜백에서는 다음 토픽을 구독합니다.

```text
v2x/vehicle/+/status
v2x/trafficlight/+
```

따라서 한 차량 프로그램은 자기 정보만 publish하지만, 모든 차량은 서로의 상태를 받을 수 있습니다.

### 6.2 차량 상태 송신

메인 루프에서 100ms마다 다음 흐름으로 publish합니다.

```text
EgoVehicle
  -> ego_to_vehicle_info()
  -> VehicleInfo
  -> vehicle_info_to_json_string()
  -> JSON payload
  -> mosquitto_publish()
  -> v2x/vehicle/<vehicle_id>/status
```

### 6.3 차량 상태 수신

다른 차량이 publish한 메시지는 `on_message()` 콜백에서 처리됩니다.

```text
MQTT message 수신
  |
  v
토픽이 v2x/vehicle/.../status 인지 확인
  |
  v
payload 복사 후 문자열 종료 문자 추가
  |
  v
vehicle_info_from_json_string()
  |
  v
자기 vehicle_id가 아니면 g_others 배열에 저장 또는 갱신
```

수신된 다른 차량 정보는 전역 배열에 저장됩니다.

- `g_others`: 다른 차량 상태 배열
- `g_others_count`: 저장된 다른 차량 수

이 배열은 MQTT 콜백 thread와 메인 thread가 같이 접근하므로 `pthread_mutex_t g_lock`으로 보호합니다.

### 6.4 신호등 상태 수신

신호등 메시지도 `on_message()` 콜백에서 처리됩니다.

```text
MQTT message 수신
  |
  v
토픽이 v2x/trafficlight/<tl_id> 인지 확인
  |
  v
tl_id 파싱
  |
  v
traffic_light_from_json_string()
  |
  v
g_traffic_lights[tl_id]에 저장
```

현재 코드에는 신호등을 publish하는 프로그램은 없습니다. 차량 클라이언트는 신호등 토픽을 구독만 해둔 상태입니다.

테스트할 때는 `mosquitto_pub`으로 직접 신호등 메시지를 넣어볼 수 있습니다.

```bash
mosquitto_pub -h 127.0.0.1 -t 'v2x/trafficlight/0' -m '{"type_mask":1,"color":2,"time_left":10}'
```

## 7. vehicle_id 지정 방식

현재 `vehicle_id`는 MAC 주소에서 자동 생성하지 않습니다. 실행할 때 직접 지정합니다.

```bash
./bin/vehicle_app 127.0.0.1 1883 1
```

여러 차량을 동시에 테스트하려면 터미널을 여러 개 열고 `VEHICLE_ID`만 다르게 실행합니다.

```bash
./bin/vehicle_app 127.0.0.1 1883 1
./bin/vehicle_app 127.0.0.1 1883 2
./bin/vehicle_app 127.0.0.1 1883 3
```

각 차량은 자기 ID가 포함된 토픽으로 publish합니다.

```text
vehicle_id=1 -> v2x/vehicle/1/status
vehicle_id=2 -> v2x/vehicle/2/status
vehicle_id=3 -> v2x/vehicle/3/status
```

## 8. Last Will 동작

차량 클라이언트는 MQTT 연결 전에 자기 상태 토픽에 Last Will을 설정합니다.

```text
topic: v2x/vehicle/<vehicle_id>/status
payload: empty
retain: true
```

차량이 비정상 종료되거나 네트워크가 끊기면 브로커가 대신 빈 payload를 publish합니다.

차량 상태 publish는 `retain=true`로 보냅니다. 따라서 브로커는 각 차량의 마지막 상태를 retained message로 갖고 있다가, 차량이 비정상 종료되면 Last Will의 빈 retained payload로 해당 retained 상태를 지웁니다.

`broker_monitor`는 빈 payload를 수신하면 해당 차량을 즉시 제거합니다. 또한 빈 payload가 오지 않는 상황에 대비해서 `last_updated_ms` 기준 500ms timeout cleanup도 수행합니다.

## 9. WSL에서 빌드와 실행

### 9.1 패키지 설치

```bash
sudo apt-get update
sudo apt-get install -y build-essential gcc make pkg-config libmosquitto-dev libcjson-dev mosquitto mosquitto-clients
```

### 9.2 빌드

```bash
make
```

성공하면 다음 실행 파일이 생성됩니다.

```text
bin/vehicle_app
bin/broker_monitor
```

### 9.3 MQTT 브로커 실행

간단한 로컬 테스트는 아래 명령으로 가능합니다.

```bash
mosquitto -v
```

외부 장비에서 WSL 브로커에 접속해야 하는 경우에는 `listener 1883 0.0.0.0` 설정이 들어간 Mosquitto 설정 파일을 사용해야 할 수 있습니다.

### 9.4 차량 클라이언트 실행

```bash
./bin/vehicle_app 127.0.0.1 1883 1
```

여러 차량을 테스트하려면 터미널을 여러 개 열고 ID만 다르게 실행합니다.

```bash
./bin/vehicle_app 127.0.0.1 1883 1
./bin/vehicle_app 127.0.0.1 1883 2
```

### 9.5 브로커 보조 프로그램 실행

WSL에서 Mosquitto 브로커를 실행한 뒤, 같은 WSL에서 아래처럼 실행합니다.

```bash
./bin/broker_monitor 127.0.0.1 1883 0
```

이 프로그램은 차량 상태를 수신해서 출력하고, 신호등 `0`번 상태를 `v2x/trafficlight/0`로 주기적으로 publish합니다.

차량 상태 관리는 다음 방식으로 수행합니다.

- `g_vehicles[vehicle_id]`에 바로 저장해서 탐색 없이 갱신한다.
- 새 payload가 들어올 때마다 `last_updated_ms`를 현재 monotonic time으로 갱신한다.
- main loop에서 50ms마다 cleanup을 돌며 500ms 이상 갱신되지 않은 차량을 제거한다.
- LWT로 빈 payload가 들어오면 timeout을 기다리지 않고 즉시 제거한다.

### 9.6 MQTT 메시지 확인

```bash
mosquitto_sub -h 127.0.0.1 -t 'v2x/#' -v
```

## 10. thread와 공유 데이터

Mosquitto는 `mosquitto_loop_start()`를 사용하므로 MQTT 수신 콜백은 별도 thread에서 실행됩니다.

현재 공유 데이터는 다음과 같습니다.

| 변수 | 역할 | 보호 방식 |
|---|---|---|
| `g_others` | 다른 차량 상태 저장 | `g_lock` |
| `g_others_count` | 다른 차량 개수 | `g_lock` |
| `g_traffic_lights` | 신호등 상태 저장 | `g_lock` |
| `g_traffic_light_valid` | 신호등 데이터 유효 여부 | `g_lock` |

메인 루프가 이 값을 읽을 때도 lock을 잡고 복사한 뒤 사용합니다.

## 11. 현재 한계와 다음 확장 포인트

현재 구현은 데모와 연동 확인에 초점이 있습니다. 다음 부분은 향후 개선 대상입니다.

- `simulate_ego()`를 실제 RTOS/CAN/UART 입력으로 교체
- `TrafficLight`를 publish하는 별도 프로그램 또는 ESP32/V2I 코드 추가
- JSON 숫자 값을 `uint8_t`, `uint16_t`로 캐스팅하기 전에 범위 검사 추가
- 필요 시 QoS 0에서 QoS 1로 변경
- 실제 통신 효율이 중요해지면 JSON 대신 64비트 `MSB_Frame` 바이너리 포맷 검토

## 12. 전체 데이터 흐름 요약

```text
[자기 차량 상태 생성]
simulate_ego()
  -> EgoVehicle
  -> ego_to_vehicle_info()
  -> VehicleInfo
  -> vehicle_info_to_json_string()
  -> JSON
  -> MQTT publish

[다른 차량 상태 수신]
MQTT subscribe
  -> JSON payload
  -> vehicle_info_from_json_string()
  -> VehicleInfo
  -> g_others 저장/갱신

[신호등 상태 수신]
MQTT subscribe
  -> JSON payload
  -> traffic_light_from_json_string()
  -> TrafficLight
  -> g_traffic_lights 저장/갱신
```
