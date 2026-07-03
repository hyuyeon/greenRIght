# Raspberry Pi + WSL Broker 현재 코드 테스트

이 문서는 예전에 확인한 단순 `CAN_Packet` 바이너리 예제가 아니라, 현재 이 폴더의 `vehicle_app` 코드가 Raspberry Pi와 WSL Mosquitto 브로커 환경에서 동작하는지 확인하는 절차입니다.

## 1. 예전 테스트 코드와 현재 코드의 차이

예전 테스트 코드는 다음 방식이었습니다.

- payload: `struct CAN_Packet` 바이너리
- 송신 토픽: `v2x/laptop/data`
- 수신 토픽: `v2x/raspberry/data`
- 브로커 IP: 코드에 직접 고정

현재 `vehicle_app` 코드는 다음 방식입니다.

- payload: `VehicleInfo` JSON 문자열
- 차량 송신 토픽: `v2x/vehicle/<vehicle_id>/status`
- 차량 수신 토픽: `v2x/vehicle/+/status`
- 신호등 수신 토픽: `v2x/trafficlight/+`
- 브로커 IP: 실행 인자로 전달
- 차량 ID: 실행 인자로 전달

따라서 지금 코드는 예전처럼 바이너리 struct를 보내지 않습니다. WSL에서 확인할 때도 JSON payload가 보여야 정상입니다.

## 2. 목표 구성

```text
Raspberry Pi
  - vehicle_app 실행
  - v2x/vehicle/1/status 로 VehicleInfo JSON publish
  - v2x/vehicle/+/status subscribe
  - v2x/trafficlight/+ subscribe

WSL
  - Mosquitto broker 실행
  - mosquitto_sub 로 v2x/# 확인
  - broker_monitor 로 차량 상태 수신 및 신호등 정보 송신
  - 필요 시 mosquitto_pub 로 신호등 JSON publish
  - 선택적으로 vehicle_app 2번 차량 실행
```

## 3. WSL에서 브로커 실행

WSL에서 Mosquitto가 설치되어 있어야 합니다.

```bash
sudo apt-get update
sudo apt-get install -y mosquitto mosquitto-clients
```

Raspberry Pi가 접속할 수 있도록 `0.0.0.0:1883`으로 브로커를 엽니다.

```bash
mkdir -p /tmp/mosq
cat > /tmp/mosq/mosquitto.conf << 'EOF'
listener 1883 0.0.0.0
allow_anonymous true
persistence false
EOF

mosquitto -c /tmp/mosq/mosquitto.conf -v
```

다른 WSL 터미널에서 전체 V2X 토픽을 확인합니다.

```bash
mosquitto_sub -h 127.0.0.1 -t 'v2x/#' -v
```

## 4. WSL에서 브로커 보조 프로그램 실행

Mosquitto는 실제 MQTT 브로커이고, 이번에 추가한 `broker_monitor`는 브로커에 붙는 보조 프로그램입니다.

`broker_monitor`가 하는 일은 다음과 같습니다.

- `v2x/vehicle/+/status`를 구독해서 라즈베리파이 차량 정보를 출력한다.
- `v2x/trafficlight/<id>`로 신호등 JSON을 주기적으로 publish한다.
- 차량 ID를 배열 인덱스로 사용해서 `O(1)`로 갱신한다.
- payload를 받을 때마다 내부 `last_updated_ms`를 갱신한다.
- 500ms 이상 갱신되지 않은 차량은 cleanup 루틴으로 제거한다.
- 차량 Last Will로 빈 payload가 들어오면 해당 차량을 제거된 것으로 처리한다.

WSL에서 빌드합니다.

```bash
make
```

실행합니다.

```bash
./bin/broker_monitor 127.0.0.1 1883 0
```

인자 의미:

| 인자 | 의미 |
|---|---|
| `127.0.0.1` | Mosquitto broker 주소 |
| `1883` | Mosquitto broker 포트 |
| `0` | publish할 신호등 ID |

정상 실행되면 아래 역할을 계속 수행합니다.

```text
RX: v2x/vehicle/+/status 수신
TX: v2x/trafficlight/0 송신
```

## 5. Raspberry Pi에 코드 복사

Windows PowerShell에서 복사합니다.

```powershell
scp -r "C:\Users\한국전파진흥협회\Downloads\v2x_mqtt3\v2x_mqtt" pi@<RPI_IP>:~/v2x_mqtt
```

예시:

```powershell
scp -r "C:\Users\한국전파진흥협회\Downloads\v2x_mqtt3\v2x_mqtt" pi@192.168.0.50:~/v2x_mqtt
```

Raspberry Pi에서 폴더로 이동합니다.

```bash
cd ~/v2x_mqtt
```

## 6. Raspberry Pi에서 빌드

Raspberry Pi에 필요한 패키지를 설치합니다.

```bash
sudo apt-get update
sudo apt-get install -y build-essential gcc make pkg-config libmosquitto-dev libcjson-dev mosquitto-clients
```

빌드합니다.

```bash
make clean
make
```

성공하면 실행 파일이 생성됩니다.

```text
bin/vehicle_app
```

## 7. 먼저 MQTT 연결만 확인

Raspberry Pi에서 WSL 브로커로 테스트 메시지를 보냅니다.

```bash
mosquitto_pub -h <BROKER_IP> -p 1883 -t 'v2x/test' -m 'hello from raspberry pi'
```

WSL의 `mosquitto_sub -t 'v2x/#' -v` 화면에 아래처럼 보이면 브로커 연결은 정상입니다.

```text
v2x/test hello from raspberry pi
```

`<BROKER_IP>`는 예전에 성공했던 브로커 IP를 그대로 쓰면 됩니다.

## 8. Raspberry Pi에서 현재 vehicle_app 실행

Raspberry Pi에서 현재 코드의 차량 앱을 실행합니다.

```bash
./bin/vehicle_app <BROKER_IP> 1883 1
```

예시:

```bash
./bin/vehicle_app 172.20.10.4 1883 1
```

WSL의 subscribe 화면에 아래와 비슷한 JSON이 계속 보이면 성공입니다.

```text
v2x/vehicle/1/status {"vehicle_id":1,"x":812,"y":1024,"speed":40,"heading":360,"lane":0,"direction":0,"cz_x":512,"cz_y":1024,"linked_tl":0,"timestamp":1234}
```

이 단계가 성공하면 Raspberry Pi -> WSL broker 방향은 현재 코드로 동작하는 것입니다.

`broker_monitor`를 실행 중이면 WSL 쪽에는 아래처럼 차량 정보가 출력됩니다.

```text
[RX] vehicle_id=1 x=812 y=1024 speed=40 heading=360 lane=0 dir=0 ts=1234
```

## 9. WSL에서 Raspberry Pi가 받는지 확인

현재 `vehicle_app`은 `v2x/trafficlight/+`를 구독하고 있습니다. 그래서 WSL에서 신호등 JSON을 publish하면 Raspberry Pi 앱이 받을 수 있습니다.

`broker_monitor`를 실행 중이면 이 신호등 메시지는 자동으로 주기 발행됩니다.

수동 테스트도 가능합니다.

WSL에서 실행:

```bash
mosquitto_pub -h 127.0.0.1 -t 'v2x/trafficlight/0' -m '{"type_mask":1,"color":2,"time_left":10}'
```

Raspberry Pi의 `vehicle_app` 로그에 아래처럼 신호등 정보가 붙어서 나오면 WSL -> Raspberry Pi 방향도 정상입니다.

```text
[TX] id=1 x=... y=... speed=40 heading=... | 주변 차량 0대 | 연동 신호등[0] color=2 time_left=10
```

## 10. 차량끼리 서로 받는지 확인

현재 코드는 같은 `vehicle_app`을 여러 개 실행해서 차량 여러 대처럼 테스트할 수 있습니다.

방법 A: Raspberry Pi 한 대와 WSL 한 대를 차량처럼 실행

WSL에서도 빌드 가능한 환경이면 프로젝트 폴더에서 실행합니다.

```bash
make
./bin/vehicle_app 127.0.0.1 1883 2
```

Raspberry Pi:

```bash
./bin/vehicle_app <BROKER_IP> 1883 1
```

정상이라면 Raspberry Pi 로그의 `주변 차량` 수가 `1대`로 증가합니다. WSL에서 실행한 2번 차량도 1번 차량을 수신합니다.

방법 B: WSL에서 직접 차량 JSON publish

WSL에서 수동으로 2번 차량 메시지를 보냅니다.

```bash
mosquitto_pub -h 127.0.0.1 -t 'v2x/vehicle/2/status' -m '{"vehicle_id":2,"x":100,"y":200,"speed":50,"heading":90,"lane":0,"direction":0,"cz_x":512,"cz_y":1024,"linked_tl":0,"timestamp":1}'
```

Raspberry Pi 로그에서 `주변 차량 1대`가 보이면 차량 상태 수신 로직도 정상입니다.

## 11. 성공 기준

아래 3개가 되면 현재 코드 기준 테스트는 성공입니다.

1. Raspberry Pi의 `vehicle_app` 실행 후 WSL `mosquitto_sub`에서 `v2x/vehicle/1/status` JSON이 보인다.
2. WSL에서 `v2x/trafficlight/0` JSON을 publish하면 Raspberry Pi 로그에 신호등 정보가 표시된다.
3. WSL 또는 다른 Raspberry Pi에서 `vehicle_id`가 다른 차량 메시지를 보내면 `주변 차량` 수가 증가한다.
4. WSL에서 `broker_monitor`를 실행하면 차량 `RX` 로그와 신호등 `TX` 로그가 함께 보인다.

## 12. 자주 헷갈리는 부분

- 예전 테스트처럼 `struct CAN_Packet` 크기만큼 바이너리가 오지 않습니다. 현재 코드는 JSON 문자열이 payload입니다.
- 현재 앱은 자기 `vehicle_id`와 같은 차량 메시지는 무시합니다.
- 차량 ID를 다르게 줘야 서로 다른 차량으로 인식합니다.
- 실행 인자 3번째 값은 MAC 주소가 아니라 숫자 차량 ID입니다.
- 현재 허용 ID 범위는 `1~254`입니다.
- `broker_monitor`는 MQTT 브로커 자체가 아닙니다. 실제 브로커는 Mosquitto이고, `broker_monitor`는 그 브로커에 접속하는 모니터/신호등 송신 프로그램입니다.
