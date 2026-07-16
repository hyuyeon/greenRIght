# GreenRight 빌드 및 실행 매뉴얼

## 1. 문서 목적

이 문서는 GreenRight 교차로 충돌 보조 시스템을 구성하는 다음 모듈의 빌드와 실행 절차를 설명한다.

- `firmware/`: STM32F429ZI 센서·자차 상태 수집 펌웨어
- `RTOS/`: STM32F429ZI FreeRTOS 기반 충돌 판단·LCD·부저 노드
- `linux/v2x_mqtt/`: Raspberry Pi/Debian Linux 기반 CAN↔MQTT·HD Map 처리 프로그램
- `subcar/`: 보조 차량 MQTT 발행 Python 프로그램
- `ai/`: 현재 저장소에는 빌드 대상 소스가 없음

기준 보드는 `NUCLEO-F429ZI`, CAN bit rate는 `500 kbit/s`, Linux CAN 인터페이스 이름은 `can0`이다.

> 중요: 현재 `RTOS/`는 완전한 STM32CubeIDE 프로젝트지만, `firmware/`에는 `.project`, `.cproject`, `.ioc`, HAL Driver, CMSIS, startup, linker script가 없다. 따라서 `firmware/`는 이 저장소만으로 단독 빌드할 수 없고 원본 STM32CubeIDE 프로젝트 또는 동일 설정으로 생성한 기반 프로젝트가 필요하다.

## 2. 전체 구성과 권장 실행 순서

```text
Firmware STM32 ── CAN 0x200 ──┬── Linux V2X/MQTT
                              └── RTOS STM32

Linux V2X/MQTT ─ CAN 0x321 ─────> RTOS STM32

Linux V2X/MQTT ←── MQTT Broker ──→ Subcar / 신호등 발행기
```

권장 순서는 다음과 같다.

1. CAN 배선과 종단 저항을 확인한다.
2. MQTT Broker를 실행한다.
3. Linux에서 `can0`를 500 kbit/s로 활성화한다.
4. RTOS를 빌드하고 STM32에 플래시한다.
5. Firmware를 원본 CubeIDE 프로젝트에서 빌드하고 STM32에 플래시한다.
6. Linux `v2x_mqtt_rpi`를 실행한다.
7. 필요하면 Subcar 또는 가짜 차량 도구를 실행한다.
8. UART, `candump`, MQTT 구독 로그로 전체 통신을 검증한다.

## 3. 공통 하드웨어 준비

### 3.1 CAN 배선

Firmware와 RTOS 코드 모두 STM32 CAN1을 다음 핀으로 사용한다.

| 신호 | STM32 핀 |
|---|---|
| CAN1 RX | PD0 |
| CAN1 TX | PD1 |

STM32 GPIO를 CANH/CANL에 직접 연결하면 안 된다. 각 STM32와 Linux 장치에 CAN transceiver 또는 USB/SPI CAN 어댑터가 필요하다.

```text
STM32 PD1(TX) ──> CAN Transceiver TXD
STM32 PD0(RX) <── CAN Transceiver RXD
                    │
                    ├── CANH
                    └── CANL
```

확인 사항:

- 모든 노드의 CANH끼리 연결
- 모든 노드의 CANL끼리 연결
- 모든 노드의 GND 공통 연결
- 버스 양 끝에 각각 120Ω 종단 저항
- 전원을 끈 상태에서 CANH-CANL 저항이 약 60Ω인지 확인
- 모든 노드의 bit rate를 500 kbit/s로 통일
- 사용하는 transceiver의 I/O 전압이 STM32 및 Raspberry Pi와 호환되는지 확인

### 3.2 UART 로그

Firmware와 RTOS의 디버그 UART는 USART3, `115200 8N1`이다.

| 신호 | STM32 핀 |
|---|---|
| USART3 TX | PD8 |
| USART3 RX | PD9 |

NUCLEO-F429ZI의 ST-LINK Virtual COM Port를 사용할 수 있다. 터미널 설정은 다음과 같다.

```text
Baud rate: 115200
Data bits: 8
Parity: None
Stop bits: 1
Flow control: None
```

## 4. RTOS 빌드 및 플래시

### 4.1 요구 사항

- STM32CubeIDE
- ST-LINK USB 연결
- NUCLEO-F429ZI

프로젝트 파일:

```text
RTOS/.project
RTOS/.cproject
RTOS/greenRightRtos.ioc
```

프로젝트 이름은 `greenRightRtos`이고 MCU는 `STM32F429ZITx`이다.

### 4.2 STM32CubeIDE로 가져오기

1. STM32CubeIDE를 실행한다.
2. `File > Import`를 선택한다.
3. `General > Existing Projects into Workspace`를 선택한다.
4. `Select root directory`에 저장소의 `RTOS` 폴더를 지정한다.
5. `greenRightRtos` 프로젝트가 선택됐는지 확인한다.
6. `Copy projects into workspace`는 해제한다. 저장소 파일을 직접 사용하는 편이 변경 관리에 안전하다.
7. `Finish`를 누른다.

### 4.3 빌드

1. Project Explorer에서 `greenRightRtos`를 선택한다.
2. Build Configuration을 `Debug`로 선택한다.
3. `Project > Clean`을 실행한다.
4. `Project > Build Project`를 실행한다.

정상 빌드 시 주요 결과물은 다음 위치에 생성된다.

```text
RTOS/Debug/greenRightRtos.elf
RTOS/Debug/greenRightRtos.map
RTOS/Debug/greenRightRtos.list
```

> 저장소의 `RTOS/Debug/makefile`에는 과거 Windows PC의 절대 linker-script 경로가 남아 있다. 다른 PC에서 터미널의 `make`를 직접 실행하면 실패할 수 있으므로 STM32CubeIDE가 현재 환경의 빌드 파일을 다시 생성하도록 하는 방법을 권장한다.

### 4.4 플래시

1. NUCLEO-F429ZI의 ST-LINK USB를 연결한다.
2. `Run > Debug Configurations`를 연다.
3. `STM32 C/C++ Application`에서 `greenRightRtos Debug` 구성을 만든다.
4. Application이 `Debug/greenRightRtos.elf`인지 확인한다.
5. `Debug` 또는 `Run`을 실행한다.

### 4.5 CubeMX 코드 재생성 주의

`greenRightRtos.ioc`를 연 뒤 무심코 `Generate Code`를 실행하면 생성 코드가 바뀔 수 있다. 다음 파일들은 사용자 구현이 많으므로 먼저 Git diff를 확인하고 백업한다.

```text
RTOS/Core/Src/can_rx_task.c
RTOS/Core/Src/turnJudgeTask.c
RTOS/Core/Src/display_tasks.c
RTOS/Core/Src/personDetect.c
RTOS/Core/Src/lcd.c
RTOS/Core/Src/buzzer.c
```

## 5. Firmware 빌드 및 플래시

### 5.1 현재 저장소 상태

`firmware/`에는 애플리케이션 소스와 헤더만 있다.

```text
firmware/Inc/*.h
firmware/src/*.c
```

다음 빌드 필수 요소가 없으므로 `firmware/`에서 바로 `make`할 수 없다.

- STM32CubeIDE `.project`, `.cproject`
- `.ioc`
- CMSIS 및 STM32 HAL Driver
- startup assembly
- linker script
- IDE 또는 Makefile 빌드 설정

### 5.2 권장 방법: 원본 CubeIDE 프로젝트 사용

원본 펌웨어 CubeIDE 프로젝트가 있다면 다음 절차를 사용한다.

1. 원본 프로젝트를 STM32CubeIDE로 Import한다.
2. 원본 프로젝트의 `Core/Inc`에 `firmware/Inc` 파일을 반영한다.
3. 원본 프로젝트의 `Core/Src`에 `firmware/src` 파일을 반영한다.
4. include path에 `Core/Inc`가 포함됐는지 확인한다.
5. MCU가 `STM32F429ZITx`, 보드가 `NUCLEO-F429ZI`인지 확인한다.
6. System clock이 168MHz, APB1 peripheral clock이 42MHz인지 확인한다.
7. Clean 후 Build한다.
8. ST-LINK로 플래시한다.

Firmware의 직접 레지스터 설정은 APB1 42MHz를 기준으로 계산되어 있다. 특히 다음 설정이 clock과 일치해야 한다.

- CAN1: 500 kbit/s
- I2C1/I2C2: 100 kHz 설정값
- TIM3 기반 속도 센서 샘플링

### 5.3 원본 프로젝트가 없을 때

원본 프로젝트가 없다면 STM32CubeIDE에서 NUCLEO-F429ZI 프로젝트를 새로 만든 뒤 필요한 HAL/CMSIS/startup/linker 파일을 생성해야 한다. 최소한 다음 주변장치가 빌드 설정에 포함되어야 한다.

- GPIO
- RCC/PWR/CORTEX
- USART3
- CAN1 또는 CAN 관련 CMSIS 정의
- 코드에 남아 있는 ETH/USB handle과 초기화 함수가 컴파일될 수 있는 HAL 모듈

그 후 `firmware/Inc`와 `firmware/src`를 프로젝트의 `Core/Inc`, `Core/Src`에 통합한다. 생성된 `main.c`, interrupt 파일과 저장소 파일이 충돌할 수 있으므로 파일을 무조건 덮어쓰지 말고 diff로 병합한다.

### 5.4 Firmware 주요 연결

| 기능 | 핀 |
|---|---|
| BNO055 I2C1 SCL/SDA | PB8 / PB9 |
| ADXL345 I2C2 SCL/SDA | PF0 / PF1 |
| 속도 센서 입력 | PE2 |
| 우/좌 입력 버튼 | PC11 / PC12 |
| 방향 표시 LED | PA5 / PA6 |
| CAN1 RX/TX | PD0 / PD1 |
| Debug UART TX/RX | PD8 / PD9 |

## 6. Linux V2X/MQTT 빌드

### 6.1 지원 환경

Linux 프로그램은 SocketCAN의 Linux 전용 헤더를 사용한다. Raspberry Pi OS 또는 Debian/Ubuntu Linux에서 빌드한다. macOS나 일반 Windows에서 그대로 빌드할 수 없다.

### 6.2 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  pkg-config \
  libcjson-dev \
  libmosquitto-dev \
  can-utils
```

로컬 MQTT Broker도 이 장치에서 실행하려면 다음을 추가한다.

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
sudo systemctl status mosquitto
```

기본 Mosquitto 설정은 로컬 접속만 허용할 수 있다. 다른 Raspberry Pi나 Subcar가 접속해야 하면 Broker의 listener와 인증 정책을 별도로 설정해야 한다.

### 6.3 빌드

먼저 저장소 루트 경로를 환경변수로 지정한다.

```bash
export GREENRIGHT_ROOT=/path/to/greenRIght
```

그다음 Linux 프로그램을 빌드한다.

```bash
cd "$GREENRIGHT_ROOT/linux/v2x_mqtt"
make -f Makefile.rpi clean
make -f Makefile.rpi
```

정상 결과물:

```text
linux/v2x_mqtt/bin/v2x_mqtt_rpi
```

가짜 차량 C 도구도 빌드하려면:

```bash
make -f Makefile.rpi fake_vehicle_l15_to_l6
```

정상 결과물:

```text
linux/v2x_mqtt/bin/fake_vehicle_l15_to_l6
```

### 6.4 SocketCAN 설정

CAN 어댑터 드라이버가 정상적으로 로드되어 `can0`가 생성됐는지 확인한다.

```bash
ip link show can0
```

500 kbit/s로 활성화한다.

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

수신 확인:

```bash
candump -tz can0
```

Firmware 프레임은 CAN ID `0x200`, Linux가 RTOS로 보내는 프레임은 CAN ID `0x321`을 사용한다.

Raspberry Pi에서 MCP2515를 사용한다면 SPI 활성화와 `dtoverlay` 설정이 추가로 필요하다. oscillator 주파수와 interrupt GPIO는 사용 중인 CAN HAT에 따라 다르므로 해당 보드의 회로도와 제조사 설정을 따른다.

### 6.5 실행 인자

```text
v2x_mqtt_rpi <mqtt_host> <mqtt_port> <map_path> <vehicle_id>
```

인자를 생략했을 때 기본값:

| 항목 | 기본값 |
|---|---|
| MQTT host | `127.0.0.1` |
| MQTT port | `1883` |
| Map | `map/intersection_lanelet_v1.xml` |
| Vehicle ID | `1` |
| CAN interface | `can0` |

실행은 map 상대 경로가 올바르게 해석되도록 `linux/v2x_mqtt` 폴더에서 하는 것을 권장한다.

```bash
cd "$GREENRIGHT_ROOT/linux/v2x_mqtt"
./bin/v2x_mqtt_rpi 127.0.0.1 1883 map/intersection_lanelet_v1.xml 1
```

원격 Broker 예시:

```bash
./bin/v2x_mqtt_rpi 192.168.0.11 1883 map/intersection_lanelet_v1.xml 1
```

`Ctrl+C`로 종료한다.

### 6.6 환경변수

#### 실제 CAN 전체 사용

`CAN_MOCK`을 설정하지 않으면 실제 `can0`에서 Firmware 프레임을 받고 RTOS 프레임을 송신한다.

```bash
./bin/v2x_mqtt_rpi 127.0.0.1 1883 map/intersection_lanelet_v1.xml 1
```

#### CAN 수신 Mock

```bash
CAN_MOCK=1 ./bin/v2x_mqtt_rpi 127.0.0.1 1883 map/intersection_lanelet_v1.xml 1
```

이 모드에서는 코드 내부의 가상 Ego 차량을 사용한다. `CAN_TX_REAL`을 같이 지정하지 않으면 RTOS로 실제 CAN 프레임을 내보내지 않는다.

#### Mock Ego + 실제 RTOS CAN 송신

```bash
CAN_MOCK=1 CAN_TX_REAL=1 \
  ./bin/v2x_mqtt_rpi 127.0.0.1 1883 map/intersection_lanelet_v1.xml 1
```

#### `CAN_TX_REAL`의 현재 부가 동작

현재 코드에서 `CAN_TX_REAL=1`은 단순히 실제 송신 여부만 의미하지 않는다. Linux CAN 송신 스케줄에도 영향을 준다.

- 미설정: 송신 태스크의 50ms 주기마다 후보 차량/신호등 상태 송신 가능
- `CAN_TX_REAL=1`: 후보 상태 1000ms, 신호등 300ms 간격 적용

실제 장비 시험에서 50ms 송신을 유지하려면 `CAN_MOCK`을 끄고 `CAN_TX_REAL`은 설정하지 않는다. 이 이름과 동작은 혼동 가능성이 있으므로 추후 설정 분리를 권장한다.

## 7. MQTT 확인 및 테스트 발행

모든 MQTT 메시지를 확인한다.

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'v2x/#'
```

주요 토픽:

```text
v2x/vehicle/<vehicle_id>/status
v2x/trafficlight/<traffic_light_id>/status
```

신호등 테스트 발행 예시:

```bash
mosquitto_pub \
  -h 127.0.0.1 \
  -p 1883 \
  -t 'v2x/trafficlight/TL1/status' \
  -m '{"color":"green","time_left":8}' \
  -r
```

허용되는 color 문자열은 `red`, `yellow`, `green`이다.

## 8. 보조 차량 프로그램

### 8.1 Python 환경

```bash
cd "$GREENRIGHT_ROOT"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install paho-mqtt smbus2
```

`--no-bno055` 옵션만 사용할 경우 `smbus2`는 실제 실행에 필요하지 않지만, 동일 환경 재현을 위해 같이 설치하는 것을 권장한다.

### 8.2 센서 없이 실행

```bash
python subcar/subcar_mqtt_pub.py \
  --vehicle-id 2 \
  --mqtt-host 127.0.0.1 \
  --mqtt-port 1883 \
  --no-bno055 \
  --initial-heading-deg 90 \
  --start-x-cm 30 \
  --start-y-cm 177
```

- `w`: 설정한 속도로 이동
- `q`: 종료
- 기본 publish 주기: 100ms

### 8.3 BNO055 사용

Linux I2C를 활성화하고 장치를 확인한다.

```bash
sudo apt install -y i2c-tools
sudo raspi-config
i2cdetect -y 1
```

BNO055 기본 주소는 `0x28`이다.

```bash
python subcar/subcar_mqtt_pub.py \
  --vehicle-id 2 \
  --mqtt-host 192.168.0.11 \
  --mqtt-port 1883 \
  --i2c-bus 1 \
  --bno055-addr 0x28
```

## 9. 가짜 차량 C 도구

빌드 후 `linux/v2x_mqtt`에서 실행한다.

```bash
cd "$GREENRIGHT_ROOT/linux/v2x_mqtt"
./bin/fake_vehicle_l15_to_l6 127.0.0.1 1883 2 map/intersection_lanelet_v1.xml
```

인자:

```text
fake_vehicle_l15_to_l6 <mqtt_host> <mqtt_port> [vehicle_id] [map_path]
```

이 도구는 L15 방향의 가상 차량 상태를 50ms 주기로 MQTT에 발행한다.

## 10. 전체 통합 실행 예시

### 터미널 1: MQTT Broker 로그

```bash
sudo journalctl -u mosquitto -f
```

### 터미널 2: MQTT 전체 구독

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'v2x/#'
```

### 터미널 3: CAN 모니터

```bash
candump -tz can0
```

### 터미널 4: Linux 메인 프로그램

```bash
cd "$GREENRIGHT_ROOT/linux/v2x_mqtt"
./bin/v2x_mqtt_rpi 127.0.0.1 1883 map/intersection_lanelet_v1.xml 1
```

### 터미널 5: 보조 차량

```bash
cd "$GREENRIGHT_ROOT"
source .venv/bin/activate
python subcar/subcar_mqtt_pub.py \
  --vehicle-id 2 \
  --mqtt-host 127.0.0.1 \
  --mqtt-port 1883 \
  --no-bno055 \
  --start-x-cm 30 \
  --start-y-cm 177 \
  --initial-heading-deg 90
```

## 11. 정상 동작 확인 항목

### RTOS

- STM32CubeIDE 빌드 오류 없음
- UART에서 초기화 로그 출력
- CAN RX 인터럽트와 `CanRxTask` 동작
- Candidate Intro `0x4`, Status `0x5`, Traffic Light `0x6` 수신
- LCD와 부저 태스크 동작

### Firmware

- UART에 `CAN Init OK`
- BNO055/ADXL345 초기화 결과 출력
- `candump can0`에서 CAN ID `200` 확인
- 8-byte payload가 약 50ms 주기로 수신
- 최초 또는 full heartbeat 프레임에서 update mask `0x1F`

### Linux

- `[IntersectionMap] loaded`
- map의 lanelet/conflict-zone/traffic-light 개수 출력
- `[CanHandler] tx/rx socket opened if=can0`
- `[MQTT] connected vehicle_id=...`
- CAN ID `321` 송신 확인

### MQTT

- `v2x/vehicle/1/status` 발행
- 보조 차량의 `v2x/vehicle/2/status` 수신
- 신호등 토픽 수신
- 연결 단절 시 LWT 또는 timeout 처리

## 12. 문제 해결

### `socket failed`, `ioctl(SIOCGIFINDEX) failed`

```bash
ip link show can0
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
```

`can0`가 없다면 CAN 어댑터 드라이버 또는 Raspberry Pi device-tree overlay를 먼저 설정한다.

### CAN RX는 보이지만 TX가 계속 재전송됨

- CANH/CANL 반전 여부 확인
- GND 공통 연결 확인
- 종단 저항 확인
- 다른 CAN 노드가 ACK할 수 있도록 정상 모드로 동작하는지 확인
- 세 장치가 모두 500 kbit/s인지 확인

### `map load failed`

프로그램을 `linux/v2x_mqtt` 폴더에서 실행하거나 map 절대 경로를 전달한다.

```bash
./bin/v2x_mqtt_rpi 127.0.0.1 1883 \
  "$GREENRIGHT_ROOT/linux/v2x_mqtt/map/intersection_lanelet_v1.xml" 1
```

### `libmosquitto` 또는 `cJSON` 링크 오류

```bash
pkg-config --cflags --libs libmosquitto
pkg-config --cflags --libs libcjson
sudo apt install --reinstall libmosquitto-dev libcjson-dev pkg-config
```

### MQTT 연결 실패

```bash
systemctl status mosquitto
ss -lntp | grep 1883
mosquitto_sub -h <broker-ip> -p 1883 -t 'v2x/#' -v
```

원격 장치라면 방화벽, Broker listener, 인증 설정을 확인한다.

### RTOS command-line make가 Windows 경로를 참조함

STM32CubeIDE에서 Project Clean/Build를 수행해 현재 PC 기준 빌드 파일을 다시 생성한다. 자동 생성된 `Debug/makefile`을 직접 고치는 방식은 다음 코드 생성 시 사라질 수 있다.

### Update Mask 적용 후 일부 필드가 초기값으로 남음

수신 장치가 최초 full-mask 프레임 이후에 시작됐을 가능성이 있다. 송신 측에서 일정 주기마다 모든 필드를 표시한 full-mask heartbeat를 전송해야 한다.

## 13. Clean 명령

Linux:

```bash
cd "$GREENRIGHT_ROOT/linux/v2x_mqtt"
make -f Makefile.rpi clean
```

Python virtual environment 삭제가 필요하면 저장소 루트의 `.venv`를 제거한 뒤 다시 생성한다.

RTOS는 STM32CubeIDE의 `Project > Clean`을 사용한다.

## 14. 현재 저장소의 알려진 제약

1. `firmware/`는 독립 빌드 프로젝트가 아니라 소스 묶음이다.
2. RTOS 자동 생성 makefile에 과거 Windows 절대 경로가 남아 있다.
3. `CAN_TX_REAL`이 실제 송신 여부와 송신 주기 제한을 동시에 제어한다.
4. Firmware와 Linux 송신부의 12-bit timestamp 의미가 완전히 통일되어 있지 않다.
5. Update-mask 수신기는 늦은 접속 복구를 위해 주기적인 full-mask 프레임이 필요하다.
6. `ai/`에는 현재 빌드 가능한 소스와 모델 실행 명령이 없다.

이 제약을 해결하면 프로젝트 전체를 CI 또는 단일 스크립트로 재현할 수 있다.
