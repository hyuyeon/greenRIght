#include <sys/timerfd.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <math.h>
#include <mosquitto.h>

#include "mqtt_thread.h"
#include "mqtt_topics.h"
#include "types.h"
#include "vehicle_codec.h"

#define MAX_OTHERS 64
#define VEHICLE_TIMEOUT_MS 500   /* [안정성] 이 시간(ms) 이상 갱신이 없으면 유령 차량으로 간주 */

static pthread_mutex_t   g_lock = PTHREAD_MUTEX_INITIALIZER;

// MAC 관련 변수 제거하고, vehicle_id만 남김
static uint8_t g_vehicle_id = VEHICLE_ID_NONE;

/*
 * [기초] ID 인덱싱으로 탐색 O(1)
 * vehicle_id를 배열 인덱스로 그대로 사용한다 (0 ~ MAX_OTHERS-1).
 * 기존의 "for문 돌면서 vehicle_id 비교" 방식(선형 검색, O(N))을 제거하고
 * g_others[v.vehicle_id] 로 바로 접근하도록 바꿨다.
 *
 * g_others_valid[i]           : i번 슬롯에 유효한 차량 정보가 있는지
 * g_others_last_updated_ms[i] : 마지막으로 해당 차량 정보를 수신한 "로컬" 시각(monotonic ms)
 *                                -> payload 안에 있는 timestamp(12bit, wrap-around)를 그대로 쓰면
 *                                   유령 차량 판정에 쓰기 애매해서, 수신 순간 우리 쪽에서 별도로 찍는다.
 */
static VehicleInfo g_others[MAX_OTHERS];
static bool         g_others_valid[MAX_OTHERS];
static uint64_t     g_others_last_updated_ms[MAX_OTHERS];

static TrafficLight g_traffic_lights[8];
static bool          g_traffic_light_valid[8];

static volatile bool g_running = true;

/* ----------------------------------------------------------------------- */
/* EgoVehicle(자차, RTOS 로부터 받는다고 가정) -> VehicleInfo 변환 */
static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)(ts.tv_nsec / 1000000);
}


static VehicleInfo ego_to_vehicle_info(const EgoVehicle* ego, uint8_t vehicle_id,
    uint16_t cz_x, uint16_t cz_y, uint8_t linked_tl)
{
    VehicleInfo v;
    memset(&v, 0, sizeof(v));

    v.vehicle_id = vehicle_id;
    v.x = ego->x & 0x03FF;             /* 10bit */
    v.y = ego->y & 0x07FF;             /* 11bit */
    v.speed = ego->speed;              /* 8bit */
    v.heading = ego->heading & 0x01FF; /* 9bit */
    v.lane = 0;                        /* 데모: 고정 차선 (실제로는 맵/RTOS 판단값 사용) */
    v.direction = ego->turn_signal & 0x03; /* 2bit */
    v.cz_x = cz_x & 0x03FF;
    v.cz_y = cz_y & 0x07FF;
    v.linked_tl = linked_tl & 0x03;

    v.timestamp = (uint16_t)(now_ms() % 4096); /* 12bit */

    return v;
}

/* 데모용 자차 상태 시뮬레이션 (실제로는 RTOS 로부터 UART/CAN 등으로 수신) */
static void simulate_ego(EgoVehicle* ego, double t_sec)
{
    /* 교차로 맵(0~1023, 0~2047) 안에서 원운동 시뮬레이션 */
    double cx = 512.0, cy = 1024.0, radius = 300.0;
    double angle = t_sec * 0.3; /* rad/s */

    ego->x = (uint16_t)(cx + radius * cos(angle));
    ego->y = (uint16_t)(cy + radius * sin(angle));
    ego->speed = 40; /* meter/min 가정, 0~150 범위 */
    ego->heading = (uint16_t)(((angle * 180.0 / M_PI) + 360.0));
    ego->heading %= 512;
    ego->turn_signal = 0; /* 00: 꺼짐 */
}

/* ----------------------------------------------------------------------- */
/* [안정성] 일정 시간(VEHICLE_TIMEOUT_MS) 이상 갱신되지 않은 차량 = 유령 차량으로 판단하고 슬롯 무효화 */

static void cleanup_stale_vehicles(void)
{
    uint64_t now = now_ms();

    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < MAX_OTHERS; i++) {
        if (!g_others_valid[i]) continue;
        if (now - g_others_last_updated_ms[i] > VEHICLE_TIMEOUT_MS) {
            printf("[CLEANUP] id=%d 유령 차량으로 판단, 삭제 (마지막 갱신 후 %llu ms 경과)\n",
                i, (unsigned long long)(now - g_others_last_updated_ms[i]));
            g_others_valid[i] = false;
        }
    }
    pthread_mutex_unlock(&g_lock);
}

static int create_periodic_timerfd(int period_ms)
{
    int fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (fd < 0) {
        perror("timerfd_create 실패");
        return -1;
    }

    struct itimerspec timer;
    memset(&timer, 0, sizeof(timer));

    timer.it_value.tv_sec = period_ms / 1000;
    timer.it_value.tv_nsec = (period_ms % 1000) * 1000000L;

    timer.it_interval.tv_sec = period_ms / 1000;
    timer.it_interval.tv_nsec = (period_ms % 1000) * 1000000L;

    if (timerfd_settime(fd, 0, &timer, NULL) < 0) {
        perror("timerfd_settime 실패");
        close(fd);
        return -1;
    }

    return fd;
}



static void on_connect(struct mosquitto* mosq, void* userdata, int rc)
{
    (void)userdata;
    if (rc != 0) {
        fprintf(stderr, "[MQTT] 연결 실패: %s\n", mosquitto_connack_string(rc));
        return;
    }
    printf("[MQTT] 브로커 연결 성공 (vehicle_id=%u)\n", g_vehicle_id);

    mosquitto_subscribe(mosq, NULL, TOPIC_VEHICLE_STATUS_WILDCARD, 0);
    mosquitto_subscribe(mosq, NULL, TOPIC_TRAFFICLIGHT_WILDCARD, 0);
    printf("[MQTT] 구독: %s , %s\n", TOPIC_VEHICLE_STATUS_WILDCARD, TOPIC_TRAFFICLIGHT_WILDCARD);
}


static void on_message(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* msg)
{
    (void)mosq; (void)userdata;
    if (!msg->topic || msg->payloadlen <= 0) return;

    char* payload = (char*)malloc(msg->payloadlen + 1);
    if (!payload) return;
    memcpy(payload, msg->payload, msg->payloadlen);
    payload[msg->payloadlen] = '\0';

    if (strncmp(msg->topic, TOPIC_VEHICLE_STATUS_PREFIX, strlen(TOPIC_VEHICLE_STATUS_PREFIX)) == 0) {
        VehicleInfo v;
        if (vehicle_info_from_json_string(payload, &v) && v.vehicle_id != g_vehicle_id) {
            /* [기초] vehicle_id를 배열 인덱스로 바로 사용 -> O(1) 갱신/삽입 */
            if (v.vehicle_id < MAX_OTHERS) {
                pthread_mutex_lock(&g_lock);
                g_others[v.vehicle_id] = v;
                g_others_valid[v.vehicle_id] = true;
                g_others_last_updated_ms[v.vehicle_id] = now_ms(); /* 수신 시각을 로컬에서 직접 기록 */
                pthread_mutex_unlock(&g_lock);
            }
        }
    }
    else if (strncmp(msg->topic, "v2x/trafficlight/", 18) == 0) {
        int tl_id = atoi(msg->topic + 18);
        if (tl_id >= 0 && tl_id < 8) {
            TrafficLight tl;
            if (traffic_light_from_json_string(payload, &tl)) {
                pthread_mutex_lock(&g_lock);
                g_traffic_lights[tl_id] = tl;
                g_traffic_light_valid[tl_id] = true;
                pthread_mutex_unlock(&g_lock);
            }
        }
    }

    free(payload);
}

void *mqttThread( void * arg) 	{

    MqttThreadArgs* mqtt_args = (MqttThreadArgs*)arg;

    const char* host = mqtt_args->host;
    int port = mqtt_args->port;
    g_vehicle_id = mqtt_args->vehicle_id;

    printf("[MQTT] Thread 시작\n");
    printf("[MQTT] Broker = %s:%d\n", host, port);
    printf("[MQTT] Vehicle ID = %u\n", g_vehicle_id);

    mosquitto_lib_init();

    /* MQTT Client ID 생성: 고유 vehicle_id를 이용해 충돌 방지 */
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "v2x_vehicle_%02u", g_vehicle_id);

    struct mosquitto* g_mosq = NULL;
    g_mosq = mosquitto_new(client_id, true, NULL);
    if (!g_mosq) { fprintf(stderr, "mosquitto_new 실패\n"); return NULL; }

    mosquitto_connect_callback_set(g_mosq, on_connect); //브로커랑 연결되면 바로 토픽 구독
    mosquitto_message_callback_set(g_mosq, on_message);

    char status_topic[64];
    snprintf(status_topic, sizeof(status_topic), TOPIC_VEHICLE_STATUS_FMT, (unsigned)g_vehicle_id);

    /*
     * [기술력] LWT (Last Will and Testament) 설정
     * mosquitto_connect() 이전에 반드시 설정해야 브로커에 등록된다.
     * 이 클라이언트가 정상 종료(disconnect) 없이 죽거나 네트워크가 끊겨
     * keepalive(60s) 동안 응답이 없으면, 브로커가 우리 대신
     * status_topic 으로 "빈 payload + retain=true" 메시지를 발행해준다.
     * -> 다른 차량들은 이 retained 빈 메시지를 받고 "이 vehicle_id는 이탈했다"고 판단할 수 있고,
     *    타임아웃(cleanup_stale_vehicles)보다 더 빠르게, 확실하게 이탈을 감지할 수 있다.
     */
    mosquitto_will_set(g_mosq, status_topic, 0, NULL, 0, true);

    if (mosquitto_connect(g_mosq, host, port, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "브로커(%s:%d) 연결 실패\n", host, port);
        return NULL;
    }
    
     /*
     * MQTT 수신 처리는 libmosquitto 내부 thread가 담당.
     * 메시지 수신 시 on_message() callback 실행.
     */
    if (mosquitto_loop_start(g_mosq) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "mosquitto_loop_start 실패\n");
        mosquitto_destroy(g_mosq);
        mosquitto_lib_cleanup();
        return NULL;
    }

    int pub_timer_fd = create_periodic_timerfd(VEHICLE_PUBLISH_PERIOD_MS);
    if (pub_timer_fd < 0) {
        mosquitto_loop_stop(g_mosq, true);
        mosquitto_destroy(g_mosq);
        mosquitto_lib_cleanup();
        return NULL;
    }

    while (g_running) {
        uint64_t expirations;
        ssize_t n = read(pub_timer_fd, &expirations, sizeof(expirations));

        if (n != sizeof(expirations)) {
            if (errno == EINTR) {
                continue;
            }

            perror("timerfd read 실패");
            break;
        }


        EgoVehicle ego;

        /*
         * 지금은 데모용.
         * 나중에는 CAN RX Thread가 갱신한 최신 자차 상태를 가져오면 됨.
         */
        simulate_ego(&ego, now_ms() / 1000.0);

        uint16_t cz_x = 512;
        uint16_t cz_y = 1024;
        uint8_t linked_tl = 0;

        VehicleInfo v = ego_to_vehicle_info(
            &ego,
            g_vehicle_id,
            cz_x,
            cz_y,
            linked_tl
        );

        char* json = vehicle_info_to_json_string(&v);

        if (json) {
            mosquitto_publish(
                g_mosq,
                NULL,
                status_topic,
                (int)strlen(json),
                json,
                0,
                true
            );

            printf("[MQTT TX] %s\n", json);

            free(json);
        }

        cleanup_stale_vehicles();
    }

    close(pub_timer_fd);



 
}
