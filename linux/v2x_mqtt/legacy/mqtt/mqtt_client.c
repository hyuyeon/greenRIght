#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <mosquitto.h>

#include "types.h"
#include "mqtt_topics.h"
#include "vehicle_codec.h"
#include "mqtt_client.h"

#define VEHICLE_TIMEOUT_MS 500   /* [안정성] 이 시간(ms) 이상 갱신이 없으면 유령 차량으로 간주 */

static struct mosquitto* g_mosq = NULL;
static pthread_mutex_t   g_lock = PTHREAD_MUTEX_INITIALIZER;

static uint8_t g_vehicle_id = VEHICLE_ID_NONE;
static char    g_status_topic[64];

/*
 * [기초] ID 인덱싱으로 탐색 O(1)
 * vehicle_id를 배열 인덱스로 그대로 사용한다 (0 ~ MAX_OTHERS-1).
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

/* ----------------------------------------------------------------------- */

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)(ts.tv_nsec / 1000000);
}

/* ----------------------------------------------------------------------- */
/* MQTT 콜백 (mosquitto 내부 스레드에서 호출됨 -> 공유 데이터는 반드시 g_lock으로 보호) */

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

/* ----------------------------------------------------------------------- */

int mqtt_handler_init(const char* host, int port, uint8_t vehicle_id)
{
    g_vehicle_id = vehicle_id;

    mosquitto_lib_init();

    /* MQTT Client ID 생성: 고유 vehicle_id를 이용해 충돌 방지 */
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "v2x_vehicle_%02u", g_vehicle_id);

    g_mosq = mosquitto_new(client_id, true, NULL);
    if (!g_mosq) {
        fprintf(stderr, "mosquitto_new 실패\n");
        return -1;
    }

    mosquitto_connect_callback_set(g_mosq, on_connect);
    mosquitto_message_callback_set(g_mosq, on_message);

    snprintf(g_status_topic, sizeof(g_status_topic), TOPIC_VEHICLE_STATUS_FMT, (unsigned)g_vehicle_id);

    /*
     * [기술력] LWT (Last Will and Testament) 설정
     * mosquitto_connect() 이전에 반드시 설정해야 브로커에 등록된다.
     * 이 클라이언트가 정상 종료(disconnect) 없이 죽거나 네트워크가 끊겨
     * keepalive(60s) 동안 응답이 없으면, 브로커가 우리 대신
     * status_topic 으로 "빈 payload + retain=true" 메시지를 발행해준다.
     * -> 다른 차량들은 이 retained 빈 메시지를 받고 "이 vehicle_id는 이탈했다"고 판단할 수 있고,
     *    타임아웃(mqtt_cleanup_stale_vehicles)보다 더 빠르게, 확실하게 이탈을 감지할 수 있다.
     */
    mosquitto_will_set(g_mosq, g_status_topic, 0, NULL, 0, true);

    if (mosquitto_connect(g_mosq, host, port, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "브로커(%s:%d) 연결 실패\n", host, port);
        return -1;
    }

    mosquitto_loop_start(g_mosq);
    return 0;
}

void mqtt_handler_cleanup(void)
{
    if (!g_mosq) return;
    mosquitto_loop_stop(g_mosq, true);
    mosquitto_destroy(g_mosq);
    mosquitto_lib_cleanup();
    g_mosq = NULL;
}

void mqtt_publish_vehicle_info(const VehicleInfo* v)
{
    if (!g_mosq || !v) return;

    char* json = vehicle_info_to_json_string(v);
    if (json) {
        mosquitto_publish(g_mosq, NULL, g_status_topic, (int)strlen(json), json, 0, true);
        free(json);
    }
}

/* [안정성] 일정 시간(VEHICLE_TIMEOUT_MS) 이상 갱신되지 않은 차량 = 유령 차량으로 판단하고 슬롯 무효화 */
void mqtt_cleanup_stale_vehicles(void)
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

int mqtt_get_others_count(void)
{
    int count = 0;

    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < MAX_OTHERS; i++) {
        if (g_others_valid[i]) count++;
    }
    pthread_mutex_unlock(&g_lock);

    return count;
}

bool mqtt_get_other_vehicle(uint8_t vehicle_id, VehicleInfo* out)
{
    if (vehicle_id >= MAX_OTHERS || !out) return false;

    pthread_mutex_lock(&g_lock);
    bool valid = g_others_valid[vehicle_id];
    if (valid) *out = g_others[vehicle_id];
    pthread_mutex_unlock(&g_lock);

    return valid;
}

bool mqtt_get_traffic_light(uint8_t tl_id, TrafficLight* out)
{
    if (tl_id >= 8 || !out) return false;

    pthread_mutex_lock(&g_lock);
    bool valid = g_traffic_light_valid[tl_id];
    if (valid) *out = g_traffic_lights[tl_id];
    pthread_mutex_unlock(&g_lock);

    return valid;
}