#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <mosquitto.h>

#include "mqtt_topics.h"
#include "types.h"
#include "vehicle_codec.h"

#define DEFAULT_VEHICLE_ID 2
#define DEFAULT_SPEED 30
#define DEFAULT_HEADING 90
#define PUBLISH_PERIOD_MS 50
#define ROUTE_START_X 30
#define ROUTE_END_X 270
#define ROUTE_STEP_X 5
#define ROUTE_Y 177
#define RESET_PAUSE_MS 500

static volatile sig_atomic_t g_running = 1;

static void handle_signal(int signo)
{
    (void)signo;
    g_running = 0;
}

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000 * 1000;
    nanosleep(&ts, NULL);
}

static void safe_copy(char* dst, size_t dst_size, const char* src)
{
    if (!dst || dst_size == 0) return;
    snprintf(dst, dst_size, "%s", src ? src : "");
}

static void fill_fake_vehicle(VehicleInfo* vehicle, uint8_t vehicle_id, uint16_t x)
{
    memset(vehicle, 0, sizeof(*vehicle));

    vehicle->vehicle_id = vehicle_id;
    vehicle->x = x;
    vehicle->y = ROUTE_Y;
    vehicle->speed = DEFAULT_SPEED;
    vehicle->heading = DEFAULT_HEADING;
    safe_copy(vehicle->turn_state, sizeof(vehicle->turn_state), "straight");
    vehicle->timestamp_ms = monotonic_ms();

    if (x < 60) {
        safe_copy(vehicle->lanelet_id, sizeof(vehicle->lanelet_id), "L15");
        safe_copy(vehicle->linked_tl_id, sizeof(vehicle->linked_tl_id), "TL4");
    } else if (x < 240) {
        safe_copy(vehicle->lanelet_id, sizeof(vehicle->lanelet_id), "");
        safe_copy(vehicle->linked_tl_id, sizeof(vehicle->linked_tl_id), "TL4");
    } else {
        safe_copy(vehicle->lanelet_id, sizeof(vehicle->lanelet_id), "L6");
        safe_copy(vehicle->linked_tl_id, sizeof(vehicle->linked_tl_id), "");
    }

    if (x <= 230) {
        safe_copy(vehicle->conflict_zone_ids[0], sizeof(vehicle->conflict_zone_ids[0]), "cz1");
        vehicle->conflict_zone_count = 1;
    }
}

static bool publish_vehicle(struct mosquitto* mqtt, const char* topic, const VehicleInfo* vehicle)
{
    char* payload = vehicle_info_to_json_string(vehicle);
    if (!payload) return false;

    int rc = mosquitto_publish(mqtt, NULL, topic, (int)strlen(payload), payload, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[fake_vehicle] publish failed: %s\n", mosquitto_strerror(rc));
        free(payload);
        return false;
    }

    printf(
        "[fake_vehicle] pub id=%u x=%u y=%u lane=%s cz_count=%u tl=%s\n",
        vehicle->vehicle_id,
        vehicle->x,
        vehicle->y,
        vehicle->lanelet_id,
        vehicle->conflict_zone_count,
        vehicle->linked_tl_id
    );
    free(payload);
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <mqtt_host> <mqtt_port> [vehicle_id]\n", argv[0]);
        return 1;
    }

    const char* host = argv[1];
    int port = atoi(argv[2]);
    uint8_t vehicle_id = argc > 3 ? (uint8_t)atoi(argv[3]) : DEFAULT_VEHICLE_ID;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    mosquitto_lib_init();
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "greenright_fake_vehicle_%u", vehicle_id);

    struct mosquitto* mqtt = mosquitto_new(client_id, true, NULL);
    if (!mqtt) {
        fprintf(stderr, "[fake_vehicle] mosquitto_new failed\n");
        mosquitto_lib_cleanup();
        return 1;
    }

    int rc = mosquitto_connect(mqtt, host, port, 30);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[fake_vehicle] connect failed: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mqtt);
        mosquitto_lib_cleanup();
        return 1;
    }

    rc = mosquitto_loop_start(mqtt);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[fake_vehicle] loop start failed: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mqtt);
        mosquitto_lib_cleanup();
        return 1;
    }

    char topic[128];
    snprintf(topic, sizeof(topic), TOPIC_VEHICLE_STATUS_FMT, vehicle_id);
    printf("[fake_vehicle] publishing to %s every %d ms\n", topic, PUBLISH_PERIOD_MS);

    uint16_t x = ROUTE_START_X;
    while (g_running) {
        VehicleInfo vehicle;
        fill_fake_vehicle(&vehicle, vehicle_id, x);
        publish_vehicle(mqtt, topic, &vehicle);

        sleep_ms(PUBLISH_PERIOD_MS);

        if (x >= ROUTE_END_X) {
            sleep_ms(RESET_PAUSE_MS);
            x = ROUTE_START_X;
        } else {
            x = (uint16_t)(x + ROUTE_STEP_X);
        }
    }

    mosquitto_loop_stop(mqtt, true);
    mosquitto_disconnect(mqtt);
    mosquitto_destroy(mqtt);
    mosquitto_lib_cleanup();
    return 0;
}
