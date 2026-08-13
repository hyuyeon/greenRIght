#include "can_handler.h"
#include "ntp_time.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef __linux__
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#endif

#define CAN_TX_ARBITRATION_ID 0x100
#define MSG_EGO_STATUS 0x0u
#define MSG_NTP_SYNC 0x2u
#define MSG_CANDIDATE_INTRO 0x4u
#define MSG_CANDIDATE_STATUS 0x5u
#define MSG_TRAFFIC_LIGHT 0x6u
#define TS_MASK 0x0FFFu

static void sleep_ms(long ms) { struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L }; nanosleep(&ts, NULL); }

static uint16_t timestamp12_from_ms(uint64_t timestamp_ms)
{
    return (uint16_t)(timestamp_ms & TS_MASK);
}

static void emit_ego(CanHandler* handler, const EgoVehicle* ego)
{
    if (handler && ego && handler->callbacks.on_ego) handler->callbacks.on_ego(ego, handler->callbacks.user_data);
}

/* The D3-G protocol carries a complete ego state in each 0x0 frame. */
static bool decode_can_payload(const uint8_t* data, uint8_t dlc, EgoVehicle* ego)
{
    if (!data || !ego || dlc < 8) return false;
    uint64_t raw = 0;
    for (int i = 0; i < 8; ++i) raw = (raw << 8) | data[i];
    if (((raw >> 60) & 0xFu) != MSG_EGO_STATUS) return false;
    ego->timestamp = (uint16_t)((raw >> 48) & TS_MASK);
    ego->speed = (uint8_t)((raw >> 32) & 0xFFu);
    ego->x = (uint16_t)((raw >> 22) & 0x3FFu);
    ego->y = (uint16_t)((raw >> 11) & 0x7FFu);
    ego->heading = (uint16_t)((raw >> 2) & 0x1FFu);
    ego->turn_signal = (uint8_t)(raw & 0x3u);
    return true;
}

static bool poll_mock(CanHandler* handler)
{
    sleep_ms(20);
    EgoVehicle ego = {0};
    ego.x = (uint16_t)(205 + (handler->mock_tick % 20));
    ego.y = (uint16_t)(70 + (handler->mock_tick % 35));
    ego.speed = 30;
    ego.turn_signal = TURN_SIGNAL_RIGHT;
    ego.timestamp = handler->mock_tick++;
    handler->last_ego = ego;
    handler->has_last_ego = true;
    emit_ego(handler, &ego);
    return true;
}

#ifdef __linux__
static bool open_socketcan(CanHandler* handler, const char* ifname)
{
    struct sockaddr_can addr = {0};
    struct ifreq ifr = {0};
    handler->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (handler->fd < 0) { perror("[CanHandler] socket failed"); return false; }
    strncpy(ifr.ifr_name, ifname ? ifname : "can0", IFNAMSIZ - 1);
    if (ioctl(handler->fd, SIOCGIFINDEX, &ifr) < 0) { perror("[CanHandler] ioctl failed"); close(handler->fd); handler->fd = -1; return false; }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(handler->fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("[CanHandler] bind failed"); close(handler->fd); handler->fd = -1; return false; }
    printf("[CanHandler] SocketCAN opened if=%s\n", ifr.ifr_name);
    return true;
}
#endif

bool can_handler_init(CanHandler* handler, const char* ifname, bool mock_mode, const CanHandlerCallbacks* callbacks)
{
    if (!handler) return false;
    memset(handler, 0, sizeof(*handler));
    handler->ifname = ifname;
    handler->fd = -1;
    handler->mock_mode = mock_mode;
    if (callbacks) handler->callbacks = *callbacks;
    if (mock_mode) {
#ifdef __linux__
        const char* tx_real = getenv("CAN_TX_REAL");
        if (tx_real && strcmp(tx_real, "1") == 0 && !open_socketcan(handler, ifname)) return false;
#endif
        handler->initialized = true;
        return true;
    }
#ifndef __linux__
    return false;
#else
    if (!open_socketcan(handler, ifname)) return false;
    handler->initialized = true;
    return true;
#endif
}

void can_handler_cleanup(CanHandler* handler)
{
    if (!handler || !handler->initialized) return;
#ifdef __linux__
    if (handler->fd >= 0) close(handler->fd);
#endif
    handler->fd = -1;
    handler->initialized = false;
}

bool can_handler_poll(CanHandler* handler, int timeout_ms)
{
    if (!handler || !handler->initialized) return false;
    if (handler->mock_mode) return poll_mock(handler);
#ifndef __linux__
    (void)timeout_ms;
    return false;
#else
    fd_set readfds;
    FD_ZERO(&readfds); FD_SET(handler->fd, &readfds);
    struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    int ready = select(handler->fd + 1, &readfds, NULL, NULL, &tv);
    if (ready <= 0) return false;
    struct can_frame frame;
    if (read(handler->fd, &frame, sizeof(frame)) != (ssize_t)sizeof(frame)) return false;
    EgoVehicle ego;
    if (!decode_can_payload(frame.data, frame.can_dlc, &ego)) return false;
    handler->last_ego = ego; handler->has_last_ego = true;
    emit_ego(handler, &ego);
    return true;
#endif
}

static bool send_frame(CanHandler* handler, uint8_t message_id, uint16_t timestamp12, uint64_t payload48)
{
    if (!handler || !handler->initialized) return false;
    uint64_t raw = ((uint64_t)(message_id & 0xFu) << 60) |
                   ((uint64_t)(timestamp12 & TS_MASK) << 48) |
                   (payload48 & 0xFFFFFFFFFFFFULL);
    if (handler->mock_mode && handler->fd < 0) return true;
#ifndef __linux__
    (void)raw;
    return false;
#else
    struct can_frame frame = {0};
    frame.can_id = CAN_TX_ARBITRATION_ID;
    frame.can_dlc = 8;
    for (int i = 7; i >= 0; --i) { frame.data[i] = (uint8_t)(raw & 0xFFu); raw >>= 8; }
    if (write(handler->fd, &frame, sizeof(frame)) != (ssize_t)sizeof(frame)) { perror("[CanHandler] tx write failed"); return false; }
    return true;
#endif
}

bool can_handler_send_ntp_sync(CanHandler* handler)
{
    uint64_t epoch_ms = ntp_time_sync_epoch_ms();
    uint64_t payload = ((epoch_ms & 0xFFFFFFFFFFULL) << 8) | (uint8_t)ntp_time_get_sync_status();
    return send_frame(handler, MSG_NTP_SYNC, timestamp12_from_ms(epoch_ms), payload);
}

bool can_handler_send_candidate_vehicle_intro(CanHandler* handler, uint8_t type, uint16_t x, uint16_t y, uint64_t timestamp)
{
    uint64_t payload = ((uint64_t)type << 32) | ((uint64_t)(x & 0x3FFu) << 22) | ((uint64_t)(y & 0x7FFu) << 11);
    return send_frame(handler, MSG_CANDIDATE_INTRO, timestamp12_from_ms(timestamp), payload);
}

bool can_handler_send_candidate_vehicle_status(CanHandler* handler, uint8_t type, const VehicleInfo* vehicle)
{
    uint8_t speed = vehicle ? vehicle->speed : 0;
    uint16_t x = vehicle ? vehicle->x : 0, y = vehicle ? vehicle->y : 0, heading = vehicle ? vehicle->heading : 0;
    uint64_t source_timestamp_ms = vehicle ? vehicle->timestamp_ms : ntp_time_sync_epoch_ms();
    uint64_t payload = ((uint64_t)type << 40) | ((uint64_t)speed << 32) | ((uint64_t)(x & 0x3FFu) << 22) | ((uint64_t)(y & 0x7FFu) << 11) | ((uint64_t)(heading & 0x1FFu) << 2);
    return send_frame(handler, MSG_CANDIDATE_STATUS, timestamp12_from_ms(source_timestamp_ms), payload);
}

bool can_handler_send_no_candidate_vehicle(CanHandler* handler) { return can_handler_send_candidate_vehicle_status(handler, 0x00u, NULL); }
bool can_handler_send_candidate_vehicle_unavailable(CanHandler* handler) { return can_handler_send_candidate_vehicle_status(handler, 0x80u, NULL); }

bool can_handler_send_traffic_light(CanHandler* handler, uint8_t tl_id, const TrafficLight* tl, uint16_t x, uint16_t y, uint8_t maneuver)
{
    uint8_t color = tl ? tl->color : 0, time_left = tl ? tl->time_left : 0;
    uint64_t payload =
        ((uint64_t)tl_id                    << 32) |
        ((uint64_t)(color & 0x3u)           << 30) |
        ((uint64_t)(time_left & 0xFu)       << 26) |
        ((uint64_t)(x & 0x3FFu)             << 16) |
        ((uint64_t)(y & 0x7FFu)             << 5)  |
        ((uint64_t)(maneuver & 0x3u)        << 3);
    uint64_t source_timestamp_ms = tl ? tl->timestamp_ms : ntp_time_sync_epoch_ms();
    return send_frame(handler, MSG_TRAFFIC_LIGHT, timestamp12_from_ms(source_timestamp_ms), payload);
}

bool can_handler_send_no_traffic_light(CanHandler* h, uint16_t x, uint16_t y, uint8_t m) { return can_handler_send_traffic_light(h, 0x00u, NULL, x, y, m); }
bool can_handler_send_traffic_light_unavailable(CanHandler* h, uint8_t m) { return can_handler_send_traffic_light(h, 0x80u, NULL, 0, 0, m); }
