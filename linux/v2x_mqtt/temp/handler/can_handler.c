#include "can_handler.h"

#include <errno.h>
#include <stdio.h>
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

#define CAN_MSG_ID_EGO_STATUS 0x0

#define UPD_BIT_SPEED       (1u << 0)
#define UPD_BIT_X           (1u << 1)
#define UPD_BIT_Y           (1u << 2)
#define UPD_BIT_HEADING     (1u << 3)
#define UPD_BIT_TURN_SIGNAL (1u << 4)

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000 * 1000;
    nanosleep(&ts, NULL);
}

static void emit_ego(CanHandler* handler, const EgoVehicle* ego)
{
    if (handler && ego && handler->callbacks.on_ego) {
        handler->callbacks.on_ego(ego, handler->callbacks.user_data);
    }
}

#ifdef __linux__
static bool decode_ego_status(uint16_t timestamp, uint8_t update_mask, uint64_t payload, EgoVehicle* ego)
{
    if (!ego) return false;

    memset(ego, 0, sizeof(*ego));

    if (update_mask & UPD_BIT_SPEED) {
        ego->speed = (uint8_t)((payload >> 32) & 0xFF);
    }
    if (update_mask & UPD_BIT_X) {
        ego->x = (uint16_t)((payload >> 22) & 0x3FF);
    }
    if (update_mask & UPD_BIT_Y) {
        ego->y = (uint16_t)((payload >> 11) & 0x7FF);
    }
    if (update_mask & UPD_BIT_HEADING) {
        ego->heading = (uint16_t)((payload >> 2) & 0x1FF);
    }
    if (update_mask & UPD_BIT_TURN_SIGNAL) {
        ego->turn_signal = (uint8_t)(payload & 0x3);
    }

    ego->timestamp = timestamp;
    return true;
}

static bool decode_can_payload(const uint8_t* data, uint8_t dlc, EgoVehicle* ego)
{
    if (!data || !ego || dlc < 8) return false;

    uint64_t raw = 0;
    for (int i = 0; i < 8; i++) {
        raw = (raw << 8) | data[i];
    }

    uint8_t message_id = (uint8_t)((raw >> 60) & 0xF);
    if (message_id != CAN_MSG_ID_EGO_STATUS) {
        return false;
    }

    uint16_t timestamp = (uint16_t)((raw >> 48) & 0xFFF);
    uint8_t update_mask = (uint8_t)((raw >> 40) & 0xFF);
    uint64_t payload = raw & 0xFFFFFFFFFFULL;
    return decode_ego_status(timestamp, update_mask, payload, ego);
}

#endif

static bool poll_mock(CanHandler* handler)
{
    sleep_ms(20);

    EgoVehicle ego;
    memset(&ego, 0, sizeof(ego));
    ego.x = (uint16_t)(150 + (handler->mock_tick % 60));
    ego.y = (uint16_t)(40 + (handler->mock_tick % 80));
    ego.speed = 30;
    ego.heading = 0;
    ego.turn_signal = 0;
    ego.timestamp = handler->mock_tick++;

    emit_ego(handler, &ego);
    return true;
}

bool can_handler_init(
    CanHandler* handler,
    const char* ifname,
    bool mock_mode,
    const CanHandlerCallbacks* callbacks
)
{
    if (!handler) return false;
    memset(handler, 0, sizeof(*handler));
    handler->ifname = ifname;
    handler->fd = -1;
    handler->mock_mode = mock_mode;
    if (callbacks) {
        handler->callbacks = *callbacks;
    }

    if (mock_mode) {
        handler->initialized = true;
        printf("[CanHandler] init if=%s mock=1\n", ifname ? ifname : "none");
        return true;
    }

#ifndef __linux__
    fprintf(stderr, "[CanHandler] real SocketCAN is only available on Linux. Use mock mode on this platform.\n");
    return false;
#else
    struct sockaddr_can addr;
    struct ifreq ifr;

    handler->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (handler->fd < 0) {
        perror("[CanHandler] socket failed");
        return false;
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname ? ifname : "can0", IFNAMSIZ - 1);

    if (ioctl(handler->fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("[CanHandler] ioctl(SIOCGIFINDEX) failed");
        close(handler->fd);
        handler->fd = -1;
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(handler->fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[CanHandler] bind failed");
        close(handler->fd);
        handler->fd = -1;
        return false;
    }

    handler->initialized = true;
    printf("[CanHandler] init if=%s mock=0\n", ifr.ifr_name);
    return true;
#endif
}

void can_handler_cleanup(CanHandler* handler)
{
    if (!handler || !handler->initialized) return;
#ifdef __linux__
    if (handler->fd >= 0) {
        close(handler->fd);
        handler->fd = -1;
    }
#endif
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
    FD_ZERO(&readfds);
    FD_SET(handler->fd, &readfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ready = select(handler->fd + 1, &readfds, NULL, NULL, &tv);
    if (ready < 0) {
        if (errno == EINTR) return false;
        perror("[CanHandler] select failed");
        return false;
    }
    if (ready == 0) return false;

    struct can_frame frame;
    ssize_t nbytes = read(handler->fd, &frame, sizeof(frame));
    if (nbytes < 0) {
        if (errno != EINTR) perror("[CanHandler] read failed");
        return false;
    }
    if (nbytes < (ssize_t)sizeof(struct can_frame)) return false;

    EgoVehicle ego;
    if (!decode_can_payload(frame.data, frame.can_dlc, &ego)) return false;
    emit_ego(handler, &ego);
    return true;
#endif
}
