#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "types.h"
#include "can.h"

/*
 * ---- CAN 프레임 내부 포맷 ----
 * CAN data[8] (64bit)를 MSB(빅엔디안) 기준으로 아래처럼 정의해서 사용한다.
 *
 * [ 헤더 24bit : bit63 ~ bit40 ]
 *   message_id  : 4bit  (bit63~60)
 *   timestamp   : 12bit (bit59~48)
 *   update_mask : 8bit  (bit47~40)
 *
 * [ 페이로드 40bit : bit39 ~ bit0 ] -> message_id에 따라 해석이 다르다.
 *
 * message_id 0000 (자차 상태)
 *   speed       : 8bit  (bit39~32) meter/min (0~150)
 *   x           : 10bit (bit31~22)
 *   y           : 11bit (bit21~11)
 *   heading     : 9bit  (bit10~2)
 *   turn_signal : 2bit  (bit1~0)

 */

#define CAN_MSG_ID_EGO_STATUS   0x0  /* 0000 */

/* message_id 0000 update_mask 비트 (요청하신 규칙: 1번째 비트=속도, 2번째=x, 3번째=y ...) */
#define UPD_BIT_SPEED       (1u << 0)
#define UPD_BIT_X           (1u << 1)
#define UPD_BIT_Y           (1u << 2)
#define UPD_BIT_HEADING     (1u << 3)
#define UPD_BIT_TURN_SIGNAL (1u << 4)

static int              g_can_sock = -1;
static pthread_t        g_can_thread;
static volatile bool    g_can_thread_running = false;

static pthread_mutex_t  g_lock = PTHREAD_MUTEX_INITIALIZER;

static EgoVehicle       g_ego;
static bool             g_ego_valid = false;


/* ----------------------------------------------------------------------- */
//msg 0000
static void parse_ego_status(uint16_t timestamp, uint8_t update_mask, uint64_t payload)
{
    pthread_mutex_lock(&g_lock);

    if (update_mask & UPD_BIT_SPEED)
        g_ego.speed = (uint8_t)((payload >> 32) & 0xFF);

    if (update_mask & UPD_BIT_X)
        g_ego.x = (uint16_t)((payload >> 22) & 0x3FF);

    if (update_mask & UPD_BIT_Y)
        g_ego.y = (uint16_t)((payload >> 11) & 0x7FF);

    if (update_mask & UPD_BIT_HEADING)
        g_ego.heading = (uint16_t)((payload >> 2) & 0x1FF);

    if (update_mask & UPD_BIT_TURN_SIGNAL)
        g_ego.turn_signal = (uint8_t)(payload & 0x3);

    g_ego.timestamp = timestamp;
    g_ego_valid = true;

    pthread_mutex_unlock(&g_lock);
}


/* data[8] -> 64bit로 합치고(MSB first) 헤더/페이로드로 분리 후 message_id로 분기 */
static void handle_can_frame(const uint8_t* data, uint8_t dlc)
{
    if (dlc < 8) {
        printf("[CAN] dlc=%u (8바이트 미만), 프로토콜 불일치로 무시\n", dlc);
        return;
    }

    uint64_t raw = 0;
    for (int i = 0; i < 8; i++) {
        raw = (raw << 8) | data[i];
    }

    uint8_t  message_id  = (uint8_t)((raw >> 60) & 0xF);
    uint16_t timestamp   = (uint16_t)((raw >> 48) & 0xFFF);
    uint8_t  update_mask = (uint8_t)((raw >> 40) & 0xFF);
    uint64_t payload     = raw & 0xFFFFFFFFFFULL; /* 하위 40bit */

    switch (message_id) {
        case CAN_MSG_ID_EGO_STATUS:
            parse_ego_status(timestamp, update_mask, payload);
            break;
        default:
            printf("[CAN] 알 수 없는 message_id=0x%X, 무시\n", message_id);
            break;
    }
}

/* ----------------------------------------------------------------------- */
/* 수신 스레드 본체.
 * read()가 블로킹 호출이므로, 커널 소켓 버퍼에 프레임이 들어올 때까지
 * 이 스레드는 CPU를 쓰지 않고 잠들어 있다가 데이터 도착 시 깨어난다. */
static void* can_rx_thread(void* arg)
{
    (void)arg;
    struct can_frame frame;

    printf("[CAN] 수신 스레드 시작 (read() 대기 중)\n");

    while (g_can_thread_running) {
        ssize_t nbytes = read(g_can_sock, &frame, sizeof(frame));

        if (nbytes < 0) {
            if (errno == EINTR) continue;      /* 시그널 인터럽트면 재시도 */
            if (!g_can_thread_running) break;   /* cleanup 과정에서 소켓이 닫힌 경우 */
            perror("[CAN] read 실패");
            break;
        }
        if (nbytes < (ssize_t)sizeof(struct can_frame)) {
            continue; /* 불완전한 프레임, 무시 */
        }

        handle_can_frame(frame.data, frame.can_dlc);
    }

    printf("[CAN] 수신 스레드 종료\n");
    return NULL;
}

/* ----------------------------------------------------------------------- */

int can_handler_init(const char* ifname)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    g_can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (g_can_sock < 0) {
        perror("[CAN] socket 생성 실패");
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

    if (ioctl(g_can_sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("[CAN] ioctl(SIOCGIFINDEX) 실패 - 인터페이스 이름 확인 필요");
        close(g_can_sock);
        g_can_sock = -1;
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(g_can_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[CAN] bind 실패");
        close(g_can_sock);
        g_can_sock = -1;
        return -1;
    }

    memset(&g_ego, 0, sizeof(g_ego));
    g_ego_valid = false;

    g_can_thread_running = true;
    if (pthread_create(&g_can_thread, NULL, can_rx_thread, NULL) != 0) {
        perror("[CAN] pthread_create 실패");
        close(g_can_sock);
        g_can_sock = -1;
        g_can_thread_running = false;
        return -1;
    }

    printf("[CAN] %s 인터페이스 bind 완료, 수신 스레드 기동\n", ifname);
    return 0;
}

void can_handler_cleanup(void)
{
    if (!g_can_thread_running) return;

    g_can_thread_running = false;

    /* read()에서 블로킹 중인 스레드를 깨우기 위해 소켓을 닫아 read가 에러로 리턴하게 만든다.
     * (참고: 프로세스 종료 시점에만 호출하는 용도라면 문제 없지만,
     *  런타임 중 빈번한 재시작이 필요하다면 self-pipe/eventfd + select 조합이 더 안전하다.) */
    if (g_can_sock >= 0) {
        close(g_can_sock);
        g_can_sock = -1;
    }

    pthread_join(g_can_thread, NULL);
}

bool can_handler_get_ego(EgoVehicle* out)
{
    if (!out) return false;

    pthread_mutex_lock(&g_lock);
    bool valid = g_ego_valid;
    if (valid) *out = g_ego;
    pthread_mutex_unlock(&g_lock);

    return valid;
}
