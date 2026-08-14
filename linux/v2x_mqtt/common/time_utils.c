#include <stdint.h>
#include <time.h>

uint64_t get_utc_time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts); // NTP 동기화된 시스템 시간

    return (uint64_t)ts.tv_sec * 1000ULL +
           (uint64_t)ts.tv_nsec / 1000000ULL;
}