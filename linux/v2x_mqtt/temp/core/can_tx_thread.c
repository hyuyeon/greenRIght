#include "can_tx_thread.h"

#include <stdatomic.h>
#include <time.h>

static void* can_tx_thread_main(void* arg)
{
    AppContext* context = (AppContext*)arg;
    while (atomic_load(&context->running)) {
        /* TODO: select candidate vehicle and candidate traffic light from the same snapshot, then send explicit CAN TX result. */
        struct timespec ts = {0, 50 * 1000 * 1000};
        nanosleep(&ts, NULL);
    }
    return NULL;
}

int can_tx_thread_start(pthread_t* thread, AppContext* context)
{
    if (!thread || !context) return -1;
    return pthread_create(thread, NULL, can_tx_thread_main, context);
}
