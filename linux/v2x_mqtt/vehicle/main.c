#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "types.h"
#include "mqtt_thread.h"

static uint8_t parse_vehicle_id(int argc, char** argv)
{
    if (argc > 3) {
        int input_id = atoi(argv[3]);

        if (input_id >= VEHICLE_ID_MIN && input_id <= VEHICLE_ID_MAX) {
            return (uint8_t)input_id;
        }

        fprintf(stderr, "ID는 %d 에서 %d 사이여야 합니다. 입력값: %d\n",
                VEHICLE_ID_MIN, VEHICLE_ID_MAX, input_id);
        exit(1);
    }

    printf("[INFO] ID가 지정되지 않아 기본값 ID 1을 사용합니다.\n");
    return 1;
}

int main(int argc, char** argv)
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? atoi(argv[2]) : 1883;
    uint8_t vehicle_id = parse_vehicle_id(argc, argv);

    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("[ID] 현재 차량에 고유 ID %u 할당 완료\n", vehicle_id);

    MqttThreadArgs mqtt_args = {
        .host = host,
        .port = port,
        .vehicle_id = vehicle_id,
    };

    pthread_t mqtt_thread;

    if (pthread_create(&mqtt_thread, NULL, mqttThread, &mqtt_args) != 0) {
        perror("pthread_create mqttThread 실패");
        return 1;
    }


    pthread_join(mqtt_thread, NULL);

    return 0;
}