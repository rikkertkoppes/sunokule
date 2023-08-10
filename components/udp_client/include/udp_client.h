#ifdef __cplusplus
extern "C" {
#endif

#ifndef UDPCLIENT_H
#define UDPCLIENT_H

#include "freertos/event_groups.h"

// number of channels reserved by fixture
// round to a nice number for easy calculations
#define CHANNEL_COUNT 8

typedef struct {
    uint16_t universe;
    uint8_t data[CHANNEL_COUNT];
} dmx_command_t;

void udp_client_task(void *pvParameters);
void startUDPTask(int stackSize, int prio, EventGroupHandle_t state, QueueHandle_t queue);

#endif

#ifdef __cplusplus
}
#endif
