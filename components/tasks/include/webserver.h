#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "freertos/event_groups.h"

esp_err_t ws_broadcast(char *str);
void startWebserverTask(int stackSize, int prio, EventGroupHandle_t state, QueueHandle_t queue);

#endif

#ifdef __cplusplus
}
#endif
