#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"

esp_err_t ws_broadcast(char *str);
void startWebserverTask(int stackSize, int prio, EventGroupHandle_t state, QueueHandle_t queue);

#endif
