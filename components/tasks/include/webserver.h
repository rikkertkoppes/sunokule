#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "freertos/event_groups.h"

void startWebserverTask(int stackSize, int prio, EventGroupHandle_t state);

#endif
