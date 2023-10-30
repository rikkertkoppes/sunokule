#include "sdkconfig.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

// Rather crude, but tasks are returned in 'random' order from GetSystemState,
// so we just look them up by PID in the array. Array thus needs to be at
// least as large as the largest PID number.
#define MAX_TASKS 30

static const char *TAG = "taskmon";

void dump_tasks() {
    static struct { unsigned long ulLastTotal; } lastStats[MAX_TASKS];
    static unsigned long lastTotalTime;

    char *buffer = pvPortMalloc(2048);
    assert(buffer);
    char *pcWriteBuffer = buffer;
    *pcWriteBuffer = (char)0x00;

    UBaseType_t uxArraySize = MAX_TASKS;
    TaskStatus_t *pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    assert(pxTaskStatusArray);

    uint32_t ulTotalTime;
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalTime);
    unsigned long ulDiffTotal = ulTotalTime - lastTotalTime;
    lastTotalTime = ulTotalTime;

    sprintf(pcWriteBuffer, "name\t\tcpu ms\tcpu%%\tprio\tcore\tstack free\r\n");
    pcWriteBuffer += strlen(pcWriteBuffer);

    for (UBaseType_t x = 0; x < uxArraySize; x++) {
        unsigned long diffTime = pxTaskStatusArray[x].ulRunTimeCounter - lastStats[pxTaskStatusArray[x].xTaskNumber].ulLastTotal;
        lastStats[pxTaskStatusArray[x].xTaskNumber].ulLastTotal = pxTaskStatusArray[x].ulRunTimeCounter;
        unsigned long percentage = 100 * diffTime / ulDiffTotal;
        sprintf(pcWriteBuffer,
            "%-15s\t%u\t%u%%\t%u\t%hd\t%u\r\n",
            pxTaskStatusArray[x].pcTaskName,
            (unsigned int)diffTime / 1000, (unsigned int)percentage,
            (unsigned int)pxTaskStatusArray[x].uxCurrentPriority, (int)pxTaskStatusArray[x].xCoreID,
            (unsigned int)pxTaskStatusArray[x].usStackHighWaterMark);
        pcWriteBuffer += strlen(pcWriteBuffer);
    }

    vPortFree(pxTaskStatusArray);

    ESP_LOGI(TAG, "Task usage (elapsed %lums):\n%s", ulDiffTotal / 1000, buffer);

    vPortFree(buffer);
}
