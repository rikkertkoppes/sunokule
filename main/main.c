/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "sdkconfig.h"

#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "functions.h"
#include "ledstorage.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mapping.h"
#include "math.h"
#include "primitives.h"
#include "storage.h"
#include "udp_client.h"
#include "webserver.h"
#include "wifi_conn.h"
#include "ws2812.h"
#include "taskmon.h"

// update mapping => colors off, strip 0, ok, others wrong
// swap data pins => no change
// swap refresh order (1,0,2) => no change
// wait after every write => all ok
// adjust timing => all ok

#define SHADER_MEM_SIZE 1024
#define PROG_MEM_SIZE 256

static const char *TAG = "shader";
QueueHandle_t main_events;

typedef unsigned char byte;
float clk = 0;
byte power = 0;  // power off

static const int num_pixels[] = {
    CONFIG_SK_NUM_LEDS_0,
    CONFIG_SK_NUM_LEDS_1,
    CONFIG_SK_NUM_LEDS_2,
};
static const int gpio_pins[] = {
    CONFIG_SK_DATA_PIN_0,
    CONFIG_SK_DATA_PIN_1,
    CONFIG_SK_DATA_PIN_2,
};
static const int num_strands = sizeof(num_pixels) / sizeof(num_pixels[0]);

/**
 * expecting rgb as a float in [0,1]
 */
void set_pixel_rgb(ws2812_pixel_t *pixels, u_int32_t index, float r, float g, float b) {
    // remap r,g,b values to suppress low values
    // r = r * r * r;
    // g = g * g * g;
    // b = b * b * b;

    r = clamp(r, 0, 1);
    g = clamp(g, 0, 1);
    b = clamp(b, 0, 1);

    // ESP_LOGI(TAG, "rgb %f %f %f", r * 255, g * 255, b * 255);

    pixels[index] = (ws2812_pixel_t){
        .red = 255 * r,
        .green = 255 * g,
        .blue = 255 * b,
    };
}

static byte mem[SHADER_MEM_SIZE];
static byte shader[PROG_MEM_SIZE];

int framecount = 0;

// frame renderer, by using a shader and incoming data
void frame(ws2812_strands_t strands, byte *shader, float clk) {
    int j = 0;
    for (int s = 0; s < num_strands; s++) {
        for (int p = 0; p < num_pixels[s]; p++, j++) {
            // byte version = shader[0];
            // byte id = shader[1];

            // uint16_t memStart = getUint16(shader, 2);
            uint16_t memSize = getUint16(shader, 4);
            uint16_t progStart = getUint16(shader, 6);
            // uint16_t progSize = getUint16(shader, 8);

            uint16_t mem_end = memSize / 4;

            // set params
            // copy time into end of memory
            setFloat(mem, mem_end, clk);
            // copy current led index into memory
            setFloat(mem, mem_end + 1, (float)j);
            // copy current led mapping directly from in memory
            setMapping(mem, mem_end + 2, mapping, j);

            // execute
            byte counter = 0;
            byte *prog = shader + progStart;
            execute(mem, prog, &counter);

            // program counter is currently at the end program instruction
            // master value is the first param of the end program instruction
            byte _master = data_fetch(mem, prog, &counter);
            // result value is the second param of the end program instruction
            byte _result = data_fetch(mem, prog, &counter);

            // retrieve rgb values
            float m = getFloat(mem, _master);
            float r = m * getFloat(mem, _result);
            float g = m * getFloat(mem, _result + 1);
            float b = m * getFloat(mem, _result + 2);

            // ESP_LOGI(TAG, "counter %i, rgb %f %f %f", counter, r, g, b);
            set_pixel_rgb(strands[s], p, r, g, b);
        }
    }
    framecount++;
}

/**
 * Last idle run timestamp for each CPU.
 *
 * We use this timestamp to occassionally 'yield'
 * to the idle tasks, to ensure any cleanup is happening
 * and watchdogs are triggered. We want to keep using
 * the watchdogs, to ensure we don't accidentally cause
 * a stall anywhere.
 * Normally, we could just put a yield in the application
 * idle hook, and have the background task also run at
 * prio 0, effectively giving 100% of the idle time to
 * our task. However, this doesn't work due to the unfair
 * scheduling implemented in ESP's patched FreeRTOS.
 * See https://docs.espressif.com/projects/esp-idf/en/v5.1.1/esp32/api-reference/system/freertos_idf.html#time-slicing
 *
 * ESP timer is 64 bit (us), but we'll be out of sync
 * with the main loop by only <5s, so that easily fits
 * in 32 bit, which can be atomically read/written.
 */
static volatile uint32_t last_idle_run[2];

/**
 * Mark each time the idle loop got a chance to run.
 * Needs to be in IRAM.
 */
IRAM_ATTR void vApplicationIdleHook(void) {
    BaseType_t core = xPortGetCoreID();
    assert(core >= 0 && core <= 1);
    last_idle_run[core] = (uint32_t)esp_timer_get_time();
}

/**
 * Maximum time in us before yielding to idle tasks.
 */
#define WATCHDOG_FEED_INTERVAL_US (CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000000UL - 500000)

/**
 * Sleep for a bit if needed to allow idle tasks to run,
 * to prevent task watchdog timeout.
 */
static inline void yield_to_idle_if_needed() {
    BaseType_t core = xPortGetCoreID();
    assert(core >= 0 && core <= 1);

    uint32_t now = (uint32_t)esp_timer_get_time();
    if (now - last_idle_run[core] >= WATCHDOG_FEED_INTERVAL_US) {
        // 'Yield' for one tick, should be enough to give idle tasks some runtime
        vTaskDelay(1);
    }
}

static void fps_task(void *pvParameters) {
    while (true) {
        dump_tasks();

        ESP_LOGI(TAG, "fps %i", framecount);
        char data[80];
        sprintf(data, "{\"fps\":%i,\"power\":%i}", framecount, power);
        ws_broadcast(data);

        framecount = 0;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void setParams(uint8_t *data) {
    uint8_t num_params = data[0];
    // data comes in quintets
    for (uint8_t i = 0; i < num_params; i++) {
        // first byte is memory address
        uint8_t ptr = data[1 + i * 5];
        float f;
        // next 4 bytes are float data, copy to local float variable
        memcpy(&f, data + 2 + i * 5, 4);
        ESP_LOGI(TAG, "set mem %i to %f", ptr, f);
        // set memory address to float value
        setFloat(mem, ptr, f);
    }
}

static void params_task(void *pvParameters) {
    uint8_t data[SHADER_MEM_SIZE];
    while (true) {
        if (xQueueReceive(main_events, data, 1000 / portTICK_PERIOD_MS)) {
            printf("part of the buffer:");
            for (int i = 0; i < 20; i++) {
                // printf("%02X ", data[i]);
                printf("%i ", data[i]);
            }
            printf("\n");
            uint8_t datatype = data[0];
            switch (datatype) {

                case 0: // shader packet
                    // store shader in NVS
                    saveShader(data + 2, data[1]);
                    // reset shader time
                    clk = 0;
                    setShader(shader, data + 2, data[1]);
                    // reinitialize working memory
                    setMem(mem, shader);
                    break;

                case 1: // params packet
                    setParams(data + 1);
                    break;

                case 2: // power on/ off packet
                    // reset shader time
                    clk = 0;
                    power = data[1];
                    savePowerState(power);
                    break;

                case 3: // dmx packet
                    // put dmx data into memory
                    setDMX(mem, data);
                    printf("%i %i %i \n", getDMX(mem, 0), getDMX(mem, 1), getDMX(mem, 2));
                    break;
            }
        }
    }
}

static void clear_strands(ws2812_strands_t strands) {
    for (int s = 0; s < num_strands; s++) {
        for (int i = 0; i < num_pixels[s]; i++) {
            strands[s][i] = (ws2812_pixel_t){0};
        }
    }
}

static void ws2812_stats_printer_task(void *arg) {
    ws2812_bus_handle_t bus = (ws2812_bus_handle_t)arg;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ws2812_stats_t stats = {0};
        ws2812_get_stats(bus, &stats);
        if (stats.max_late_buffers > 0 || stats.dma_underrun_errors > 0) {
            // ws2812_reset_stats(bus);
            ESP_LOGI(
                TAG,
                "max_late_buffers=%d, dma_underrun_errors=%d, max_int_time=%" PRIu32 ", needed_late_buffers=%d",
                stats.max_late_buffers,
                stats.dma_underrun_errors, stats.max_int_time, stats.needed_late_buffers);
        }
    }
}

static void led_strip_task(void *pvParameters) {
    // Instantiate WS2812 driver
    ws2812_bus_config_t config = {
        .num_strands = num_strands,
        .max_late_buffers = 2,
    };
    for (int s = 0; s < num_strands; s++) {
        config.num_pixels[s] = num_pixels[s];
        config.gpio_pins[s] = gpio_pins[s];
        ESP_LOGI(TAG, "configure %i leds on pin %i", num_pixels[s], gpio_pins[s]);
    }
    ws2812_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(ws2812_new(&config, &bus));
    xTaskCreate(ws2812_stats_printer_task, "ws2812_stats_printer", 2048, (void *)bus, 2, NULL);

    float tick = 0.001;

    // uint16_t start_rgb = 0;
    uint32_t time = clock();

    // get shader from NVS
    readShader(shader);

    power = readPowerState();

    // initialize working memory
    setMem(mem, shader);

    while (true) {
        // t and elapsed are in ms;
        uint32_t t = clock();
        uint32_t elapsed = t - time;
        time = t;
        // clock value in seconds
        clk += (float)elapsed * tick;

        ws2812_strands_t strands = ws2812_get_strands(bus);
        if (power) {
            frame(strands, shader, clk);
        } else {
            clear_strands(strands);
        }
        ws2812_enqueue_strands(bus, strands);

        if (!power) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        yield_to_idle_if_needed();
    }
}

void app_main(void) {
    main_events = xQueueCreate(4, SHADER_MEM_SIZE);

    init_flash();

    wifi_init();
    startWebserverTask(4096, 5, NULL, main_events);

    startUDPTask(4096, 5, NULL, main_events);
    // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(fps_task, "fps", 4096, NULL, 8, NULL);
    xTaskCreate(params_task, "params", 4096, NULL, 8, NULL);

    xTaskCreatePinnedToCore(led_strip_task, "led_strip", 2 * 4096, NULL, 1, NULL, 1);
}
