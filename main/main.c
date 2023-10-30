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
    xTaskCreate(ws2812_stats_printer_task, "ws2812_stats_printer", 1024, (void *)bus, 1, NULL);

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
    }
}

void app_main(void) {
    main_events = xQueueCreate(4, SHADER_MEM_SIZE);
    ESP_LOGI(TAG, "app main, start ledstrip task");

    init_flash();

    wifi_init();
    startWebserverTask(4096, 5, NULL, main_events);

    startUDPTask(4096, 5, NULL, main_events);
    // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(fps_task, "fps", 4096, NULL, 8, NULL);
    xTaskCreate(params_task, "params", 4096, NULL, 8, NULL);

    xTaskCreatePinnedToCore(led_strip_task, "led_strip", 2 * 4096, NULL, 8, NULL, 1);
}
