/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>

#include "addr_from_stdin.h"
#include "driver/rmt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "functions.h"
#include "led_strip.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mapping.h"
#include "math.h"
#include "nvs_flash.h"
#include "primitives.h"
#include "protocol_examples_common.h"
#include "webserver.h"
#include "wifi_conn.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_CHANNEL1 RMT_CHANNEL_1
#define DATA_PIN 26
#define DATA_PIN1 27
#define NUM_LEDS 300

#define SHADER_MEM_SIZE 256
#define PROG_MEM_SIZE 256

static const char *TAG = "shader";
QueueHandle_t main_events;

typedef unsigned char byte;

led_strip_t *stripCreateInit(gpio_num_t gpio_num, rmt_channel_t channel, uint32_t num_leds) {
    // strip 0
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio_num, channel);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(num_leds, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    return strip;
}

/**
 * expecting rgb as a float in [0,1]
 */
void led_strip_setPixelRGB(led_strip_t *strip, u_int32_t index, float r, float g, float b) {
    strip->set_pixel(strip, index, 255 * r, 255 * g, 255 * b);
}

static byte mem[SHADER_MEM_SIZE];
static byte shader[PROG_MEM_SIZE];

// wheather shader
byte _weather[] = {
    8,

    0, 0, 0, 0,
    185, 184, 184, 62,
    180, 179, 51, 63,
    0, 0, 128, 63,
    0, 0, 128, 63,
    214, 213, 85, 63,
    0, 0, 0, 0,
    0, 0, 128, 63,

    1, 0, 3, 16,
    1, 4, 7, 20,
    2, 11, 16, 20, 24,
    0};

byte _rainbow[] = {
    11,

    154, 153, 153, 62,
    0, 0, 128, 63,
    0, 0, 128, 63,
    0, 0, 0, 0,
    0, 0, 0, 63,
    0, 0, 0, 63,
    0, 0, 0, 0,
    0, 0, 128, 63,
    0, 0, 128, 63,
    0, 0, 128, 63,
    0, 0, 128, 63,

    5, 0, 1, 19,
    3, 2, 19, 3, 4, 5, 6, 7, 16, 11, 20,
    5, 8, 9, 21,
    4, 20, 10, 21, 22,
    0};

byte _colorloop[] = {
    10,

    0, 0, 128, 63,
    205, 204, 76, 62,
    0, 0, 0, 0,
    0, 0, 0, 63,
    0, 0, 0, 63,
    0, 0, 0, 0,
    0, 0, 128, 63,
    0, 0, 0, 0,
    0, 0, 128, 63,
    0, 0, 128, 63,

    3, 0, 1, 2, 3, 4, 5, 6, 7, 10, 18,
    4, 18, 8, 9, 19,
    0};

byte _scan2[] = {
    20,

    51, 51, 51, 63,
    0, 0, 128, 63,
    154, 153, 153, 62,
    0, 0, 128, 63,
    0, 0, 128, 64,
    0, 0, 64, 64,
    0, 0, 128, 191,
    0, 0, 0, 64,
    0, 0, 0, 64,
    0, 0, 0, 0,
    0, 0, 128, 63,
    0, 0, 128, 191,
    0, 0, 0, 64,
    0, 0, 0, 64,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 128, 63,
    0, 0, 160, 64,
    205, 204, 204, 61,
    0, 0, 128, 63,

    5, 0, 1, 28,
    5, 2, 3, 29,
    6, 29, 4, 5, 30,
    6, 30, 6, 7, 31,
    3, 8, 28, 31, 30, 30, 9, 10, 25, 20, 32,
    6, 20, 11, 12, 33,
    3, 13, 28, 14, 30, 15, 30, 16, 25, 33, 34,
    6, 32, 34, 17, 35,
    4, 18, 19, 35, 36,
    0};

byte _rain[] = {
    14,

    0, 0, 224, 64,
    0, 0, 0, 192,
    0, 0, 0, 0,
    0, 0, 128, 62,
    0, 0, 0, 64,
    0, 0, 0, 0,
    0, 0, 128, 63,
    0, 0, 0, 0,
    0, 0, 0, 63,
    0, 0, 0, 0,
    0, 0, 0, 63,
    0, 0, 128, 63,
    205, 204, 76, 62,
    0, 0, 128, 63,

    7, 16, 0, 22,                           // trig
    6, 22, 1, 2, 23,                        // math
    6, 23, 3, 4, 24,                        // math
    6, 14, 22, 5, 25,                       // math
    3, 24, 6, 7, 8, 9, 10, 11, 17, 25, 26,  // waveform
    4, 12, 13, 26, 27,                      // hsv2rgb
    0,                                      // end program
};

byte _solid[] = {
    4,

    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 128, 63,

    1, 0, 3, 12,
    0};

int framecount = 0;

// frame renderer, by using a shader and incoming data
void frame(led_strip_t *strip, led_strip_t *strip1, byte *shader, float clk) {
    for (int j = 0; j < NUM_LEDS * 2; j += 1) {
        size_t mem_end = shader[0];
        // set params
        setFloat(mem, mem_end, clk);
        // copy mapping directly from in memory
        setMapping(mem, mem_end + 2, mapping, j);

        // execute
        byte counter = 0;
        byte *prog = shader + shader[0] * 4 + 1;
        execute(mem, prog, &counter);

        // get result pointer
        byte _result = prog[counter - 2];

        // retrieve rgb values
        float r = getFloat(mem, _result);
        float g = getFloat(mem, _result + 1);
        float b = getFloat(mem, _result + 2);

        // ESP_LOGI(TAG, "rgb %f %f %f", r, g, b);
        if (j < NUM_LEDS) {
            led_strip_setPixelRGB(strip, j, r, g, b);
        } else {
            led_strip_setPixelRGB(strip1, j - NUM_LEDS, r, g, b);
        }
    }
    framecount++;
    /**
     * note: letting the rmt hardware write multiple strips in parallel,
     * then waiting for them to finish
     *
     * this allows for faster framerates by segmenting the strip
     * otherwise the strip timing becomes a bottleneck
     *
     * sequentially 2x300 leds -> 33fps for scan shader
     * parallel 2x300 leds -> 45fps for scan shader
     *
     */
    strip->refresh_immediate(strip);
    strip->refresh_immediate(strip1);
    strip->wait(strip, 50);
    strip->wait(strip1, 50);
}

static void fps_task(void *pvParameters) {
    while (true) {
        ESP_LOGI(TAG, "fps %i", framecount);
        framecount = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void setParams(uint8_t *data) {
    uint8_t num_params = data[0];
    for (uint8_t i = 0; i < num_params; i++) {
        uint8_t ptr = data[1 + i * 5];
        float f;
        memcpy(&f, data + 2 + i * 5, 4);
        ESP_LOGI(TAG, "set mem %i to %f", ptr, f);
        setFloat(mem, ptr, f);
    }
}

static void params_task(void *pvParameters) {
    uint8_t data[PROG_MEM_SIZE];
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
                case 0:
                    setShader(shader, data + 2, data[1]);
                    // reinitialize working memory
                    setMem(mem, shader);
                    break;
                case 1:
                    setParams(data + 1);
                    break;
            }
        }
    }
}

static void led_strip_task(void *pvParameters) {
    float clk = 0;
    float tick = 0.001;

    // uint16_t start_rgb = 0;
    uint32_t time = clock();

    // initialize strip segments
    led_strip_t *strip = stripCreateInit(DATA_PIN, RMT_TX_CHANNEL, NUM_LEDS);
    led_strip_t *strip1 = stripCreateInit(DATA_PIN1, RMT_TX_CHANNEL1, NUM_LEDS);

    setShader(shader, _scan2, sizeof(_scan2));

    // initialize working memory
    setMem(mem, shader);

    while (true) {
        // t and elapsed are in ms;
        uint32_t t = clock();
        uint32_t elapsed = t - time;
        time = t;
        // clock value in seconds
        clk += (float)elapsed * tick;

        frame(strip, strip1, shader, clk);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    main_events = xQueueCreate(4, PROG_MEM_SIZE);
    ESP_LOGI(TAG, "app main, start ledstrip task");

    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_sta();
    startWebserverTask(4096, 5, NULL, main_events);

    // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    xTaskCreate(fps_task, "fps", 4096, NULL, 8, NULL);
    xTaskCreate(params_task, "params", 4096, NULL, 8, NULL);
    xTaskCreatePinnedToCore(led_strip_task, "led_strip", 4096, NULL, 8, NULL, 1);
}
