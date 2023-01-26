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

static const char *TAG = "example";

typedef unsigned char byte;

/**
 * expecting rgb as a float in [0,1]
 */
void led_strip_setPixelRGB(led_strip_t *strip, u_int32_t index, float r, float g, float b) {
    strip->set_pixel(strip, index, 255 * r, 255 * g, 255 * b);
}

static byte mem[SHADER_MEM_SIZE];
static byte prog[PROG_MEM_SIZE];

void execute(byte *mem, byte *prog, byte *counter) {
    byte op = instruction_fetch(mem, prog, counter);
    while (op) {
        switch (op) {
            case 0:
                // end of program
                return;
            case 1:
                color(mem, prog, counter);
                break;
            case 2:
                gradient(mem, prog, counter);
                break;
            case 3:
                waveform(mem, prog, counter);
                break;
            case 4:
                hsv2rgb(mem, prog, counter);
                break;
            case 5:
                value(mem, prog, counter);
                break;
            case 6:
                math(mem, prog, counter);
                break;
            case 7:
                trig(mem, prog, counter);
                break;
        }
        op = instruction_fetch(mem, prog, counter);
    }
}

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
    15,

    0, 0, 224, 64,
    0, 0, 0, 192,
    0, 0, 0, 0,
    0, 0, 224, 64,
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
    0, 0, 0, 0,

    7, 17, 0, 23,
    6, 23, 1, 2, 24,
    7, 17, 3, 25,
    6, 24, 4, 5, 26,
    6, 15, 25, 6, 27,
    3, 26, 7, 8, 9, 10, 11, 12, 18, 27, 28,
    4, 13, 14, 28, 29,
    0};

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

const uint8_t NUM_EFFECTS = 8;

static void fps_task(void *pvParameters) {
    while (true) {
        ESP_LOGI(TAG, "fps %i", framecount);
        framecount = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void led_strip_task(void *pvParameters) {
    float clk = 0;
    float tick = 0.001;

    // uint16_t start_rgb = 0;
    uint32_t time = clock();

    // strip 0
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(DATA_PIN, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(NUM_LEDS, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    // strip 0
    rmt_config_t config1 = RMT_DEFAULT_CONFIG_TX(DATA_PIN1, RMT_TX_CHANNEL1);
    // set counter clock to 40MHz
    config1.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config1));
    ESP_ERROR_CHECK(rmt_driver_install(config1.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config1 = LED_STRIP_DEFAULT_CONFIG(NUM_LEDS, (led_strip_dev_t)config1.channel);
    led_strip_t *strip1 = led_strip_new_rmt_ws2812(&strip_config1);
    if (!strip1) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip1->clear(strip1, 100));

    byte *shader = _scan2;

    // initialize working memory
    setMem(mem, shader);

    while (true) {
        uint32_t t = clock();
        uint32_t elapsed = t - time;
        time = t;
        clk += (float)elapsed * tick;

        frame(strip, strip1, shader, clk);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

uint8_t step = 256 / NUM_EFFECTS;
uint8_t effect(uint8_t index) {
    return step / 2 + step * index;
}

void app_main(void) {
    ESP_LOGI(TAG, "app main, start ledstrip task");

    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_sta();
    start_webserver();

    // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    xTaskCreate(fps_task, "fps", 4096, NULL, 8, NULL);
    xTaskCreatePinnedToCore(led_strip_task, "led_strip", 4096, NULL, 8, NULL, 1);
}
