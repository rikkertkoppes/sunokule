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
#include "ledstorage.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mapping.h"
#include "math.h"
#include "nvs_flash.h"
#include "primitives.h"
#include "protocol_examples_common.h"
#include "storage.h"
#include "webserver.h"
#include "wifi_conn.h"

#define RMT_TX_CHANNEL0 RMT_CHANNEL_0
#define RMT_TX_CHANNEL1 RMT_CHANNEL_1
#define RMT_TX_CHANNEL2 RMT_CHANNEL_2
#define DATA_PIN0 CONFIG_SK_DATA_PIN_0
#define DATA_PIN1 CONFIG_SK_DATA_PIN_1
#define DATA_PIN2 CONFIG_SK_DATA_PIN_2
#define NUM_LEDS0 CONFIG_SK_NUM_LEDS_0
#define NUM_LEDS1 CONFIG_SK_NUM_LEDS_1
#define NUM_LEDS2 CONFIG_SK_NUM_LEDS_2

// update mapping => colors off, strip 0, ok, others wrong
// swap data pins => no change
// swap refresh order (1,0,2) => no change
// wait after every write => all ok
// adjust timing => all ok

#define SHADER_MEM_SIZE 256
#define PROG_MEM_SIZE 256

static const char *TAG = "shader";
QueueHandle_t main_events;

typedef unsigned char byte;
float clk = 0;
byte power = 0;  // power off

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
    // remap r,g,b values to suppress low values
    // r = r * r * r;
    // g = g * g * g;
    // b = b * b * b;

    // ESP_LOGI(TAG, "rgb %f %f %f", r * 255, g * 255, b * 255);

    strip->set_pixel(strip, index, 255 * r, 255 * g, 255 * b);
}

static byte mem[SHADER_MEM_SIZE];
static byte shader[PROG_MEM_SIZE];

int framecount = 0;

// frame renderer, by using a shader and incoming data
void frame(led_strip_t *strip0, led_strip_t *strip1, led_strip_t *strip2, byte *shader, float clk) {
    for (int j = 0; j < NUM_LEDS0 + NUM_LEDS1 + NUM_LEDS2; j += 1) {
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
        byte _master = data_fetch(mem, prog, &counter);
        byte _result = data_fetch(mem, prog, &counter);

        // retrieve rgb values
        float m = getFloat(mem, _master);
        float r = m * getFloat(mem, _result);
        float g = m * getFloat(mem, _result + 1);
        float b = m * getFloat(mem, _result + 2);

        // ESP_LOGI(TAG, "counter %i, rgb %f %f %f", counter, r, g, b);
        if (j < NUM_LEDS0) {
            led_strip_setPixelRGB(strip0, j, r, g, b);
        } else if (j < (NUM_LEDS0 + NUM_LEDS1)) {
            led_strip_setPixelRGB(strip1, j - NUM_LEDS0, r, g, b);
        } else {
            led_strip_setPixelRGB(strip2, j - NUM_LEDS0 - NUM_LEDS1, r, g, b);
        }
    }
    framecount++;
    /**
     * TODO: we can do even better than this
     * - allocate 2 buffers for the entire strip
     * - directly use rmt_write_sample and pass the correct memory segment
     * - calculate next frame while hardware is writing to rmt
     * - rather than calculate => refresh => wait, do:
     * -             refresh => calculate next => wait
     *
     */
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

    if (NUM_LEDS0 > 0) {
        strip0->refresh_immediate(strip0);
    }
    if (NUM_LEDS1 > 0) {
        strip1->refresh_immediate(strip1);
    }
    if (NUM_LEDS2 > 0) {
        strip2->refresh_immediate(strip2);
    }

    if (NUM_LEDS0 > 0) {
        strip0->wait(strip0, 50);
    }
    if (NUM_LEDS1 > 0) {
        strip1->wait(strip1, 50);
    }
    if (NUM_LEDS2 > 0) {
        strip2->wait(strip2, 50);
    }
}

static void fps_task(void *pvParameters) {
    while (true) {
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
                    // store shader in NVS
                    saveShader(data + 2, data[1]);
                    // reset shader time
                    clk = 0;
                    setShader(shader, data + 2, data[1]);
                    // reinitialize working memory
                    setMem(mem, shader);
                    break;
                case 1:
                    setParams(data + 1);
                    break;
                case 2:
                    // reset shader time
                    clk = 0;
                    power = data[1];
                    savePowerState(power);
            }
        }
    }
}

static void led_strip_task(void *pvParameters) {
    float tick = 0.001;

    // uint16_t start_rgb = 0;
    uint32_t time = clock();

    // initialize strip segments
    led_strip_t *strip0 = stripCreateInit(DATA_PIN0, RMT_TX_CHANNEL0, NUM_LEDS0);
    led_strip_t *strip1 = stripCreateInit(DATA_PIN1, RMT_TX_CHANNEL1, NUM_LEDS1);
    led_strip_t *strip2 = stripCreateInit(DATA_PIN2, RMT_TX_CHANNEL2, NUM_LEDS2);

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

        if (power) {
            frame(strip0, strip1, strip2, shader, clk);
        } else {
            strip0->clear(strip0, 50);
            strip1->clear(strip1, 50);
            strip2->clear(strip2, 50);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    main_events = xQueueCreate(4, PROG_MEM_SIZE);
    ESP_LOGI(TAG, "app main, start ledstrip task");

    init_flash();

    wifi_init();
    startWebserverTask(4096, 5, NULL, main_events);

    // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    xTaskCreate(fps_task, "fps", 4096, NULL, 8, NULL);
    xTaskCreate(params_task, "params", 4096, NULL, 8, NULL);
    xTaskCreatePinnedToCore(led_strip_task, "led_strip", 2 * 4096, NULL, 8, NULL, 1);
}
