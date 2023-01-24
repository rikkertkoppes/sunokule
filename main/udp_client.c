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
#include "math.h"
#include "nvs_flash.h"
#include "primitives.h"
#include "protocol_examples_common.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define DATA_PIN 26
#define NUM_LEDS 600
#define EXAMPLE_CHASE_SPEED_MS 50
#define SHADER_MEM_SIZE 256
#define PROG_MEM_SIZE 256

static const char *TAG = "example";

typedef unsigned char byte;

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID CONFIG_EXAMPLE_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_EXAMPLE_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY 2

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

typedef struct {
    float r;
    float g;
    float b;
} rgb;

/**
 * expecting rgb as a float in [0,1]
 */
void led_strip_setPixelRGB(led_strip_t *strip, u_int32_t index, float r, float g, float b) {
    strip->set_pixel(strip, index, 255 * r, 255 * g, 255 * b);
}

// from [0,1]
float ledX(uint32_t index) {
    return (float)index / (NUM_LEDS - 1);
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

int framecount = 0;

// frame renderer, by using a shader and incoming data
void frame(led_strip_t *strip, byte *shader, float clk) {
    for (int j = 0; j < NUM_LEDS; j += 1) {
        size_t mem_end = shader[0];
        float x = ledX(j);
        // set params
        setFloat(mem, mem_end, clk);
        setFloat(mem, mem_end + 2, x);
        setFloat(mem, mem_end + 3, x);  // TODO: should be y
        setFloat(mem, mem_end + 4, x);  // TODO: should be z
        setFloat(mem, mem_end + 5, x);  // TODO: should be u
        setFloat(mem, mem_end + 6, x);  // TODO: should be v
        setFloat(mem, mem_end + 7, x);  // TODO: should be w

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
        led_strip_setPixelRGB(strip, j, r, g, b);
    }
    framecount++;
    strip->refresh(strip, 50);
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
    // Show simple rainbow chasing pattern

    byte *shader = _scan2;

    // initialize working memory
    setMem(mem, shader);

    while (true) {
        uint32_t t = clock();
        uint32_t elapsed = t - time;
        time = t;
        clk += (float)elapsed * tick;

        frame(strip, shader, clk);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

uint8_t step = 256 / NUM_EFFECTS;
uint8_t effect(uint8_t index) {
    return step / 2 + step * index;
}

void app_main(void) {
    ESP_LOGI(TAG, "app main, start ledstrip task");
    // initDIP();
    // channel = CHANNEL_COUNT * readDIP();
    // ESP_LOGI(TAG, "listening on DMX channel %i", channel+1);
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // ESP_ERROR_CHECK(example_connect());
    // wifi_init_sta();

    xTaskCreate(fps_task, "fps", 4096, NULL, 8, NULL);
    xTaskCreatePinnedToCore(led_strip_task, "led_strip", 4096, NULL, 8, NULL, 1);

    // buttons
}
