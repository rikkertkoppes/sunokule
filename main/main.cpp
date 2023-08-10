/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>

#include "FastLED.h"
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
#include "udp_client.h"
#include "webserver.h"
#include "wifi_conn.h"

#define RMT_TX_CHANNEL0 RMT_CHANNEL_0
#define RMT_TX_CHANNEL1 RMT_CHANNEL_1
#define RMT_TX_CHANNEL2 RMT_CHANNEL_2
#define DATA_PIN0 static_cast<gpio_num_t>(CONFIG_SK_DATA_PIN_0)
#define DATA_PIN1 static_cast<gpio_num_t>(CONFIG_SK_DATA_PIN_1)
#define DATA_PIN2 static_cast<gpio_num_t>(CONFIG_SK_DATA_PIN_2)
#define NUM_LEDS0 CONFIG_SK_NUM_LEDS_0
#define NUM_LEDS1 CONFIG_SK_NUM_LEDS_1
#define NUM_LEDS2 CONFIG_SK_NUM_LEDS_2

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

/**
 * expecting rgb as a float in [0,1]
 */
void fastled_setPixelRGB(CRGB *leds, u_int32_t index, float r, float g, float b) {
    r = clamp(r, 0, 1);
    g = clamp(g, 0, 1);
    b = clamp(b, 0, 1);

    // ESP_LOGI(TAG, "rgb %f %f %f", r * 255, g * 255, b * 255);

    leds[index] = CRGB(255 * r, 255 * g, 255 * b);
}

static byte mem[SHADER_MEM_SIZE];
static byte shader[PROG_MEM_SIZE];

int framecount = 0;

// frame renderer, by using a shader and incoming data
void frame(CRGB *leds_strip0, CRGB *leds_strip1, CRGB *leds_strip2, byte *shader, float clk) {
    for (int j = 0; j < NUM_LEDS0 + NUM_LEDS1 + NUM_LEDS2; j += 1) {
        byte version = shader[0];
        byte id = shader[1];

        uint16_t memStart = getUint16(shader, 2);
        uint16_t memSize = getUint16(shader, 4);
        uint16_t progStart = getUint16(shader, 6);
        uint16_t progSize = getUint16(shader, 8);

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
        if (j < NUM_LEDS0) {
            fastled_setPixelRGB(leds_strip0, j, r, g, b);
        } else if (j < (NUM_LEDS0 + NUM_LEDS1)) {
            fastled_setPixelRGB(leds_strip1, j - NUM_LEDS0, r, g, b);
        } else {
            fastled_setPixelRGB(leds_strip2, j - NUM_LEDS0 - NUM_LEDS1, r, g, b);
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
    FastLED.show();
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
                // shader packet
                case 0:
                    // store shader in NVS
                    saveShader(data + 2, data[1]);
                    // reset shader time
                    clk = 0;
                    setShader(shader, data + 2, data[1]);
                    // reinitialize working memory
                    setMem(mem, shader);
                    break;
                // params packet
                case 1:
                    setParams(data + 1);
                    break;
                // power on/ off packet
                case 2:
                    // reset shader time
                    clk = 0;
                    power = data[1];
                    savePowerState(power);
                    break;
                // dmx packet
                case 3:
                    // put dmx data into memory
                    setDMX(mem, data);
                    printf("%i %i %i \n", getDMX(mem, 0), getDMX(mem, 1), getDMX(mem, 2));
                    break;
            }
        }
    }
}

static void led_strip_task(void *pvParameters) {
    float tick = 0.001;

    // uint16_t start_rgb = 0;
    uint32_t time = clock();

    ESP_LOGI(TAG, "configure %i leds on pin %i", NUM_LEDS0, DATA_PIN0);
    ESP_LOGI(TAG, "configure %i leds on pin %i", NUM_LEDS1, DATA_PIN1);
    ESP_LOGI(TAG, "configure %i leds on pin %i", NUM_LEDS2, DATA_PIN2);

    // initialize strip segments
    CRGB leds_strip0[NUM_LEDS0];
    CRGB leds_strip1[NUM_LEDS1];
    CRGB leds_strip2[NUM_LEDS2];

    // fast led setup
    FastLED.addLeds<WS2812, DATA_PIN0, GRB>(leds_strip0, NUM_LEDS0);  // Adjust <WS2812B> according to your LED strip type
    FastLED.addLeds<WS2812, DATA_PIN1, GRB>(leds_strip1, NUM_LEDS1);
    FastLED.addLeds<WS2812, DATA_PIN2, GRB>(leds_strip2, NUM_LEDS2);

    FastLED.setCorrection(TypicalLEDStrip);

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
            frame(leds_strip0, leds_strip1, leds_strip2, shader, clk);
        } else {
            FastLED.clear(true);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void app_main(void) {
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
