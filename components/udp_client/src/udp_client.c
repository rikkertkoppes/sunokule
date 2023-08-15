#include "esp_log.h"
// #include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "storage.h"
#include "udp_client.h"

static const char *TAG = "udpclient";
static QueueHandle_t dmxQueue = NULL;
static int channel = 0;

int beginUDP(in_addr_t address, uint16_t port, const char *multicastIP) {
    uint8_t tx_buffer[1460];
    int udp_server;

    if ((udp_server = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        ESP_LOGE(TAG, "could not create socket: %d", errno);
        return -1;
    }

    int yes = 1;
    if (setsockopt(udp_server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
        ESP_LOGE(TAG, "could not set socket option: %d", errno);
        return -1;
    }

    struct sockaddr_in addr;
    memset((char *)&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = (in_addr_t)address;
    if (bind(udp_server, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        ESP_LOGE(TAG, "could not bind socket: %d", errno);
        return -1;
    }

    // const char *multicastIP = "239.255.255.250";

    // Join the multicast group
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicastIP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(udp_server, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ESP_LOGE(TAG, "Error in setting socket option IP_ADD_MEMBERSHIP, err = %d", errno);
        return -1;
    }

    fcntl(udp_server, F_SETFL, O_NONBLOCK);
    return udp_server;
}

int recvUDP(int sock) {
    // todo: universe from dip switches
    int myUniverse = 0;
    char rx_buffer[1460];
    // char data[512];
    struct sockaddr_in source_addr;  // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, rx_buffer, 1460, 0, (struct sockaddr *)&source_addr, &socklen);

    if (len < 0) {
        // ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        return 0;
    }
    // Data received
    else {
        // TODO: implement
        // byte 14, 15 => universe LE
        // byte 16, 17 => length
        // byte 18 => data
        int universe = (rx_buffer[15] << 8) | rx_buffer[14];
        int length = (rx_buffer[16] << 8) | rx_buffer[17];
        char *data = rx_buffer + 18;
        ESP_LOGI(TAG, "%i: (%i) %i %i %i %i %i", universe, length, data[0], data[1], data[2], data[3], data[4]);

        // dmx_command_t dmxCommand = {
        //     .universe = universe,
        //     .data = {0},
        // };
        // first is always 3 indicating dmx event
        data[0] = 3;
        if (data != NULL && universe == myUniverse) {
            // for (int i = 0; i < CHANNEL_COUNT; i++) {
            //     dmxCommand.data[i] = data[i + channel];
            // }
            // create data structure to set params
            // [
            //     1,
            //     num_params,
            // ]

            if (dmxQueue) {
                xQueueSend(dmxQueue, data, portMAX_DELAY);
            }
        }
    }

    return 1;
}

void udp_client_task(void *pvParameters) {
    char stored_multicast[16] = {0};
    esp_err_t multicast_err = read_multicast_ip(stored_multicast);

    ESP_LOGI(TAG, "starting udp client. Multicast address: %s", stored_multicast);

    while (1) {
        int sock = beginUDP(INADDR_ANY, 6454, stored_multicast);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket, retrying: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        while (1) {
            recvUDP(sock);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

void startUDPTask(int stackSize, int prio, EventGroupHandle_t state, QueueHandle_t queue) {
    dmxQueue = queue;
    ESP_LOGI(TAG, "Available heap: %lu", (unsigned long)esp_get_free_heap_size());
    xTaskCreate(udp_client_task, "udp_client", stackSize, (void *)state, prio, NULL);
}
