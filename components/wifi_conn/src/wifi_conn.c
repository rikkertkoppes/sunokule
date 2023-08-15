#include "wifi_conn.h"

#include <esp_wifi.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_bit_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "storage.h"
#include "string.h"

static const char *TAG = "wifi_conn";

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID CONFIG_SK_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_SK_WIFI_PASS
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

esp_netif_t *sta_netif;
static bool sta_netif_created = false;

bool wifi_init_sta(char *stored_ssid, char *stored_pass, char *stored_static_ip, char *stored_gateway) {
    // TODO: this crashes and restarts the esp, but should just restart the wifi
    sta_netif = esp_netif_create_default_wifi_sta();

    // if (!sta_netif_created) {
    //     sta_netif = esp_netif_create_default_wifi_sta();
    //     sta_netif_created = true;
    // }

    // Check if a static IP address is provided
    if (strlen(stored_static_ip) > 0 && strlen(stored_gateway) > 0) {
        ESP_LOGI(TAG, "init wifi from stored ip %s gatweay %s.", stored_static_ip, stored_gateway);

        esp_netif_dhcpc_stop(sta_netif);

        // Set the static IP address
        esp_netif_ip_info_t ip_info = {
            .ip = {.addr = esp_ip4addr_aton(stored_static_ip) },
            .gw = {.addr = esp_ip4addr_aton(stored_gateway) },
            .netmask = {.addr = ESP_IP4TOADDR(255, 255, 255, 0) }
        };
        esp_netif_set_ip_info(sta_netif, &ip_info);
    } else {
        esp_netif_dhcpc_start(sta_netif);
    }

    // Connect to the specified Wi-Fi network (Station mode)
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT,
                                            ESP_EVENT_ANY_ID,
                                            &event_handler,
                                            NULL,
                                            &instance_any_id));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT,
                                            IP_EVENT_STA_GOT_IP,
                                            &event_handler,
                                            NULL,
                                            &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    // Set stored_ssid and stored_pass
    strncpy((char *)wifi_config.sta.ssid, stored_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, stored_pass, sizeof(wifi_config.sta.password));

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

    bool sta_result = false;

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 stored_ssid, stored_pass);
        sta_result = true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 stored_ssid, stored_pass);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    return sta_result;
}

void wifi_init_ap(void) {
    // Start in access point mode
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Set a predefined IP address, netmask, and gateway for the access point
    esp_netif_ip_info_t ip_info = {
        .ip = {.addr = ESP_IP4TOADDR(192, 168, 4, 1) },
        .gw = {.addr = ESP_IP4TOADDR(192, 168, 4, 1) },
        .netmask = {.addr = ESP_IP4TOADDR(255, 255, 255, 0) }
    };
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Suno Kule Device",
            .ssid_len = strlen("Suno Kule Device"),
            .channel = 1,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "ESP32 started in access point mode");
    // Print the access point's IP address
    char ip_address_str[16];
    esp_ip4addr_ntoa(&ip_info.ip, ip_address_str, sizeof(ip_address_str));
    ESP_LOGI(TAG, "Access Point IP address: %s", ip_address_str);
}

void wifi_stop() {
    // Stop Wi-Fi in AP mode
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_event_loop_delete_default());
}

void wifi_init(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    char stored_ssid[33] = {0};
    char stored_pass[65] = {0};
    char stored_static_ip[16] = {0};
    char stored_gateway[16] = {0};

    esp_err_t cred_err = read_wifi_credentials(stored_ssid, stored_pass);
    esp_err_t static_err = read_static_ip(stored_static_ip, stored_gateway);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    bool start_ap = true;

    // Check if SSID and password are provided
    if (cred_err == ESP_OK) {
        start_ap = false;
        bool sta_connected = wifi_init_sta(stored_ssid, stored_pass, stored_static_ip, stored_gateway);
        if (!sta_connected) {
            start_ap = true;
        }
    }

    // check whether ap mode needs to be started
    if (start_ap) {
        wifi_init_ap();
    }
}

void wifi_restart(void) {
    wifi_stop();
    wifi_init();
}
