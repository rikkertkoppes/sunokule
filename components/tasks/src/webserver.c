
#include <esp_https_server.h>

#include "esp_log.h"
#include "freertos/event_groups.h"
#include "storage.h"
#include "tcpip_adapter.h"
#include "wifi_conn.h"

#define MAX_PAYLOAD_LEN 256
#define STATE_GOT_PARAMS 1

static const char *TAG = "webserver";

httpd_handle_t http_server = NULL;
httpd_handle_t https_server = NULL;
EventGroupHandle_t webserverState;
static QueueHandle_t eventQueue = NULL;

// self signed certs
extern const uint8_t cacert_pem_start[] asm("_binary_cacert_pem_start");
extern const uint8_t cacert_pem_end[] asm("_binary_cacert_pem_end");
extern const uint8_t prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
extern const uint8_t prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");

const char *wifi_setup_html =
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "  <title>Wi-Fi Setup</title>"
    "</head>"
    "<body>"
    "  <h1>Suno Kule Setup</h1>"
    "  <form action=\"/setup\" method=\"POST\">"
    "    <label for=\"ssid\">SSID:</label>"
    "    <input type=\"text\" id=\"ssid\" name=\"ssid\" value=\"%s\" required><br><br>"
    "    <label for=\"password\">Password:</label>"
    "    <input type=\"password\" id=\"password\" name=\"password\" value=\"%s\" required><br><br>"
    "    <label for=\"static_ip\">Static IP:</label>"
    "    <input type=\"text\" id=\"static_ip\" name=\"static_ip\" value=\"%s\"><br><br>"
    "    <label for=\"gateway\">Gateway:</label>"
    "    <input type=\"text\" id=\"gateway\" name=\"gateway\" value=\"%s\"><br><br>"
    "    <input type=\"submit\" value=\"Submit\">"
    "  </form>"
    "</body>"
    "</html>";

esp_err_t wifi_setup_handler(httpd_req_t *req) {
    char stored_ssid[33] = {0};
    char stored_pass[65] = {0};
    char stored_static_ip[16] = {0};
    char stored_gateway[16] = {0};
    esp_err_t cred_err = read_wifi_credentials(stored_ssid, stored_pass);
    esp_err_t static_err = read_static_ip(stored_static_ip, stored_gateway);

    // Buffer to hold the dynamically generated HTML content
    char html_buf[1024];

    snprintf(html_buf, sizeof(html_buf), wifi_setup_html, stored_ssid, stored_pass, stored_static_ip, stored_gateway);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_buf, strlen(html_buf));
    // httpd_resp_send(req, wifi_setup_html, strlen(wifi_setup_html));
    return ESP_OK;
}

esp_err_t wifi_setup_submit_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        // Buffer to store incoming POST data
        char buf[100];
        int received = httpd_req_recv(req, buf, sizeof(buf));
        if (received <= 0) {  // 0 return value indicates connection closed
            // Check if timeout occurred
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            }
            return ESP_FAIL;
        }
        buf[received] = '\0';

        // Parse SSID and password from the POST data
        char ssid[33] = {0};
        char password[65] = {0};
        char static_ip[16] = {0};
        char gateway[16] = {0};
        sscanf(buf, "ssid=%[^&]&password=%[^&]&static_ip=%[^&]&gateway=%15s", ssid, password, static_ip, gateway);

        ESP_LOGI(TAG, "ssid %s", ssid);
        ESP_LOGI(TAG, "pass %s", password);
        ESP_LOGI(TAG, "static_ip %s", static_ip);
        ESP_LOGI(TAG, "gateway %s", gateway);

        // Store SSID and password in NVS if they are not empty
        if (strlen(ssid) > 0 && strlen(password) > 0) {
            store_wifi_credentials(ssid, password);
        }

        // Store static IP in NVS
        store_static_ip(static_ip, gateway);

        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_send(req, NULL, 0);
        // Send a success response
        // httpd_resp_sendstr(req, "SSID and password have been saved.");

        // restart wifi:
        wifi_restart();
    } else {
        // Send a 405 Method Not Allowed response if the request method is not POST
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method not allowed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// index handler
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "<html><body><h1>Hello World!</h1><script>let sock=new WebSocket(`ws://${location.hostname}/ws`);</script></body></html>", -1);

    // tcpip_adapter_ip_info_t ip_info;
    // ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));

    // char url[80];
    // sprintf(url, "http://localhost:3078/?%s", ip4addr_ntoa(&ip_info.ip));

    // httpd_resp_set_status(req, "303 See Other");
    // httpd_resp_set_hdr(req, "Location", url);
    // httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t ws_broadcast(char *str) {
    static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
    size_t fds = max_clients;
    int client_fds[max_clients];

    esp_err_t ret = httpd_get_client_list(https_server, &fds, client_fds);

    if (ret != ESP_OK) {
        return ret;
    }

    for (int i = 0; i < fds; i++) {
        int client_info = httpd_ws_get_fd_info(https_server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_frame_t packet;
            packet.type = HTTPD_WS_TYPE_TEXT;
            packet.payload = (uint8_t *)str;
            packet.len = strlen(str);
            httpd_ws_send_frame_async(https_server, client_fds[i], &packet);
        }
    }

    return ESP_OK;
}

esp_err_t ws_send_string(httpd_req_t *req, char *str) {
    httpd_ws_frame_t packet;
    packet.type = HTTPD_WS_TYPE_TEXT;
    packet.payload = (uint8_t *)str;
    packet.len = strlen(str);
    esp_err_t ret = httpd_ws_send_frame(req, &packet);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    }
    return ret;
}

esp_err_t ws_read_length(httpd_req_t *req, httpd_ws_frame_t *packet) {
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, packet, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    // ESP_LOGI(TAG, "frame length is %d", packet.len);
    return ret;
}

esp_err_t ws_read_string(httpd_req_t *req, uint8_t *buf, size_t max_len) {
    esp_err_t ret = ESP_OK;

    // packet
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // get length
    ret = ws_read_length(req, &ws_pkt);
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

    if (ws_pkt.len + 1 > max_len) {
        ESP_LOGE(TAG, "Not enough memory to store payload, need %d", ws_pkt.len);
        return ESP_ERR_NO_MEM;
    }

    if (ws_pkt.len) {
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            return ret;
        }
    }
    ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
    return ret;
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // handle handshake
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    esp_err_t ret = ESP_OK;

    // read into buffer
    uint8_t buf[MAX_PAYLOAD_LEN];
    memset(buf, 0, sizeof(buf));
    ret = ws_read_string(req, buf, sizeof(buf));
    // ESP_LOGI(TAG, "Got packet with message: %s", buf);
    // xEventGroupSetBits(webserverState, STATE_GOT_PARAMS);

    if (eventQueue) {
        xQueueSend(eventQueue, buf, portMAX_DELAY);
    }

    uint8_t datatype = buf[0];

    // send back
    switch (datatype) {
        case 0:
            ws_send_string(req, "{\"message\":\"got shader\"}");
            break;
        case 1:
            ws_send_string(req, "{\"message\":\"got params\"}");
            break;
    }
    // ret = ws_send_string(req, "blalala");

    return ret;
}

static void register_uri_handlers(httpd_method_t method, const char *uri_path, esp_err_t (*handler)(httpd_req_t *)) {
    // Register handler
    httpd_uri_t uri_handler = {
        .uri = uri_path,
        .method = method,
        .handler = handler,
        .user_ctx = NULL};

    // Register URI handler for the HTTP server
    esp_err_t ret = httpd_register_uri_handler(http_server, &uri_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register URI handler for HTTP server: %s", esp_err_to_name(ret));
    }

    // Register the same URI handler for the HTTPS server
    ret = httpd_register_uri_handler(https_server, &uri_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register URI handler for HTTPS server: %s", esp_err_to_name(ret));
    }
}

static void http_get(const char *uri_path, esp_err_t (*handler)(httpd_req_t *)) {
    return register_uri_handlers(HTTP_GET, uri_path, handler);
}
static void http_post(const char *uri_path, esp_err_t (*handler)(httpd_req_t *)) {
    return register_uri_handlers(HTTP_POST, uri_path, handler);
}

void start_webserver(void) {
    esp_err_t ret;

    // http config
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.uri_match_fn = httpd_uri_match_wildcard;
    http_config.core_id = 0;
    http_config.task_priority = 4;
    http_config.ctrl_port = 32768;

    // Start HTTP server
    ret = httpd_start(&http_server, &http_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Http webserver running");

    // https config
    httpd_ssl_config_t https_config = HTTPD_SSL_CONFIG_DEFAULT();
    https_config.httpd.uri_match_fn = httpd_uri_match_wildcard;

    // add certificates
    https_config.cacert_pem = cacert_pem_start;
    https_config.cacert_len = cacert_pem_end - cacert_pem_start;
    https_config.prvtkey_pem = prvtkey_pem_start;
    https_config.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    https_config.httpd.core_id = 0;
    https_config.httpd.task_priority = 4;
    https_config.httpd.ctrl_port = 32769;

    // Start HTTPS server
    ret = httpd_ssl_start(&https_server, &https_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTPS server: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Https webserver running");

    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = true,
    };

    // register_uri_handlers("/", index_handler);
    http_get("/", wifi_setup_handler);
    http_post("/setup", wifi_setup_submit_handler);

    ESP_ERROR_CHECK(httpd_register_uri_handler(https_server, &ws_uri));
}

// void webserverTask(void *arg) {
//     webserverState = xEventGroupCreate();

//     start_webserver();
//     ESP_LOGI(TAG, "Webserver running");

//     EventBits_t stateBits;
//     for (;;) {
//         // TODO: wait for webserver event group bits from handlers to forward stuff to state
//         stateBits = xEventGroupWaitBits(webserverState, STATE_GOT_PARAMS, pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));
//         if (stateBits & STATE_GOT_PARAMS) {
//             ESP_LOGI(TAG, "params received");
//             //     setNeedReboot(true);
//             //     vTaskDelay(5000 / portTICK_RATE_MS);
//             //     MDF_LOGI("rebooting now");
//             //     esp_restart();
//             xEventGroupClearBits(webserverState, STATE_GOT_PARAMS);
//         }
//         vTaskDelay(5000 / portTICK_RATE_MS);
//     }

//     // TODO: from oko main setuptask
//     // stop_webserver(server);
//     // stop_dns_server(dnsserver);

//     vTaskDelete(NULL);
// }

void startWebserverTask(int stackSize, int prio, EventGroupHandle_t state, QueueHandle_t queue) {
    eventQueue = queue;
    ESP_LOGI(TAG, "Available heap: %u", esp_get_free_heap_size());
    start_webserver();
    ESP_LOGI(TAG, "Webserver running");
    // xTaskCreate(webserverTask, "webserver_task", stackSize, (void *)state, prio, NULL);
}
