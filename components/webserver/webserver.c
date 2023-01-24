#include <esp_http_server.h>

#include "esp_log.h"
#include "tcpip_adapter.h"

static const char *TAG = "webserver";

httpd_handle_t server = NULL;

static esp_err_t index_handler(httpd_req_t *req) {
    // httpd_resp_set_type(req, "text/html");
    // httpd_resp_send(req, "<html><body><h1>Hello World!</h1></body></html>", -1);

    tcpip_adapter_ip_info_t ip_info;
    ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));

    char url[80];
    sprintf(url, "http://localhost:3078/?%s", ip4addr_ntoa(&ip_info.ip));

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", url);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.core_id = 0;
    config.task_priority = 4;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL};

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
}
