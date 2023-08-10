#ifdef __cplusplus
extern "C" {
#endif

#ifndef STORAGE_H
#define STORAGE_H

void init_flash(void);
void store_wifi_credentials(const char *ssid, const char *password);
esp_err_t read_wifi_credentials(char *stored_ssid, char *stored_pass);
void store_static_ip(const char *static_ip, const char *gateway);
esp_err_t read_static_ip(char *stored_static_ip, char *stored_gateway);
void store_multicast_ip(const char *multicast_ip);
esp_err_t read_multicast_ip(char *stored_multicast_ip);

#endif

#ifdef __cplusplus
}
#endif
