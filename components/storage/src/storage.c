#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_conn";

void init_flash(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void store_wifi_credentials(const char *ssid, const char *password) {
    esp_err_t err;

    // Open NVS handle
    nvs_handle_t nvs_handle;
    err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }

    // Store SSID
    err = nvs_set_str(nvs_handle, "ssid", ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to store SSID: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "SSID stored successfully");
    }

    // Store password
    err = nvs_set_str(nvs_handle, "password", password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to store password: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Password stored successfully");
    }

    // Commit the changes to NVS
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit changes to NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Changes committed to NVS");
    }

    // Close the NVS handle
    nvs_close(nvs_handle);
}

esp_err_t read_wifi_credentials(char *stored_ssid, char *stored_pass) {
    if (stored_ssid == NULL || stored_pass == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t ssid_size = 33;
    size_t pass_size = 65;

    err = nvs_get_str(nvs_handle, "ssid", stored_ssid, &ssid_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_get_str(nvs_handle, "password", stored_pass, &pass_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

void store_static_ip(const char *static_ip, const char *gateway) {
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("wifi_config", NVS_READWRITE, &nvs_handle));
    nvs_set_str(nvs_handle, "static_ip", static_ip);
    nvs_set_str(nvs_handle, "gateway", gateway);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

esp_err_t read_static_ip(char *stored_static_ip, char *stored_gateway) {
    if (stored_static_ip == NULL || stored_gateway == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t static_ip_size = 16;
    size_t gateway_size = 16;

    err = nvs_get_str(nvs_handle, "static_ip", stored_static_ip, &static_ip_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_get_str(nvs_handle, "gateway", stored_gateway, &gateway_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}
