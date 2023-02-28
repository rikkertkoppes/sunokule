#include "nvs_flash.h"

#define STORAGE_NAMESPACE "storage"

typedef unsigned char byte;

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

esp_err_t saveShader(byte* prog, size_t size) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write shader
    err = nvs_set_blob(my_handle, "shader", prog, size);

    if (err != ESP_OK) {
        nvs_close(my_handle);
        return err;
    }

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return err;
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t readShader(byte* data) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read the size of memory space required for blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "shader", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(my_handle);
        return err;
    }

    if (required_size > 0) {
        // read if available
        err = nvs_get_blob(my_handle, "shader", data, &required_size);
        if (err != ESP_OK) {
            nvs_close(my_handle);
            return err;
        }
    } else {
        // TODO: return default
        memcpy(data, _scan2, sizeof(_scan2));
    }
    return ESP_OK;
}
