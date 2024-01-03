#include "nvs_flash.h"

#define STORAGE_NAMESPACE "storage"

typedef unsigned char byte;

byte _scan2[] = {
    1,  // version
    3,  // shader id

    12, 0,  // mem offset: 12
    72, 0,  // mem size: 72
    84, 0,  // prog offset 84
    51, 0,  // prog size 51
    0, 0,   // control offset: 0

    // memory
    154, 153, 25, 63,  // 0: 0.6
    51, 51, 51, 63,    // 1: 0.7
    0, 0, 0, 0,        // 2: 0
    0, 0, 0, 192,      // 3: -2
    0, 0, 0, 0,        // 4: 0
    0, 0, 0, 0,        // 5: 0
    0, 0, 128, 63,     // 6: 1
    0, 0, 128, 64,     // 7: 4
    0, 0, 64, 64,      // 8: 3
    236, 81, 56, 62,   // 9: 0.18
    236, 81, 56, 62,   // 10: 0.18
    0, 0, 0, 0,        // 11: 0
    0, 0, 0, 0,        // 12: 0
    0, 0, 0, 64,       // 13: 2
    0, 0, 0, 0,        // 14: 0
    0, 0, 0, 0,        // 15: 0
    0, 0, 128, 63,     // 16: 1
    0, 0, 128, 63,     // 17: 1

    // program
    6, 11, 23, 12, 34,                          // 0: n930777
    5, 9, 32,                                   // 5: n622556
    6, 32, 7, 8, 31,                            // 8: n910223
    5, 10, 33,                                  // 13: n633476
    3, 13, 33, 14, 31, 31, 15, 16, 34, 18, 35,  // 16: n340570
    3, 3, 33, 4, 31, 5, 31, 6, 34, 18, 30,      // 27: n2397
    6, 35, 30, 2, 29,                           // 38: n902092
    4, 0, 1, 29, 26,                            // 43: n408274
    0, 17, 26,                                  // 48: end program
};

esp_err_t savePowerState(byte state) {
    nvs_handle_t my_handle;
    esp_err_t err = ESP_OK;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write shader
    err = nvs_set_u8(my_handle, "power", state);

    if (err != ESP_OK) {
        nvs_close(my_handle);
        return err;
    }

    // Commit
    err = nvs_commit(my_handle);

    // Close
    nvs_close(my_handle);
    return err;
}
byte readPowerState() {
    nvs_handle_t my_handle;
    byte state = 0;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return state;

    // read power state
    err = nvs_get_u8(my_handle, "power", &state);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(my_handle);
        return 0;
    }

    return state;
}

esp_err_t saveShader(byte* prog, size_t size) {
    nvs_handle_t my_handle;
    esp_err_t err = ESP_OK;

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

    // Close
    nvs_close(my_handle);
    return err;
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
