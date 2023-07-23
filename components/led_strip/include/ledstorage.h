#include "nvs_flash.h"

#define STORAGE_NAMESPACE "storage"

typedef unsigned char byte;

byte _scan2[] = {
    1,  // version
    1,  // id

    12, 0,  // mem offset
    76, 0,  // mem size: 19*4 = 76
    88, 0,  // prog offset
    52, 0,  // prog size
    0, 0,   // control offset

    205, 204, 204, 61,  // 0: 0.1
    0, 0, 128, 63,      // 1: 1
    0, 0, 160, 64,      // 2: 5
    0, 0, 0, 192,       // 3: -2
    0, 0, 0, 0,         // 4: 0
    0, 0, 0, 0,         // 5: 0
    0, 0, 128, 63,      // 6: 1
    0, 0, 0, 0,         // 7: 0
    0, 0, 0, 0,         // 8: 0
    0, 0, 128, 64,      // 9: 4
    0, 0, 64, 64,       // 10: 3
    205, 204, 204, 61,  // 11: 0.1
    0, 0, 128, 63,      // 12: true
    205, 204, 204, 62,  // 13: 0.4
    0, 0, 128, 63,      // 14: true
    0, 0, 0, 64,        // 15: 2
    0, 0, 0, 0,         // 16: 0
    0, 0, 0, 0,         // 17: 0
    0, 0, 128, 63,      // 18: 1

    5, 13, 14, 35,                              // 0: n99929
    5, 11, 12, 34,                              // 4: n961742
    6, 34, 9, 10, 33,                           // 8: size
    6, 7, 24, 8, 32,                            // 13: n436392
    3, 15, 35, 16, 33, 33, 17, 18, 32, 19, 36,  // 18: n353535
    3, 3, 35, 4, 33, 5, 33, 6, 32, 19, 31,      // 29: n676149
    6, 36, 31, 2, 30,                           // 40: r6
    4, 0, 1, 30, 27,                            // 45: r7
    0, 27                                       // 50: end program
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
