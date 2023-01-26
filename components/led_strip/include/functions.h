
#pragma once

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdlib.h>

#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

// [0,oo] -> [0,1]
float frac(float x) {
    return x - floorf(x);
}
float smoothstep(float min, float max, float x) {
    float k = fmax(0, fmin(1, (x - min) / (max - min)));
    return k * k * (3.0 - 2.0 * k);
}
// any -> [0,1]
float rnd(float seed) {
    int s = seed * 2147483647;
    srand(s);
    int nr = rand();
    return (float)(nr) / (RAND_MAX);
}
// any -> [-1,1]
float triangular(float x) {
    return 2 * fabs((frac(x) * 2) - 1) - 1;
}

float boostLow(float x, uint8_t strength) {
    float k = x;
    while (strength--) k *= x;
    return k;
}

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(float h, float s, float v, float *r, float *g, float *b) {
    float rgb_max = v;
    float rgb_min = rgb_max * (1 - s);
    h = frac(h);  // [0,1] always, wrap around
    // h = fmod(h, 1);
    // h = fmod(h + 2, 1);

    uint32_t i = ((int)(359 * h)) / 60;
    uint32_t diff = ((int)(359 * h)) % 60;

    // RGB adjustment amount by hue
    float rgb_adj = (rgb_max - rgb_min) * diff / 60.0;

    // ESP_LOGI(TAG, "%i, %i, %i, %i, %i", h,s,v, rgb_min, rgb_max);

    switch (i) {
        case 0:
            *r = rgb_max;
            *g = rgb_min + rgb_adj;
            *b = rgb_min;
            break;
        case 1:
            *r = rgb_max - rgb_adj;
            *g = rgb_max;
            *b = rgb_min;
            break;
        case 2:
            *r = rgb_min;
            *g = rgb_max;
            *b = rgb_min + rgb_adj;
            break;
        case 3:
            *r = rgb_min;
            *g = rgb_max - rgb_adj;
            *b = rgb_max;
            break;
        case 4:
            *r = rgb_min + rgb_adj;
            *g = rgb_min;
            *b = rgb_max;
            break;
        default:
            *r = rgb_max;
            *g = rgb_min;
            *b = rgb_max - rgb_adj;
            break;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* !FUNCTIONS_H */
