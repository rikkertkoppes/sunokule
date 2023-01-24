
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "functions.h"

typedef unsigned char byte;

// getters and setters into mem
float getFloat(byte *mem, byte ptr) {
    float f;
    memcpy(&f, mem + ptr * 4, 4);
    return f;
}

void setFloat(byte *mem, byte ptr, float f) {
    memcpy(mem + ptr * 4, (byte *)(&f), 4);
}

void setMem(byte *mem, byte *data) {
    byte size = data[0] * 4;
    memcpy(mem, data + 1, size);
}

// runner
byte instruction_fetch(byte *mem, byte *prog, byte *counter) {
    byte op = *(prog + *(counter));
    *counter += 1;
    return op;
}

byte data_fetch(byte *mem, byte *prog, byte *counter) {
    byte ptr = *(prog + *(counter));
    *counter += 1;
    return ptr;
}

// primitives
void color(byte *mem, byte *prog, byte *counter) {
    byte _col = data_fetch(mem, prog, counter);
    byte _control = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float r = getFloat(mem, _col);
    float g = getFloat(mem, _col + 1);
    float b = getFloat(mem, _col + 2);
    setFloat(mem, _result, r);
    setFloat(mem, _result + 1, g);
    setFloat(mem, _result + 2, b);
}

void value(byte *mem, byte *prog, byte *counter) {
    byte _val = data_fetch(mem, prog, counter);
    byte _control = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float val = getFloat(mem, _val);
    setFloat(mem, _result, val);
}

void gradient(byte *mem, byte *prog, byte *counter) {
    byte _t = data_fetch(mem, prog, counter);
    byte _col1 = data_fetch(mem, prog, counter);
    byte _col2 = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float t = getFloat(mem, _t);
    float u = 1 - t;
    float r = u * getFloat(mem, _col1) + t * getFloat(mem, _col2);
    float g = u * getFloat(mem, _col1 + 1) + t * getFloat(mem, _col2 + 1);
    float b = u * getFloat(mem, _col1 + 2) + t * getFloat(mem, _col2 + 2);
    setFloat(mem, _result, r);
    setFloat(mem, _result + 1, g);
    setFloat(mem, _result + 2, b);
}

void waveform(byte *mem, byte *prog, byte *counter) {
    byte _period = data_fetch(mem, prog, counter);
    byte _speed = data_fetch(mem, prog, counter);
    byte _phase = data_fetch(mem, prog, counter);
    byte _duty = data_fetch(mem, prog, counter);
    byte _leading = data_fetch(mem, prog, counter);
    byte _trailing = data_fetch(mem, prog, counter);
    byte _smooth = data_fetch(mem, prog, counter);
    byte _x = data_fetch(mem, prog, counter);
    byte _t = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);

    float period = getFloat(mem, _period);
    float speed = getFloat(mem, _speed);
    float phase = getFloat(mem, _phase);
    float duty = getFloat(mem, _duty);
    float leading = getFloat(mem, _leading);
    float trailing = getFloat(mem, _trailing);
    float smooth = getFloat(mem, _smooth);
    float x = getFloat(mem, _x);
    float t = getFloat(mem, _t);
    t *= speed;
    x /= period;
    x = frac(x - phase - t);
    if (period < 0) {
        float temp = leading;
        leading = trailing;
        trailing = temp;
    }
    float value = 0;
    if (x < duty) {
        value = 1;
        if (leading != 0.0) value = min(value, x / leading);
        if (trailing != 0.0) value = min(value, (duty - x) / trailing);
    } else {
        value = -1;
        if (leading != 0.0) value = max(value, (x - 1) / leading);
        if (trailing != 0.0) value = max(value, (duty - x) / trailing);
    }
    // TODO: smooth;
    value = (value + 1) / 2;
    setFloat(mem, _result, value);
}

void hsv2rgb(byte *mem, byte *prog, byte *counter) {
    byte _h = *(prog + *counter);
    byte _s = *(prog + *counter + 1);
    byte _v = *(prog + *counter + 2);
    byte _result = *(prog + *counter + 3);

    float h = getFloat(mem, _h);
    float s = getFloat(mem, _s);
    float v = getFloat(mem, _v);

    float rgb_max = v;
    float rgb_min = rgb_max * (1 - s);
    h = frac(h);  // [0,1] always, wrap around

    float r = 0, g = 0, b = 0;

    uint32_t i = ((int)(359 * h)) / 60;
    uint32_t diff = ((int)(359 * h)) % 60;

    // RGB adjustment amount by hue
    float rgb_adj = (rgb_max - rgb_min) * diff / 60.0;

    switch (i) {
        case 0:
            r = rgb_max;
            g = rgb_min + rgb_adj;
            b = rgb_min;
            break;
        case 1:
            r = rgb_max - rgb_adj;
            g = rgb_max;
            b = rgb_min;
            break;
        case 2:
            r = rgb_min;
            g = rgb_max;
            b = rgb_min + rgb_adj;
            break;
        case 3:
            r = rgb_min;
            g = rgb_max - rgb_adj;
            b = rgb_max;
            break;
        case 4:
            r = rgb_min + rgb_adj;
            g = rgb_min;
            b = rgb_max;
            break;
        default:
            r = rgb_max;
            g = rgb_min;
            b = rgb_max - rgb_adj;
            break;
    }
    // ESP_LOGI("primitives", "rgb %f, %i %f %f %f", h, i, r, g, b);
    setFloat(mem, _result, r);
    setFloat(mem, _result + 1, g);
    setFloat(mem, _result + 2, b);
}
