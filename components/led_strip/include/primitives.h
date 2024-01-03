
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "colortemp.h"
#include "esp_log.h"
#include "functions.h"

#define DMX_OFFSET 512

// see shaderformat.md
#define VERSION_OFFSET 0
#define ID_OFFSET 1
#define MEM_START_OFFSET 2
#define MEM_SIZE_OFFSET 4
#define PROG_START_OFFSET 6
#define PROG_SIZE_OFFSET 8

typedef unsigned char byte;

// getters and setters into mem
float getFloat(byte *mem, byte ptr) {
    float f;
    memcpy(&f, mem + ptr * 4, 4);
    return f;
}

uint16_t getUint16(byte *mem, byte ptr) {
    uint16_t i;
    memcpy(&i, mem + ptr, 2);
    return i;
}

uint8_t getUint8(byte *mem, byte ptr) {
    uint8_t i;
    memcpy(&i, mem + ptr, 1);
    return i;
}

uint8_t getDMX(byte *mem, byte ptr) {
    uint8_t i;
    memcpy(&i, mem + DMX_OFFSET + ptr, 1);
    return i;
}

float getMaster(byte *mem, byte *shader) {
    byte prog_start = getUint16(shader, PROG_START_OFFSET);
    byte prog_size = getUint16(shader, PROG_SIZE_OFFSET);
    // master value is the first param of the end program instruction
    // end program instruction has 2 params and is the last instruction of the program
    // so the master value is referenced by second to last byte of the program
    byte _master = *(shader + prog_start + prog_size - 2);
    float m = getFloat(mem, _master);
    return m;
}

void setFloat(byte *mem, byte ptr, float f) {
    memcpy(mem + ptr * 4, (byte *)(&f), 4);
}

void setMem(byte *mem, byte *shader) {
    // get mem start and size from shader
    uint16_t memstart = getUint16(shader, MEM_START_OFFSET);
    uint16_t memsize = getUint16(shader, MEM_SIZE_OFFSET);
    memcpy(mem, shader + memstart, memsize);
}

void setDMX(byte *mem, byte *data) {
    memcpy(mem + DMX_OFFSET, data, 512);
}

void setShader(byte *prog, byte *data, byte size) {
    memcpy(prog, data, size);
}

void setMapping(byte *mem, byte ptr, byte *mapping, int index) {
    memcpy(mem + ptr * 4, mapping + (index * 24), 24);
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
void node_condition(byte *mem, byte *prog, byte *counter) {
    byte _condition = data_fetch(mem, prog, counter);
    byte _ref = data_fetch(mem, prog, counter);
    float condition = getFloat(mem, _condition);
    // ESP_LOGI("condition", "condition %f", condition);
    if (condition > 0.5) {
        // just proceed to next instruction
    } else {
        // jump to ref
        *counter = _ref;
    }
}

void node_jump(byte *mem, byte *prog, byte *counter) {
    byte _ref = data_fetch(mem, prog, counter);
    *counter = _ref;
}

void node_store_jump(byte *mem, byte *prog, byte *counter) {
    byte _from = data_fetch(mem, prog, counter);
    byte _to = data_fetch(mem, prog, counter);
    byte _ref = data_fetch(mem, prog, counter);
    float r = getFloat(mem, _from);
    float g = getFloat(mem, _from + 1);
    float b = getFloat(mem, _from + 2);
    setFloat(mem, _to, r);
    setFloat(mem, _to + 1, g);
    setFloat(mem, _to + 2, b);
    *counter = _ref;
}

void node_label(byte *mem, byte *prog, byte *counter) {
    // just skips to next instruction
}

void node_color(byte *mem, byte *prog, byte *counter) {
    byte _col = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float r = getFloat(mem, _col);
    float g = getFloat(mem, _col + 1);
    float b = getFloat(mem, _col + 2);
    setFloat(mem, _result, r);
    setFloat(mem, _result + 1, g);
    setFloat(mem, _result + 2, b);
}

void node_value(byte *mem, byte *prog, byte *counter) {
    byte _val = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float val;
    val = getFloat(mem, _val);
    setFloat(mem, _result, val);
}

void node_dmxValue(byte *mem, byte *prog, byte *counter) {
    byte _channel = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float channel = getFloat(mem, _channel);
    // get value from dmx
    // dmx channels are 1 indexed
    uint8_t val8 = getDMX(mem, (int)channel - 1);
    // printf("%i %i \n", (int)channel, val8);
    float val = val8 / 255.0;

    setFloat(mem, _result, val);
}

void node_gradient(byte *mem, byte *prog, byte *counter) {
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

void node_smoothstep(byte *mem, byte *prog, byte *counter) {
    byte _x = data_fetch(mem, prog, counter);
    byte _a = data_fetch(mem, prog, counter);
    byte _b = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float x = getFloat(mem, _x);
    float a = getFloat(mem, _a);
    float b = getFloat(mem, _b);
    float value = smoothstep(a, b, x);
    setFloat(mem, _result, value);
}

void node_waveform(byte *mem, byte *prog, byte *counter) {
    byte _period = data_fetch(mem, prog, counter);
    byte _speed = data_fetch(mem, prog, counter);
    byte _phase = data_fetch(mem, prog, counter);
    byte _duty = data_fetch(mem, prog, counter);
    byte _leading = data_fetch(mem, prog, counter);
    byte _trailing = data_fetch(mem, prog, counter);
    // byte _smooth =
    data_fetch(mem, prog, counter);
    byte _x = data_fetch(mem, prog, counter);
    byte _t = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);

    float period = getFloat(mem, _period);
    float speed = getFloat(mem, _speed);
    float phase = getFloat(mem, _phase);
    float duty = getFloat(mem, _duty);
    float leading = getFloat(mem, _leading);
    float trailing = getFloat(mem, _trailing);
    // float smooth = getFloat(mem, _smooth);
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

void node_hsv2rgb(byte *mem, byte *prog, byte *counter) {
    byte _h = data_fetch(mem, prog, counter);
    byte _s = data_fetch(mem, prog, counter);
    byte _v = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);

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

void node_combineRGB(byte *mem, byte *prog, byte *counter) {
    byte _r = data_fetch(mem, prog, counter);
    byte _g = data_fetch(mem, prog, counter);
    byte _b = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);

    float r = getFloat(mem, _r);
    float g = getFloat(mem, _g);
    float b = getFloat(mem, _b);

    setFloat(mem, _result, r);
    setFloat(mem, _result + 1, g);
    setFloat(mem, _result + 2, b);
}

void node_temp2rgb(byte *mem, byte *prog, byte *counter) {
    byte _t = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);

    float t = getFloat(mem, _t);
    int index = 3 * (int)((t - 1000) / 100);  // table starts at 1000 K, increments by 100 K

    byte r = colortemp[index];
    byte g = colortemp[index + 1];
    byte b = colortemp[index + 2];

    // const char *TAG = "colortemp";
    // ESP_LOGI(TAG, "temp %f index %i rgb %i %i %i", t, index, r, g, b);

    setFloat(mem, _result, r / 255.0);
    setFloat(mem, _result + 1, g / 255.0);
    setFloat(mem, _result + 2, b / 255.0);
}

void node_math(byte *mem, byte *prog, byte *counter) {
    byte _a = data_fetch(mem, prog, counter);
    byte _b = data_fetch(mem, prog, counter);
    byte _fn = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float a = getFloat(mem, _a);
    float b = getFloat(mem, _b);
    int fn = (int)(getFloat(mem, _fn));

    float value = 0;

    switch (fn) {
        case 0:
            value = a + b;
            break;
        case 1:
            value = a - b;
            break;
        case 2:
            value = a * b;
            break;
        case 3:
            value = a / b;
            break;
        case 4:
            value = fmin(a, b);
            break;
        case 5:
            value = fmax(a, b);
            break;
    }

    setFloat(mem, _result, value);
}

void node_mapRange(byte *mem, byte *prog, byte *counter) {
    byte _v = data_fetch(mem, prog, counter);
    byte _fmin = data_fetch(mem, prog, counter);
    byte _fmax = data_fetch(mem, prog, counter);
    byte _tmin = data_fetch(mem, prog, counter);
    byte _tmax = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);

    float v = getFloat(mem, _v);
    float fmin = getFloat(mem, _fmin);
    float fmax = getFloat(mem, _fmax);
    float tmin = getFloat(mem, _tmin);
    float tmax = getFloat(mem, _tmax);

    float result =
        tmin + ((tmax - tmin) * (v - fmin)) / (fmax - fmin);

    setFloat(mem, _result, result);
}

void node_compare(byte *mem, byte *prog, byte *counter) {
    byte _a = data_fetch(mem, prog, counter);
    byte _b = data_fetch(mem, prog, counter);
    byte _fn = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float a = getFloat(mem, _a);
    float b = getFloat(mem, _b);
    int fn = (int)(getFloat(mem, _fn));

    float value = 0;

    switch (fn) {
        case 0:
            value = a <= b;
            break;
        case 1:
            value = a < b;
            break;
        case 2:
            value = a == b;
            break;
        case 3:
            value = a > b;
            break;
        case 4:
            value = a >= b;
            break;
        case 5:
            value = a != b;
            break;
    }

    setFloat(mem, _result, value);
}

void node_trig(byte *mem, byte *prog, byte *counter) {
    byte _x = data_fetch(mem, prog, counter);
    byte _fn = data_fetch(mem, prog, counter);
    byte _result = data_fetch(mem, prog, counter);
    float x = getFloat(mem, _x);
    int fn = (int)(getFloat(mem, _fn));

    float value = 0;
    switch (fn) {
        case 0:
            value = sin(x);
            break;
        case 1:
            value = cos(x);
            break;
        case 2:
            value = tan(x);
            break;
        case 3:
            value = sin(2 * M_PI * x);
            break;
        case 4:
            value = cos(2 * M_PI * x);
            break;
        case 5:
            value = tan(2 * M_PI * x);
            break;
        case 6:
            value = frac(x);
            break;
        case 7:
            value = rnd(x);
            break;
        case 8:
            value = round(x);
            break;
        case 9:
            value = floor(x);
            break;
        case 10:
            value = ceil(x);
            break;
        case 11:
            value = fabs(x);
            break;
    }

    setFloat(mem, _result, value);
}

void execute(byte *mem, byte *prog, byte *counter) {
    byte op = instruction_fetch(mem, prog, counter);
    while (op) {
        switch (op) {
            case 0:
                // end of program
                return;
            case 1:
                node_color(mem, prog, counter);
                break;
            case 2:
                node_gradient(mem, prog, counter);
                break;
            case 3:
                node_waveform(mem, prog, counter);
                break;
            case 4:
                node_hsv2rgb(mem, prog, counter);
                break;
            case 5:
                node_value(mem, prog, counter);
                break;
            case 6:
                node_math(mem, prog, counter);
                break;
            case 7:
                node_trig(mem, prog, counter);
                break;
            case 8:
                node_condition(mem, prog, counter);
                break;
            case 9:
                node_compare(mem, prog, counter);
                break;
            case 10:
                node_label(mem, prog, counter);
                break;
            case 11:
                node_jump(mem, prog, counter);
                break;
            case 12:
                node_store_jump(mem, prog, counter);
                break;
            case 13:
                node_temp2rgb(mem, prog, counter);
                break;
            case 14:
                node_mapRange(mem, prog, counter);
                break;
            case 15:
                node_dmxValue(mem, prog, counter);
                break;
            case 17:
                node_smoothstep(mem, prog, counter);
                break;
            case 18:
                node_combineRGB(mem, prog, counter);
                break;
        }
        op = instruction_fetch(mem, prog, counter);
    }
}
