
#pragma once

#ifndef RUNNER_H
#define RUNNER_H

#include "primitives.h"

typedef unsigned char byte;

byte fetch(byte *mem, byte *prog, byte *counter) {
    byte op = *(prog + *(counter));
    *counter += 1;
    return op;
}

void execute(byte *mem, byte *prog, byte *counter) {
    byte op = fetch(mem, prog, counter);
    while (op) {
        switch (op) {
            case 0:
                // end of program
                return;
            case 1:
                color(mem, prog, counter);
                break;
            case 2:
                gradient(mem, prog, counter);
                break;
            case 3:
                waveform(mem, prog, counter);
                break;
            case 4:
                hsv2rgb(mem, prog, counter);
                break;
        }
        op = fetch(mem, prog, counter);
    }
}

#endif /* !RUNNER_H */
