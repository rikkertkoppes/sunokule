
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

#endif /* !RUNNER_H */
