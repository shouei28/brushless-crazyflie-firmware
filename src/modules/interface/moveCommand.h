#pragma once

#include <stdint.h>

typedef struct {
    int16_t vx;       // velocity x
    int16_t vy;       // velocity y
    int16_t vz;       // velocity z
    int16_t yawRate;  // yaw rotation speed
} MoveCommand;
