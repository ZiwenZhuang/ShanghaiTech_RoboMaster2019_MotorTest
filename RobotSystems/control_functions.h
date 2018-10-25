#ifndef CONTROL_H
#define CONTROL_H

#include "stdlib.h"
#include "stdint.h"
#include "string.h"

void mecanum_calc(float vx, float vy, float vw, float *speed);

#endif
