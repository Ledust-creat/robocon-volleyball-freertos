#ifndef HOST_INCLUDES_H
#define HOST_INCLUDES_H
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "pid.h"
/* Minimal legacy declarations needed by the real board-init source. */
#define MotorCount 4
typedef struct { int unused; } motor_info_t;
#endif
