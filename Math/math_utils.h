#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>

/* 浮点型极小值，用于判断差值是否接近零 */
#define FLOAT_EPSILON 1e-6f

/* 原有接口：保持向下兼容 */
float clamp_max(float value, float max_limit);
float clamp_min(float value, float min_limit);
float normalize(float value, float data_min, float data_max);
float normalize_to_range(float value, float src_min, float src_max, float dst_min, float dst_max);
int abs_int(int num);
float abs_float(float value);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float low_pass(float input, float output_last, float alpha);

/* 新增接口：给 rc_task 等新代码使用，命名更清楚 */
float Math_ClampFloat(float value, float min_limit, float max_limit);
float Math_ApplyDeadzoneFloat(float value, float deadzone);

#endif /* MATH_UTILS_H */
