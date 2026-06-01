#include "math_utils.h"

// 最大限幅函数实现
float clamp_max(float value, float max_limit) {
    return (value > max_limit) ? max_limit : value;
}

// 最小限幅函数实现
float clamp_min(float value, float min_limit) {
    return (value < min_limit) ? min_limit : value;
}

// 归一化到[0, 1]区间（保持向下兼容）
float normalize(float value, float data_min, float data_max) {
    // 浮点型差值判断，避免除零错误（兼容src_min > src_max的异常输入）
    if ((data_max - data_min) < FLOAT_EPSILON && (data_min - data_max) < FLOAT_EPSILON) {
        return 0.0f;
    }
    // 核心公式：映射到[0, 1]
    return (value - data_min) / (data_max - data_min);
}

// 新增：归一化到任意正负目标区间
float normalize_to_range(float value, float src_min, float src_max, float dst_min, float dst_max) {
    // 第一步：先做除零保护，避免原始区间上下限相等导致程序异常
    if ((src_max - src_min) < FLOAT_EPSILON && (src_min - src_max) < FLOAT_EPSILON) {
        return 0.0f;
    }
    
    // 第二步：核心双线性映射公式（支持任意正负区间）
    // 1. 先将原始数据归一化到[0, 1]区间
    float norm_0_to_1 = (value - src_min) / (src_max - src_min);
    // 2. 再将[0, 1]区间映射到目标区间[dst_min, dst_max]
    return dst_min + norm_0_to_1 * (dst_max - dst_min);
}

// 整型绝对值函数实现
int abs_int(int num) {
    return (num < 0) ? -num : num;
}

// 浮点型绝对值函数实现（无依赖，适配STM32）
float abs_float(float value) {
    return (value < 0.0f) ? -value : value;
}
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
float low_pass(float input, float output_last, float alpha)
{
    return output_last + alpha * (input - output_last);
}