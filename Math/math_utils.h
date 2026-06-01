#ifndef MATH_UTILS_H
#define MATH_UTILS_H

// 浮点型极小值，用于判断差值是否接近零（避免除零错误）
#define FLOAT_EPSILON 1e-6f

/**
 * @brief  最大限幅函数（上限位幅）
 * @param  value: 输入需要限幅的数值
 * @param  max_limit: 最大限制阈值（上限）
 * @retval 若value超过max_limit，返回max_limit；否则返回原value
 */
float clamp_max(float value, float max_limit);

/**
 * @brief  最小限幅函数（下限位幅）
 * @param  value: 输入需要限幅的数值
 * @param  min_limit: 最小限制阈值（下限）
 * @retval 若value低于min_limit，返回min_limit；否则返回原value
 */
float clamp_min(float value, float min_limit);

/**
 * @brief  数据归一化函数（映射到[0, 1]区间，保持向下兼容）
 * @param  value: 输入需要归一化的数值
 * @param  data_min: 数据系列的最小值（原始区间下限）
 * @param  data_max: 数据系列的最大值（原始区间上限）
 * @retval 归一化结果；若data_min≈data_max，返回0.0f避免除零错误
 */
float normalize(float value, float data_min, float data_max);

/**
 * @brief  数据归一化函数（映射到任意正负目标区间，新增核心功能）
 * @param  value: 输入需要归一化的数值
 * @param  src_min: 原始数据区间的最小值
 * @param  src_max: 原始数据区间的最大值
 * @param  dst_min: 目标区间的最小值（支持负数，如-1.0f）
 * @param  dst_max: 目标区间的最大值（支持负数，如5.0f）
 * @retval 映射到目标区间的归一化结果；若src_min≈src_max，返回0.0f避免除零错误
 */
float normalize_to_range(float value, float src_min, float src_max, float dst_min, float dst_max);

/**
 * @brief  整型绝对值函数
 * @param  num: 输入整型数值
 * @retval 输入数值的绝对值
 */
int abs_int(int num);

/**
 * @brief  浮点型绝对值函数（适配STM32，无需依赖math.h的fabsf）
 * @param  value: 输入浮点型数值
 * @retval 输入数值的绝对值
 */
float abs_float(float value);

/**
  * @brief  将浮点数值映射为指定位数的无符号整数（归一化转换）
  * @param  x_float: 待转换的浮点数值
  * @param  x_min:   该浮点数值的最小值（映射为整数0）
  * @param  x_max:   该浮点数值的最大值（映射为整数最大值）
  * @param  bits:    目标整数的位数（决定整数最大值，如8位→255，16位→65535）
  * @retval 转换后的无符号整数
  * @note   1. 实现原理：x_int = (x_float - x_min) / (x_max - x_min) * (2^bits - 1)
  *         2. 若x_float超出[x_min, x_max]范围，会被钳位到0或(2^bits - 1)
  *         3. 适用于嵌入式设备中浮点数据的量化（如CAN通讯、ADC/DAC数据转换）
  *         4. 示例：float_to_uint(2.5f, 0.0f, 5.0f, 8) → 127（2.5/5*255≈127）
  */
int float_to_uint(float x_float, float x_min, float x_max, int bits);

/**
  * @brief  将指定位数的无符号整数还原为浮点数值（反归一化转换）
  * @param  x_int:   待转换的无符号整数
  * @param  x_min:   浮点数值的最小值（对应整数0）
  * @param  x_max:   浮点数值的最大值（对应整数最大值）
  * @param  bits:    整数的位数（决定整数最大值，如12位→4095）
  * @retval 转换后的浮点数值
  * @note   1. 实现原理：x_float = x_min + x_int / (2^bits - 1) * (x_max - x_min)
  *         2. 若x_int超出[0, 2^bits - 1]范围，会按模或钳位处理（取决于实现）
  *         3. 适用于量化整数的还原（如接收CAN总线的量化数据后还原为物理量）
  *         4. 示例：uint_to_float(127, 0.0f, 5.0f, 8) → 2.4902f（127/255*5≈2.49）
  */
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float low_pass(float input, float output_last, float alpha);
#endif /* MATH_UTILS_H */
