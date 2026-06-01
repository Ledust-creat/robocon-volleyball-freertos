/**
  ******************************************************************************
  * @file    N630.h
  * @brief   N630 CAN通信驱动头文件
  * @author  Auto-generated
  * @date    2026-01-31
  ******************************************************************************
  * @attention
  * 1. 所有CAN帧均使用29位扩展ID
  * 2. 数据长度超过8字节时会被自动截断
  * 3. 依赖STM32 HAL库及CAN底层驱动(can.h)
  ******************************************************************************
  */

#ifndef __N630_H
#define __N630_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含依赖头文件 */
#include "stdint.h"
#include "stdbool.h"

/**
 * @brief CAN通信指令包ID枚举
 * @note 32位枚举保证跨平台兼容性，避免类型截断
 */
typedef enum {
    CAN_PACKET_SET_DUTY = 0,                /* 设置占空比 */
    CAN_PACKET_SET_CURRENT,                 /* 设置电流 */
    CAN_PACKET_SET_CURRENT_BRAKE,           /* 设置制动电流 */
    CAN_PACKET_SET_RPM,                     /* 设置转速 */
    CAN_PACKET_SET_POS,                     /* 设置位置 */
    CAN_PACKET_SET_CURRENT_REL = 10,        /* 设置相对电流 */
    CAN_PACKET_SET_CURRENT_BRAKE_REL,       /* 设置相对制动电流 */
    CAN_PACKET_SET_CURRENT_HANDBRAKE,       /* 设置手刹电流 */
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,   /* 设置相对手刹电流 */
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF /* 强制32位枚举 */
} CAN_PACKET_ID;

/**
 * @brief 发送29位扩展ID CAN帧
 * @param id: 29位扩展CAN ID
 * @param data: 发送数据缓冲区指针(非空)
 * @param len: 发送数据长度(0~8，超过8会自动截断)
 * @note 仅当CAN发送邮箱空闲时才会发送
 */
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief 向缓冲区追加16位有符号整数(大端序)
 * @param buffer: 目标缓冲区指针(非空)
 * @param number: 待追加的16位有符号整数
 * @param index: 缓冲区写入索引指针(会自动自增)
 */
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);

/**
 * @brief 向缓冲区追加32位有符号整数(大端序)
 * @param buffer: 目标缓冲区指针(非空)
 * @param number: 待追加的32位有符号整数
 * @param index: 缓冲区写入索引指针(会自动自增)
 */
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);

/**
 * @brief 向缓冲区追加浮点型数据(按比例转换为16位整数，大端序)
 * @param buffer: 目标缓冲区指针(非空)
 * @param number: 待追加的浮点型数据
 * @param scale: 缩放比例(如1e3表示*1000后取整)
 * @param index: 缓冲区写入索引指针(会自动自增)
 */
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);

/**
 * @brief 向缓冲区追加浮点型数据(按比例转换为32位整数，大端序)
 * @param buffer: 目标缓冲区指针(非空)
 * @param number: 待追加的浮点型数据
 * @param scale: 缩放比例(如1e5表示*100000后取整)
 * @param index: 缓冲区写入索引指针(会自动自增)
 */
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);

/**
 * @brief CAN设置占空比指令
 * @param controller_id: 控制器ID(8位)
 * @param duty: 占空比(浮点型，实际值*100000后传输)
 */
void comm_can_set_duty(uint8_t controller_id, float duty);

/**
 * @brief CAN设置电流指令(4字节版本)
 * @param controller_id: 控制器ID(8位)
 * @param current: 电流值(浮点型，实际值*1000后传输)
 */
void comm_can_set_current(uint8_t controller_id, float current);

/**
 * @brief CAN设置电流指令(带关断延时，6字节版本)
 * @param controller_id: 控制器ID(8位)
 * @param current: 电流值(浮点型，实际值*1000后传输)
 * @param off_delay: 关断延时(浮点型，实际值*1000后转换为16位整数传输)
 * @note 延时用于电流低于最小值时保持控制器运行一段时间
 */
void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);

/**
 * @brief CAN设置制动电流指令
 * @param controller_id: 控制器ID(8位)
 * @param current: 制动电流值(浮点型，实际值*1000后传输)
 */
void comm_can_set_current_brake(uint8_t controller_id, float current);

/**
 * @brief CAN设置转速指令
 * @param controller_id: 控制器ID(8位)
 * @param rpm: 转速值(浮点型，直接取整为32位整数传输)
 */
void comm_can_set_rpm(uint8_t controller_id, float rpm);

/**
 * @brief CAN设置位置指令
 * @param controller_id: 控制器ID(8位)
 * @param pos: 位置值(浮点型，实际值*1000000后传输)
 */
void comm_can_set_pos(uint8_t controller_id, float pos);

/**
 * @brief CAN设置相对电流指令
 * @param controller_id: 控制器ID(8位)
 * @param current_rel: 相对电流值(浮点型，实际值*1e5后传输)
 */
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);

/**
 * @brief CAN设置相对电流指令(带关断延时，6字节版本)
 * @param controller_id: 控制器ID(8位)
 * @param current_rel: 相对电流值(浮点型，实际值*1e5后传输)
 * @param off_delay: 关断延时(浮点型，实际值*1000后转换为16位整数传输)
 * @note 延时用于电流低于最小值时保持控制器运行一段时间
 */
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay);

/**
 * @brief CAN设置相对制动电流指令
 * @param controller_id: 控制器ID(8位)
 * @param current_rel: 相对制动电流值(浮点型，实际值*1e5后传输)
 */
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);

/**
 * @brief CAN设置手刹电流指令
 * @param controller_id: 控制器ID(8位)
 * @param current: 手刹电流值(浮点型，实际值*1000后传输)
 */
void comm_can_set_handbrake(uint8_t controller_id, float current);

/**
 * @brief CAN设置相对手刹电流指令
 * @param controller_id: 控制器ID(8位)
 * @param current_rel: 相对手刹电流值(浮点型，实际值*1e5后传输)
 */
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);

#ifdef __cplusplus
}
#endif

#endif /* __N630_H */