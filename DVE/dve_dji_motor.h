/**
  ******************************************************************************
  * @file    dve_dji_motor.h
  * @brief   DJI 协议电机设备层接口
  ******************************************************************************
  */

#ifndef __DVE_DJI_MOTOR_H
#define __DVE_DJI_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "bsp_can.h"
#include <stdint.h>
#include <stdbool.h>

/* 宏定义 --------------------------------------------------------------------*/

#define DVE_DJI_MOTOR_STD_ID_BASE       (0x201U)    /* 当前 DJI 电机反馈 ID 起始值 */
#define DVE_DJI_MOTOR_STD_ID_MAX        (0x205U)    /* 当前 DJI 电机反馈 ID 最大值 */

#define DVE_DJI_TX_ID_GROUP_200         (0x200U)    /* 控制 0x201~0x204 */
#define DVE_DJI_TX_ID_GROUP_1FF         (0x1FFU)    /* 控制 0x205~0x208 */

#define DVE_DJI_CURRENT_MAX             (16384)     /* C620 电流控制值限幅 */
#define DVE_DJI_ECD_RANGE               (8192.0f)   /* 单圈编码器范围 0~8191 */
#define DVE_DJI_CAN_FRAME_LEN           (8U)        /* DJI 反馈/控制帧长度 */
#define DVE_DJI_GROUP_MOTOR_NUM         (4U)        /* 每组控制帧最多控制 4 个电机 */

/* 类型定义 ------------------------------------------------------------------*/

/**
  * @brief DJI 电机软件编号
  * @note  枚举值只是数组下标，不是 CAN ID。真实 CAN ID 在 .c 配置表中绑定。
  */
typedef enum
{
    DJI_MOTOR_CHASSIS_1 = 0,        /* 软件下标0，对应 CAN2 ID 0x201 */
    DJI_MOTOR_CHASSIS_2,            /* 软件下标1，对应 CAN2 ID 0x202 */
    DJI_MOTOR_CHASSIS_3,            /* 软件下标2，对应 CAN2 ID 0x203 */
    DJI_MOTOR_CHASSIS_4,            /* 软件下标3，对应 CAN2 ID 0x204 */

    DJI_MOTOR_HIT,                  /* 软件下标4，对应 CAN2 ID 0x205 */

    DJI_MOTOR_NUM
} DVE_DjiMotorId_t;

typedef enum
{
    DVE_DJI_TX_GROUP_200 = 0,       /* 发送 0x200 控制帧 */
    DVE_DJI_TX_GROUP_1FF            /* 发送 0x1FF 控制帧 */
} DVE_DjiTxGroup_t;

typedef enum
{
    DVE_DJI_OK              = 0,    /* 操作成功 */
    DVE_DJI_ERR_PARAM       = 1,    /* 参数错误 */
    DVE_DJI_ERR_CAN         = 2,    /* CAN 发送失败 */
    DVE_DJI_ERR_NOT_INIT    = 3,    /* 模块未初始化 */
    DVE_DJI_ERR_UNSUPPORTED = 4     /* 当前接口暂不支持 */
} DVE_DjiMotorStatus_t;

typedef struct
{
    uint16_t ecd;                   /* 原始编码器值，0~8191 */
    float angle_deg;                /* 单圈机械角度，单位 deg */
    int16_t speed_rpm;              /* 转速，单位 rpm */
    int16_t current;                /* 反馈转矩电流 */
    uint8_t temp;                   /* 电机温度 */
    uint32_t last_update_ms;        /* 最近一次反馈时间，HAL tick，单位 ms */
    bool online;                    /* 是否收到过有效反馈 */
} DVE_DjiMotorFeedback_t;

/* 函数声明 ------------------------------------------------------------------*/

/* 初始化 DJI 电机对象表。当前 hcan 应传 &hcan2。 */
DVE_DjiMotorStatus_t DVE_DjiMotor_Init(CAN_HandleTypeDef *hcan);

/* CAN RX 中断分发入口。只解析反馈并更新缓存，不做控制计算。 */
bool DVE_DjiMotor_RxCallback(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *rx_header,
                             const uint8_t rx_data[8]);

/* 读取某个电机的反馈副本。 */
DVE_DjiMotorStatus_t DVE_DjiMotor_GetFeedback(DVE_DjiMotorId_t id,
                                              DVE_DjiMotorFeedback_t *feedback);

/* 快速读取转速。内部仍然通过 GetFeedback 拷贝，避免直接暴露结构体。 */
int16_t DVE_DjiMotor_GetSpeedRpm(DVE_DjiMotorId_t id);

/* 根据 last_update_ms 判断电机是否在线。 */
bool DVE_DjiMotor_IsOnline(DVE_DjiMotorId_t id, uint32_t timeout_ms);

/* 设置某个电机的输出电流缓存，不会立即发送 CAN。 */
DVE_DjiMotorStatus_t DVE_DjiMotor_SetCurrent(DVE_DjiMotorId_t id,
                                             int32_t current);

/* 清除某个电机的输出电流缓存。 */
DVE_DjiMotorStatus_t DVE_DjiMotor_ClearCurrent(DVE_DjiMotorId_t id);

/* 清除所有 DJI 电机的输出电流缓存。 */
DVE_DjiMotorStatus_t DVE_DjiMotor_ClearAllCurrent(void);

/* 发送一组控制帧：0x200 或 0x1FF。 */
DVE_DjiMotorStatus_t DVE_DjiMotor_SendCurrentGroup(DVE_DjiTxGroup_t group);

/* 同时发送 0x200 和 0x1FF 两组控制帧。 */
DVE_DjiMotorStatus_t DVE_DjiMotor_SendAllCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* __DVE_DJI_MOTOR_H */
