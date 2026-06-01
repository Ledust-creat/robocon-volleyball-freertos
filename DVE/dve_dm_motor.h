/**
  ******************************************************************************
  * @file    dve_dm_motor.h
  * @brief   达妙 DM 电机设备层接口
  ******************************************************************************
  */

#ifndef __DVE_DM_MOTOR_H
#define __DVE_DM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "bsp_can.h"
#include <stdint.h>
#include <stdbool.h>

/* 宏定义 --------------------------------------------------------------------*/

#define DVE_DM_CAN_ID_OFFSET_MIT        (0x000U)    /* MIT模式 / 特殊命令 ID 偏移 */
#define DVE_DM_CAN_ID_OFFSET_POS        (0x100U)    /* 位置模式 ID 偏移 */
#define DVE_DM_CAN_ID_OFFSET_SPEED      (0x200U)    /* 速度模式 ID 偏移 */

#define DVE_DM_CMD_FRAME_LEN            (8U)        /* 使能/失能/设零点命令长度 */
#define DVE_DM_MIT_FRAME_LEN            (8U)        /* MIT模式控制帧长度 */
#define DVE_DM_POS_FRAME_LEN            (8U)        /* 位置模式控制帧长度 */
#define DVE_DM_SPEED_FRAME_LEN          (4U)        /* 速度模式控制帧长度 */

#define DVE_DM_POS_MIN_RAD              (-12.5f)    /* MIT位置范围，单位rad */
#define DVE_DM_POS_MAX_RAD              ( 12.5f)
#define DVE_DM_VEL_MIN_RAD_S            (-30.0f)    /* MIT速度范围，单位rad/s */
#define DVE_DM_VEL_MAX_RAD_S            ( 30.0f)
#define DVE_DM_KP_MIN                   (  0.0f)    /* MIT Kp范围 */
#define DVE_DM_KP_MAX                   (500.0f)
#define DVE_DM_KD_MIN                   (  0.0f)    /* MIT Kd范围 */
#define DVE_DM_KD_MAX                   (  5.0f)
#define DVE_DM_TORQUE_MIN_NM            (-10.0f)    /* MIT前馈力矩范围，单位Nm */
#define DVE_DM_TORQUE_MAX_NM            ( 10.0f)

#define DVE_DM_POS_BITS                 (16U)
#define DVE_DM_VEL_BITS                 (12U)
#define DVE_DM_KP_BITS                  (12U)
#define DVE_DM_KD_BITS                  (12U)
#define DVE_DM_TORQUE_BITS              (12U)

#define DVE_DM_CMD_ENABLE               (0xFCU)     /* 特殊命令：使能 */
#define DVE_DM_CMD_DISABLE              (0xFDU)     /* 特殊命令：失能 */
#define DVE_DM_CMD_ZERO                 (0xFEU)     /* 特殊命令：当前位置设为零点 */

/* 类型定义 ------------------------------------------------------------------*/

/**
  * @brief DM电机软件编号
  * @note  枚举值只是软件下标，不是CAN ID。真实CAN ID在.c配置表中绑定。
  */
typedef enum
{
    DM_MOTOR_LIFT_1 = 0,              /* 抬球DM电机1 */
    DM_MOTOR_LIFT_2,                  /* 抬球DM电机2 */
    DM_MOTOR_LIFT_3,                  /* 抬球DM电机3 */

    DM_MOTOR_NUM
} DVE_DmMotorId_t;

typedef enum
{
    DVE_DM_OK              = 0,       /* 操作成功 */
    DVE_DM_ERR_PARAM       = 1,       /* 参数错误 */
    DVE_DM_ERR_CAN         = 2,       /* CAN发送失败 */
    DVE_DM_ERR_NOT_INIT    = 3,       /* 模块未初始化 */
    DVE_DM_ERR_UNSUPPORTED = 4        /* 当前接口暂不支持 */
} DVE_DmMotorStatus_t;

/* 函数声明 ------------------------------------------------------------------*/

/* 初始化DM电机模块。当前hcan应传&hcan1。 */
DVE_DmMotorStatus_t DVE_DmMotor_Init(CAN_HandleTypeDef *hcan);

/* 发送DM电机使能/失能/设零点特殊命令。 */
DVE_DmMotorStatus_t DVE_DmMotor_Enable(DVE_DmMotorId_t id);
DVE_DmMotorStatus_t DVE_DmMotor_Disable(DVE_DmMotorId_t id);
DVE_DmMotorStatus_t DVE_DmMotor_SetZero(DVE_DmMotorId_t id);

/* 发送DM电机MIT模式控制命令。 */
DVE_DmMotorStatus_t DVE_DmMotor_SetMit(DVE_DmMotorId_t id,
                                       float pos_rad,
                                       float vel_rad_s,
                                       float kp,
                                       float kd,
                                       float torque_nm);

/* 发送DM电机速度模式控制命令。 */
DVE_DmMotorStatus_t DVE_DmMotor_SetSpeed(DVE_DmMotorId_t id,
                                         float speed_rad_s);

/* 发送DM电机位置模式控制命令。 */
DVE_DmMotorStatus_t DVE_DmMotor_SetPosition(DVE_DmMotorId_t id,
                                            float pos_rad,
                                            float vel_rad_s);

/* CAN RX中断分发入口。当前只预留接口，不解析反馈。 */
bool DVE_DmMotor_RxCallback(CAN_HandleTypeDef *hcan,
                            const CAN_RxHeaderTypeDef *rx_header,
                            const uint8_t rx_data[8]);

#ifdef __cplusplus
}
#endif

#endif /* __DVE_DM_MOTOR_H */
