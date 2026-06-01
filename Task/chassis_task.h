/**
  ******************************************************************************
  * @file    chassis_task.h
  * @brief   底盘控制任务接口
  ******************************************************************************
  */

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>

/* 类型定义 ------------------------------------------------------------------*/

typedef struct
{
    float vx;                       /* 前后速度命令，沿用原工程遥控标定 */
    float vy;                       /* 左右速度命令，沿用原工程遥控标定 */
    float wz;                       /* 旋转速度命令，沿用原工程遥控标定 */
} Chassis_MoveCmd_t;

/* 全局变量声明 --------------------------------------------------------------*/

extern QueueHandle_t xChassisCtrlQueue;
extern TaskHandle_t chassis_Task_Handle;

/* 函数声明 ------------------------------------------------------------------*/

void chassis_task_creat(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_TASK_H__ */
