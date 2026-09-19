#ifndef __HIT_TASK_H
#define __HIT_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

/* 宏定义 --------------------------------------------------------------------*/

#define HIT_TASK_PRIO                  (25U)        /* 击球任务优先级 */
#define HIT_TASK_SIZE                  (256U)       /* 击球任务栈大小，单位为 StackType_t */
#define HIT_TASK_PERIOD_MS             (10U)        /* 击球控制周期，单位 ms */

#define HIT_CMD_TIMEOUT_MS             (100U)       /* 遥控命令超时时间，超时后 0x1FF 组输出清零 */
#define HIT_MOTOR_OFFLINE_MS           (100U)       /* 击球电机反馈超时时间，超时后对应电机输出清零 */

#define HIT_MOTOR_NUM                  (3U)         /* 击球 3508 数量，当前为 0x206~0x208 */
#define HIT_PRESET_NUM                 (3U)         /* 击球预设角度组数，对应 SWC 三档 */

#define HIT_SCALE                      (19.0f)      /* 机构角度到电机累计角度的比例系数 */
#define HIT_RETURN_STEP                (0.25f * HIT_SCALE)  /* 回零时每周期目标角度变化量 */

#define HIT_OUTPUT_LIMIT               (20000)      /* 击球电机输出限幅 */
#define HIT_PITCH_OUTPUT_LIMIT         (10000)      /* 0x205 俯仰/机械臂电机输出限幅 */

#define HIT_PRESET0_ANGLE_DEG          (65.0f)      /* SWC_UP 对应击球角度，单位 deg */
#define HIT_PRESET1_ANGLE_DEG          (25.0f)      /* SWC_MID 对应击球角度，单位 deg */
#define HIT_PRESET2_ANGLE_DEG          (15.0f)      /* SWC_DOWN 对应击球角度，单位 deg */
#define HIT_RESET_ANGLE_DEG            (10.0f)      /* SF 松开后的回零角度，单位 deg */

#define HIT_MOTOR1_DIR                 (-1.0f)      /* 0x206 击球电机方向修正 */
#define HIT_MOTOR2_DIR                 (-1.0f)      /* 0x207 击球电机方向修正 */
#define HIT_MOTOR3_DIR                 ( 1.0f)      /* 0x208 击球电机方向修正 */

#define HIT_PITCH_SPEED_SET            (25000.0f)   /* SWD_DOWN 时 0x205 目标速度 */
#define HIT_PITCH_SPEED_ZERO           (0.0f)       /* SWD 松开时 0x205 目标速度 */

/* 类型定义 ------------------------------------------------------------------*/

typedef struct
{
    uint8_t enable;                    /* 击球功能使能，0 表示停机保护 */
    uint8_t preset_index;              /* 击球预设编号，范围 0~2 */
    uint8_t hit_pressed;               /* SF 是否按下，1 为动作角度，0 为回零角度 */
    float pitch_speed_set;             /* 0x205 俯仰/机械臂目标速度，单位 rpm */
} Hit_Cmd_t;

/* 全局变量声明 --------------------------------------------------------------*/

extern QueueHandle_t xHitCmdQueue;

/* 函数声明 ------------------------------------------------------------------*/

void hit_task_creat(void);
void hit_task(void *parm);

#endif /* __HIT_TASK_H */
