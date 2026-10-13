#ifndef __RC_TASK_H
#define __RC_TASK_H

#include "task_init.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

/*
*********************************************************************************************************
*                                      宏定义
*********************************************************************************************************
*/

/* 遥控解析任务配置 */
#define RC_PARSE_TASK_PRIO               27
#define RC_PARSE_TASK_SIZE               256

/*
 * T15 通道映射配置
 *
 * 注意:
 * 如果遥控上下/左右反了，优先改这里，不要改 DVE_T15 解析函数。
 */
#define RC_CH_VX_INDEX                   0U      /* 前后方向通道 */
#define RC_CH_VY_INDEX                   1U      /* 左右方向通道 */
#define RC_CH_WZ_INDEX                   3U      /* 旋转方向通道 */

/*
 * 轴方向修正
 *
 * 例如:
 * 遥控向上，车向后，则把 RC_AXIS_VX_SIGN 改成 -1.0f。
 */
#define RC_AXIS_VX_SIGN                  (1.0f)
#define RC_AXIS_VY_SIGN                  (-1.0f)
#define RC_AXIS_WZ_SIGN                  (-1.0f)

/* 底盘最大速度限制，首次上机建议先小一点 */
#define RC_CHASSIS_MAX_VX                1000.0f
#define RC_CHASSIS_MAX_VY                1000.0f
#define RC_CHASSIS_MAX_WZ                800.0f

/* T15 摇杆死区 */
#define RC_T15_DEADZONE                  20.0f

/* 遥控掉线保护时间 */
#define RC_OFFLINE_TIMEOUT_MS            100U


/*
*********************************************************************************************************
*                                      全局变量声明
*********************************************************************************************************
*/

/*
 * 遥控原始帧队列
 *
 * 定义位置:
 * Task/rc_task.c
 *
 * 使用位置:
 * BSP/bsp_rc_uart.c 在 USART1 IDLE 中断中写入原始帧；
 * Task/rc_task.c 读取原始帧并解析。
 */
extern QueueHandle_t xRCRawDataQueue;

/*
*********************************************************************************************************
*                                      函数声明
*********************************************************************************************************
*/

Task_CreateResult_t rc_task_creat(void);
void rc_task(void *parm);

#endif
