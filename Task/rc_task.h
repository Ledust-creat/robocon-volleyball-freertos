#ifndef __RC_TASK_H
#define __RC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "chassis_task.h"

/************************* 宏定义 *************************/

#define RC_FRAME_LENGTH                 25U     /* T15/SBUS 一帧有效长度 */
#define SBUS_RX_BUF_NUM                 50U     /* USART1 DMA 单缓冲长度 */

#define DBUS_MAX_LEN                    SBUS_RX_BUF_NUM
#define DBUS_BUFLEN                     RC_FRAME_LENGTH
#define DBUS_HUART                      huart1

#define RC_PARSE_TASK_PRIO              24U
#define RC_PARSE_TASK_SIZE              256U

#define RC_SW_UP                        1U
#define RC_SW_MID                       3U
#define RC_SW_DOWN                      2U

#define switch_is_down(s)               ((s) == RC_SW_DOWN)
#define switch_is_mid(s)                ((s) == RC_SW_MID)
#define switch_is_up(s)                 ((s) == RC_SW_UP)

/************************ 数据类型 ************************/

typedef struct
{
    struct
    {
        int16_t ch[5];
        uint8_t s[2];
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct
    {
        uint16_t v;
    } key;
} RC_ctrl_t;

/************************ 变量声明 ************************/

extern QueueHandle_t xRCRawDataQueue;
extern TaskHandle_t rc_parse_task_handle;

/************************ 函数声明 ************************/

void rc_task_creat(void);
const RC_ctrl_t *get_remote_control_point(void);
float remote_control_meanum_update(float input, float target, float up_ticks, float down_ticks, float max_speed);

#ifdef __cplusplus
}
#endif

#endif /* __RC_TASK_H */
