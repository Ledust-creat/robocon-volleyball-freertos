/************************************************************
 * 文件：rc_task.c
 * 说明：T15/SBUS 遥控解析任务
 * 层级：Task 层
 *
 * 功能：
 *   1. 从 xRCRawDataQueue 读取 USART1 DMA/IDLE 中断投递的 25 字节 SBUS 原始帧；
 *   2. 解析 T15 遥控器 10 个通道；
 *   3. 当前最小测试只使用 ch0/ch1/ch3 生成底盘 vx/vy/wz；
 *   4. 通过 xChassisCtrlQueue 把底盘目标速度发送给 chassis_task。
 *
 * 注意：
 *   1. 本文件不直接控制电机；
 *   2. 本文件不调用 MecanumWheel_Move()；
 *   3. 本文件不调用击球、抬球、达妙相关接口；
 *   4. 本文件已经包含 T15 基础 SBUS 解析，本轮测试不需要加入 jumper_t15_rc.c。
 ************************************************************/

/************************* 头文件 *************************/
#include "rc_task.h"
#include "bsp_usart.h"
#include "chassis_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/************************* 宏定义 *************************/

#define RC_TASK_PERIOD_MS               2U          /* 队列等待周期，单位 ms */

#define SBUS_FRAME_HEAD                 0x0FU       /* SBUS 帧头 */
#define SBUS_FRAME_END                  0x00U       /* 当前 T15 工程使用的 SBUS 帧尾 */
#define SBUS_CH_VALUE_OFFSET            992         /* T15 通道中值 */
#define SBUS_CH_MIN                     (-800.0f)   /* 减去中值后的最小标定值 */
#define SBUS_CH_MAX                     ( 800.0f)   /* 减去中值后的最大标定值 */

#define RC_CH_DEADZONE                  80          /* 摇杆通道死区，单位：通道值 */
#define RC_SWITCH_DEADZONE              100         /* 三档开关档位判断死区 */

#define RC_MAX_CAR_SPEED                1000.0f     /* 输出到底盘任务的最大速度命令 */

/************************ 数据类型 ************************/

typedef struct
{
    int16_t ch[10];                                 /* T15 解析后的 10 个通道，已减去 992 */
    uint8_t sw[6];                                  /* SWA/SWB/SWC/SWD/SE/SF 六个三档开关 */
} T15_RcData_t;

/************************ 变量定义 ************************/

TaskHandle_t rc_parse_task_handle = NULL;
QueueHandle_t xRCRawDataQueue = NULL;

/* 兼容旧工程可能存在的 get_remote_control_point() 调用。 */
static RC_ctrl_t s_rc_ctrl_compat;

/* 当前 T15 遥控解析结果。 */
static T15_RcData_t s_t15_rc;

/*********************** 内部函数声明 **********************/

static void rc_task(void *parm);
static bool t15_sbus_decode(const uint8_t frame[RC_FRAME_LENGTH], T15_RcData_t *rc);
static void t15_make_chassis_cmd(const T15_RcData_t *rc, Chassis_MoveCmd_t *cmd);
static uint8_t t15_detect_switch(int16_t value);
static float rc_channel_to_cmd(int16_t ch_value, bool reverse);
static float normalize_to_range(float value, float in_min, float in_max, float out_min, float out_max);
static float apply_deadzone_f32(float value, float deadzone);
static float clamp_f32(float value, float min_value, float max_value);
static void update_compat_rc(const T15_RcData_t *rc);

/************************ 函数定义 ************************/

/*
*********************************************************************************************************
*   函 数 名: get_remote_control_point
*   功能说明: 获取兼容旧工程的遥控数据指针
*   形    参: 无
*   返 回 值: RC_ctrl_t 指针
*********************************************************************************************************
*/
const RC_ctrl_t *get_remote_control_point(void)
{
    return &s_rc_ctrl_compat;
}

/*
*********************************************************************************************************
*   函 数 名: remote_control_meanum_update
*   功能说明: 遥控目标值缓变更新函数，兼容旧工程中可能存在的调用
*   形    参: input             - 当前值
*             target            - 目标值
*             up_ticks          - 加速到目标所需周期数
*             down_ticks        - 减速到目标所需周期数
*             max_speed         - 最大变化范围
*   返 回 值: 更新后的值
*********************************************************************************************************
*/
float remote_control_meanum_update(float input, float target, float up_ticks, float down_ticks, float max_speed)
{
    float delta;
    float step;

    delta = target - input;

    if (fabsf(delta) < 20.0f)
    {
        return target;
    }

    if (delta > 0.0f)
    {
        step = max_speed / up_ticks;
        if (step > delta)
        {
            step = delta;
        }
        return input + step;
    }
    else
    {
        step = max_speed / down_ticks;
        if (step > -delta)
        {
            step = -delta;
        }
        return input - step;
    }
}

/*
*********************************************************************************************************
*   函 数 名: rc_task_creat
*   功能说明: 创建遥控原始数据队列，并创建 T15 遥控解析任务
*   形    参: 无
*   返 回 值: 无
*   说    明: 必须先创建 xRCRawDataQueue，再启动 USART1 DMA 接收。
*********************************************************************************************************
*/
void rc_task_creat(void)
{
    xRCRawDataQueue = xQueueCreate(1, sizeof(uint8_t[SBUS_RX_BUF_NUM]));
    configASSERT(xRCRawDataQueue != NULL);

    /* 队列已经创建完成，此时再启动 USART1 DMA/IDLE 接收，避免中断访问空队列。 */
    remote_control_init();

    xTaskCreate((TaskFunction_t)rc_task,
                (const char *)"rc_task",
                (configSTACK_DEPTH_TYPE)RC_PARSE_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)RC_PARSE_TASK_PRIO,
                (TaskHandle_t *)&rc_parse_task_handle);
}

/*
*********************************************************************************************************
*   函 数 名: rc_task
*   功能说明: T15 遥控解析任务主体
*   形    参: parm              - FreeRTOS 任务参数，未使用
*   返 回 值: 无
*   主要流程: 1. 等待 USART1 中断投递原始 SBUS 帧；
*             2. 校验并解析 T15 通道；
*             3. 生成底盘速度命令；
*             4. 写入 xChassisCtrlQueue。
*********************************************************************************************************
*/
static void rc_task(void *parm)
{
    uint8_t raw_frame[SBUS_RX_BUF_NUM] = {0};
    Chassis_MoveCmd_t chassis_cmd = {0};

    (void)parm;

    for (;;)
    {
        if (xQueueReceive(xRCRawDataQueue, raw_frame, pdMS_TO_TICKS(RC_TASK_PERIOD_MS)) == pdTRUE)
        {
			HAL_GPIO_TogglePin(led2_GPIO_Port,led2_Pin);
            if (t15_sbus_decode(raw_frame, &s_t15_rc) == true)
            {
                update_compat_rc(&s_t15_rc);
                t15_make_chassis_cmd(&s_t15_rc, &chassis_cmd);

                /* 底盘目标速度只需要最新值，队列长度为 1，使用覆盖写。 */
                if (xChassisCtrlQueue != NULL)
                {
                    xQueueOverwrite(xChassisCtrlQueue, &chassis_cmd);
                }
            }
        }
    }
}

/*
*********************************************************************************************************
*   函 数 名: t15_sbus_decode
*   功能说明: 解析 T15 的 25 字节 SBUS 原始帧
*   形    参: frame             - 25 字节 SBUS 原始帧
*             rc                - 解析结果输出
*   返 回 值: true              - 帧有效并解析成功
*             false             - 参数错误或帧头帧尾错误
*********************************************************************************************************
*/
static bool t15_sbus_decode(const uint8_t frame[RC_FRAME_LENGTH], T15_RcData_t *rc)
{
    uint8_t i;

    if ((frame == NULL) || (rc == NULL))
    {
        return false;
    }

    if ((frame[0] != SBUS_FRAME_HEAD) || (frame[24] != SBUS_FRAME_END))
    {
        return false;
    }

    /* 按 SBUS 11bit 通道格式解析，保留与裸机 jumper_t15_rc.c 相同的拆包顺序。 */
    rc->ch[0] = (int16_t)(((frame[1])       | (frame[2] << 8))                    & 0x07FF);
    rc->ch[1] = (int16_t)(((frame[2] >> 3)  | (frame[3] << 5))                    & 0x07FF);
    rc->ch[2] = (int16_t)(((frame[3] >> 6)  | (frame[4] << 2) | (frame[5] << 10)) & 0x07FF);
    rc->ch[3] = (int16_t)(((frame[5] >> 1)  | (frame[6] << 7))                    & 0x07FF);
    rc->ch[4] = (int16_t)(((frame[6] >> 4)  | (frame[7] << 4))                    & 0x07FF);
    rc->ch[5] = (int16_t)(((frame[7] >> 7)  | (frame[8] << 1) | (frame[9] << 9))  & 0x07FF);
    rc->ch[6] = (int16_t)(((frame[9] >> 2)  | (frame[10] << 6))                   & 0x07FF);
    rc->ch[7] = (int16_t)(((frame[10] >> 5) | (frame[11] << 3))                   & 0x07FF);
    rc->ch[8] = (int16_t)(((frame[12])      | (frame[13] << 8))                   & 0x07FF);
    rc->ch[9] = (int16_t)(((frame[13] >> 3) | (frame[14] << 5))                   & 0x07FF);

    for (i = 0U; i < 10U; i++)
    {
        rc->ch[i] = (int16_t)(rc->ch[i] - SBUS_CH_VALUE_OFFSET);
    }

    rc->sw[0] = t15_detect_switch(rc->ch[4]);     /* SWA */
    rc->sw[1] = t15_detect_switch(rc->ch[5]);     /* SWB */
    rc->sw[2] = t15_detect_switch(rc->ch[6]);     /* SWC */
    rc->sw[3] = t15_detect_switch(rc->ch[7]);     /* SWD */
    rc->sw[4] = t15_detect_switch(rc->ch[8]);     /* SE  */
    rc->sw[5] = t15_detect_switch(rc->ch[9]);     /* SF  */

    return true;
}

/*
*********************************************************************************************************
*   函 数 名: t15_make_chassis_cmd
*   功能说明: 将 T15 摇杆通道转换为底盘速度命令
*   形    参: rc                - T15 解析结果
*             cmd               - 底盘速度命令输出
*   返 回 值: 无
*   说    明: 映射关系沿用裸机：ch1->x，ch0取反->y，ch3取反->w。
*********************************************************************************************************
*/
static void t15_make_chassis_cmd(const T15_RcData_t *rc, Chassis_MoveCmd_t *cmd)
{
    if ((rc == NULL) || (cmd == NULL))
    {
        return;
    }

    cmd->vx = rc_channel_to_cmd(rc->ch[0], false);
    cmd->vy = rc_channel_to_cmd(rc->ch[1], true);
    cmd->wz = rc_channel_to_cmd(rc->ch[3], true);
}

/*
*********************************************************************************************************
*   函 数 名: t15_detect_switch
*   功能说明: 根据 T15 三档开关通道值判断档位
*   形    参: value             - 已减去中值后的通道值
*   返 回 值: RC_SW_UP / RC_SW_MID / RC_SW_DOWN
*********************************************************************************************************
*/
static uint8_t t15_detect_switch(int16_t value)
{
    if (value <= (int16_t)(SBUS_CH_MIN + RC_SWITCH_DEADZONE))
    {
        return RC_SW_UP;
    }

    if (value >= (int16_t)(SBUS_CH_MAX - RC_SWITCH_DEADZONE))
    {
        return RC_SW_DOWN;
    }

    return RC_SW_MID;
}

/*
*********************************************************************************************************
*   函 数 名: rc_channel_to_cmd
*   功能说明: 将单个摇杆通道值转换为底盘命令量
*   形    参: ch_value          - 已减去中值后的 SBUS 通道值
*             reverse           - 是否取反
*   返 回 值: 底盘速度命令
*********************************************************************************************************
*/
static float rc_channel_to_cmd(int16_t ch_value, bool reverse)
{
    float value;
    float cmd;

    value = (float)ch_value;

    value = apply_deadzone_f32(value, (float)RC_CH_DEADZONE);
    value = clamp_f32(value, SBUS_CH_MIN, SBUS_CH_MAX);
		
    cmd = normalize_to_range(value, SBUS_CH_MIN, SBUS_CH_MAX, -RC_MAX_CAR_SPEED, RC_MAX_CAR_SPEED);
	
    if (reverse == true)
    {
        cmd = -cmd;
    }

    return cmd;
}

/*
*********************************************************************************************************
*   函 数 名: normalize_to_range
*   功能说明: 将输入值从一个范围线性映射到另一个范围
*   形    参: value             - 输入值
*             in_min/in_max     - 输入范围
*             out_min/out_max   - 输出范围
*   返 回 值: 映射后的值
*********************************************************************************************************
*/
static float normalize_to_range(float value, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min)
    {
        return 0.0f;
    }

    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
*********************************************************************************************************
*   函 数 名: apply_deadzone_f32
*   功能说明: 对浮点值应用死区
*   形    参: value             - 输入值
*             deadzone          - 死区范围
*   返 回 值: 死区处理后的值
*********************************************************************************************************
*/
static float apply_deadzone_f32(float value, float deadzone)
{
    if ((value > -deadzone) && (value < deadzone))
    {
        return 0.0f;
    }

    return value;
}

/*
*********************************************************************************************************
*   函 数 名: clamp_f32
*   功能说明: 限制浮点数范围
*   形    参: value             - 输入值
*             min_value         - 最小值
*             max_value         - 最大值
*   返 回 值: 限幅后的值
*********************************************************************************************************
*/
static float clamp_f32(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/*
*********************************************************************************************************
*   函 数 名: update_compat_rc
*   功能说明: 更新兼容旧工程 RC_ctrl_t 的遥控数据
*   形    参: rc                - T15 解析结果
*   返 回 值: 无
*********************************************************************************************************
*/
static void update_compat_rc(const T15_RcData_t *rc)
{
    uint8_t i;

    if (rc == NULL)
    {
        return;
    }

    memset(&s_rc_ctrl_compat, 0, sizeof(s_rc_ctrl_compat));

    for (i = 0U; i < 5U; i++)
    {
        s_rc_ctrl_compat.rc.ch[i] = rc->ch[i];
    }

    s_rc_ctrl_compat.rc.s[0] = rc->sw[0];
    s_rc_ctrl_compat.rc.s[1] = rc->sw[1];
}
