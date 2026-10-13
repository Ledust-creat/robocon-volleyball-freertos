
#include <stdbool.h>
#include "rc_task.h"
#include "bsp_rc_uart.h"
#include "chassis_task.h"
#include "hit_task.h"
#include "dve_t15.h"
#include "math_utils.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/*
*********************************************************************************************************
*                                      全局变量
*********************************************************************************************************
*/

/* 遥控原始帧队列，由 rc_task 创建，由 bsp_rc_uart 中断写入 */
QueueHandle_t xRCRawDataQueue = NULL;

/* 遥控任务句柄 */
static TaskHandle_t s_rc_task_handle = NULL;
// 上电及失控后均等待安全拨杆确认；覆盖队列中只发布当前最新命令。
static bool s_rc_recovery_wait = true;

/*
*********************************************************************************************************
*                                      内部函数声明
*********************************************************************************************************
*/

static void RC_T15ToChassisCmd(const DVE_T15_Data_t *t15, Chassis_MoveCmd_t *cmd);
static void RC_SetChassisStopCmd(Chassis_MoveCmd_t *cmd);

static void RC_T15ToHitCmd(const DVE_T15_Data_t *t15, Hit_Cmd_t *cmd);
static void RC_SetHitStopCmd(Hit_Cmd_t *cmd);
static void RC_ProcessFrame(const uint8_t *frame, bool received,
                            Chassis_MoveCmd_t *chassis_cmd, Hit_Cmd_t *hit_cmd);

/*
*********************************************************************************************************
*   函 数 名: rc_task_creat
*   功能说明: 创建本模块命令队列与业务任务，并向调用者报告结果
*   形    参: 无；仅系统启动阶段调用一次
*   返 回 值: OK表示成功；ERR_QUEUE表示队列失败；ERR_TASK表示任务失败且队列已释放
*   说    明: 调用者保证创建阶段不被业务任务抢占；本模块不依赖启动模块或决定整机故障策略。
*********************************************************************************************************
*/
Task_CreateResult_t rc_task_creat(void)
{
    xRCRawDataQueue = xQueueCreate(1, sizeof(uint8_t[BSP_RC_UART_RX_BUF_LEN]));
    if (xRCRawDataQueue == NULL) return TASK_CREATE_ERR_QUEUE;

    /*
     * 启动遥控 USART1 DMA 接收。
     * BSP_RcUart 收到完整帧后，会直接用 xQueueOverwriteFromISR()
     * 写入 xRCRawDataQueue。
     */


    if (xTaskCreate((TaskFunction_t)rc_task,
                (const char *)"rc_task",
                (configSTACK_DEPTH_TYPE)RC_PARSE_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)RC_PARSE_TASK_PRIO,
                (TaskHandle_t *)&s_rc_task_handle) != pdPASS)
    {
        // 创建失败时只释放本模块刚分配的队列，不调用上层启动或故障处理函数。
        vQueueDelete(xRCRawDataQueue);
        xRCRawDataQueue = NULL;
        return TASK_CREATE_ERR_TASK;
    }
    // 原始帧队列和解析任务创建成功后才打开DMA/IDLE，避免ISR访问未就绪资源。
    BSP_RcUart_Start();
    return TASK_CREATE_OK;
}

/*
*********************************************************************************************************
*   函 数 名: rc_task
*   功能说明: 遥控任务，解析 T15/SBUS 原始帧并发送底盘速度命令
*   形    参: parm - FreeRTOS 任务参数，未使用
*   返 回 值: 无
*********************************************************************************************************
*/
void rc_task(void *parm)
{
    uint8_t raw_frame[BSP_RC_UART_RX_BUF_LEN] = {0};
    Chassis_MoveCmd_t chassis_cmd;
	Hit_Cmd_t hit_cmd;
    (void)parm;
	
    RC_SetChassisStopCmd(&chassis_cmd);
	RC_SetHitStopCmd(&hit_cmd);
    for (;;)
    {
        /*
         * 等待遥控原始帧。
         * 超时没有收到新帧，则认为遥控掉线，发送停机命令。
         */
        bool received = (xQueueReceive(xRCRawDataQueue, raw_frame,
                         pdMS_TO_TICKS(RC_OFFLINE_TIMEOUT_MS)) == pdTRUE);
        RC_ProcessFrame(raw_frame, received, &chassis_cmd, &hit_cmd);

        /*
         * 直接使用 FreeRTOS 队列发送底盘命令，不额外封装发送函数。
         */
        if (xChassisCtrlQueue != NULL)
        {
            xQueueOverwrite(xChassisCtrlQueue, &chassis_cmd);
        }
		if (xHitCmdQueue != NULL)
		{
			xQueueOverwrite(xHitCmdQueue, &hit_cmd);
		}
        /*
         * 没有串口打印时，可用 led1 判断遥控任务是否在运行。
         * 如果不需要 LED 调试，可以删除这一句。
         */
#ifdef led1_GPIO_Port
        HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
#endif
    }
}

/*
*********************************************************************************************************
*                                      内部函数定义
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*   函 数 名: RC_T15ToChassisCmd
*   功能说明: 将 T15 通道值转换成底盘速度命令
*   形    参: t15 - T15 解析结果
*             cmd - 底盘速度命令
*   返 回 值: 无
*********************************************************************************************************
*/
static void RC_T15ToChassisCmd(const DVE_T15_Data_t *t15, Chassis_MoveCmd_t *cmd)
{
    float vx_ch;
    float vy_ch;
    float wz_ch;

    if ((t15 == NULL) || (cmd == NULL) || (t15->frame_valid == 0U))
    {
        return;
    }

    /*
     * 通道映射与方向修正在 rc_task.h 中配置。
     * 如果上下/左右反了，只改 RC_CH_xxx_INDEX；
     * 如果方向反了，只改 RC_AXIS_xxx_SIGN。
     */
    cmd->enable = 1U;
    vx_ch = RC_AXIS_VX_SIGN * (float)t15->ch[RC_CH_VX_INDEX];
    vy_ch = RC_AXIS_VY_SIGN * (float)t15->ch[RC_CH_VY_INDEX];
    wz_ch = RC_AXIS_WZ_SIGN * (float)t15->ch[RC_CH_WZ_INDEX];

    vx_ch = Math_ApplyDeadzoneFloat(vx_ch, RC_T15_DEADZONE);
    vy_ch = Math_ApplyDeadzoneFloat(vy_ch, RC_T15_DEADZONE);
    wz_ch = Math_ApplyDeadzoneFloat(wz_ch, RC_T15_DEADZONE);

    /*
     * T15 通道满量程约为 ±660 ~ ±800。
     * 这里先按 660 做归一化，避免最大速度过大。
     */
    cmd->vx = Math_ClampFloat((vx_ch / 660.0f) * RC_CHASSIS_MAX_VX,
                              -RC_CHASSIS_MAX_VX,
                               RC_CHASSIS_MAX_VX);

    cmd->vy = Math_ClampFloat((vy_ch / 660.0f) * RC_CHASSIS_MAX_VY,
                              -RC_CHASSIS_MAX_VY,
                               RC_CHASSIS_MAX_VY);

    cmd->wz = Math_ClampFloat((wz_ch / 660.0f) * RC_CHASSIS_MAX_WZ,
                              -RC_CHASSIS_MAX_WZ,
                               RC_CHASSIS_MAX_WZ);

}

/*
*********************************************************************************************************
*   函 数 名: RC_SetChassisStopCmd
*   功能说明: 设置底盘停机命令
*   形    参: cmd - 底盘速度命令
*   返 回 值: 无
*********************************************************************************************************
*/
static void RC_SetChassisStopCmd(Chassis_MoveCmd_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->enable = 0U;
    cmd->vx = 0.0f;
    cmd->vy = 0.0f;
    cmd->wz = 0.0f;
}
/*
*********************************************************************************************************
*   函 数 名: RC_T15ToHitCmd
*   功能说明: 将 T15 拨杆状态转换成击球任务命令
*   形    参: t15 - T15 解析结果
*             cmd - 击球命令
*   返 回 值: 无
*********************************************************************************************************
*/
static void RC_T15ToHitCmd(const DVE_T15_Data_t *t15, Hit_Cmd_t *cmd)
{
    if ((t15 == NULL) || (cmd == NULL) || (t15->frame_valid == 0U))
    {
        return;
    }

    cmd->enable = 1U;
    // 只在有效遥控帧中生成安全拨杆确认，失控帧不会进入本映射。
    cmd->safe_to_arm = ((t15->sf == DVE_T15_SWITCH_UP) &&
                        (t15->swd != DVE_T15_SWITCH_DOWN)) ? 1U : 0U;

    /* SWC 三档选择击球预设角度 */
    switch (t15->swc)
    {
    case DVE_T15_SWITCH_UP:
        cmd->preset_index = 0U;
        break;

    case DVE_T15_SWITCH_MID:
        cmd->preset_index = 1U;
        break;

    case DVE_T15_SWITCH_DOWN:
        cmd->preset_index = 2U;
        break;

    default:
        cmd->preset_index = 1U;
        break;
    }

    /* SF 按下进入击球动作位，松开回准备位 */
    cmd->hit_pressed = (t15->sf == DVE_T15_SWITCH_DOWN) ? 1U : 0U;
	
    /* SWD_DOWN 控制 0x205 俯仰/机械臂速度 */
    if (t15->swd == DVE_T15_SWITCH_DOWN)
    {
        cmd->pitch_speed_set = HIT_PITCH_SPEED_SET;
    }
    else
    {
        cmd->pitch_speed_set = HIT_PITCH_SPEED_ZERO;
    }
}

/*
*********************************************************************************************************
*   函 数 名: RC_SetHitStopCmd
*   功能说明: 设置击球安全停机命令
*   形    参: cmd - 击球命令
*   返 回 值: 无
*********************************************************************************************************
*/
static void RC_SetHitStopCmd(Hit_Cmd_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->enable = 0U;
    cmd->safe_to_arm = 0U;
    cmd->preset_index = 1U;
    cmd->hit_pressed = 0U;
    cmd->pitch_speed_set = HIT_PITCH_SPEED_ZERO;
}

/*
*********************************************************************************************************
*   函 数 名: RC_ProcessFrame
*   功能说明: 解析一帧遥控输入并执行失控停机和恢复确认
*   形    参: frame - 原始SBUS帧；received - 是否收到帧；chassis_cmd/hit_cmd - 输出命令
*   返 回 值: 无；坏帧、失控标志或接收超时均输出停机命令
*   说    明: 恢复要求SF松开且SWD关闭；确认帧只给击打任务safe_to_arm，不使能输出。
*********************************************************************************************************
*/
static void RC_ProcessFrame(const uint8_t *frame, bool received,
                            Chassis_MoveCmd_t *chassis_cmd, Hit_Cmd_t *hit_cmd)
{
    DVE_T15_Data_t t15_data;
    if (received &&
        (DVE_T15_ParseFrame(frame, DVE_T15_FRAME_LEN, &t15_data) != 0U))
    {
        if (s_rc_recovery_wait)
        {
            // 首个安全帧只确认恢复，enable仍为0；后续新帧才发布可执行目标。
            RC_SetChassisStopCmd(chassis_cmd);
            RC_SetHitStopCmd(hit_cmd);
            if ((t15_data.sf == DVE_T15_SWITCH_UP) &&
                (t15_data.swd != DVE_T15_SWITCH_DOWN))
            {
                s_rc_recovery_wait = false;
                hit_cmd->safe_to_arm = 1U;
            }
        }
        else
        {
            RC_T15ToChassisCmd(&t15_data, chassis_cmd);
            RC_T15ToHitCmd(&t15_data, hit_cmd);
        }
    }
    else
    {
        s_rc_recovery_wait = true;
        RC_SetChassisStopCmd(chassis_cmd);
        RC_SetHitStopCmd(hit_cmd);
    }

}
