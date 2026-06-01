#include "bsp_can.h"
#include "dve_dji_motor.h"
#include "dve_dm_motor.h"

/*
*********************************************************************************************************
*   文件说明
*********************************************************************************************************
* 文件名称：bsp_can_irq.c
* 功能定位：CAN 接收中断统一分发文件。
*/

/*
*********************************************************************************************************
*   函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
*   功能说明: HAL CAN FIFO0 接收中断回调函数
*   形    参: hcan - 触发 FIFO0 消息挂起中断的 CAN 句柄
*   返 回 值: 无
*   运行位置: 中断上下文
*   说    明:
*             1. 本函数在 CAN 接收中断中执行，不能做阻塞操作；
*             2. 不要在这里做 PID、状态机、串口 printf 等耗时逻辑；
*             3. 使用 while 清空 FIFO0，避免短时间多帧进入后只处理一帧；
*             4. DVE 层回调只更新内部反馈缓存，任务层周期读取最新值即可。
*********************************************************************************************************
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan == NULL)
    {
        return;
    }

    /*
     * FIFO0 里可能一次积累多帧。
     * 中断触发后尽量一次性读空，减少后续中断压力。
     */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            return;
        }

        if (hcan == &hcan2)
        {
            /* CAN2：当前接 DJI 3508/C620。解析 0x201~0x205 反馈。 */
            (void)DVE_DjiMotor_RxCallback(hcan, &rx_header, rx_data);
        }
        else if (hcan == &hcan1)
        {
            /* CAN1：预留 DM 达妙。当前主发球机器人暂时可以不使用。 */
            (void)DVE_DmMotor_RxCallback(hcan, &rx_header, rx_data);
        }
        else
        {
            /* 未知 CAN 句柄，不处理。 */
        }
    }
}
