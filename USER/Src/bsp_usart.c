/************************************************************
 * 文件：bsp_usart.c
 * 说明：USART1 遥控接收底层
 * 层级：BSP 层
 *
 * 功能：
 *   1. 配置 USART1 RX DMA 双缓冲接收；
 *   2. 使用 USART1 IDLE 空闲中断判断一帧 SBUS 接收完成；
 *   3. 只把 25 字节 T15/SBUS 原始帧投递给 xRCRawDataQueue；
 *   4. 不在中断中解析协议，不在中断中控制电机。
 ************************************************************/

/************************* 头文件 *************************/
#include "bsp_usart.h"
#include "rc_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stdint.h>

/************************ 外部变量 ************************/

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern QueueHandle_t xRCRawDataQueue;

/************************ 变量定义 ************************/

static uint8_t s_sbus_rx_buf[2][SBUS_RX_BUF_NUM];       /* USART1 DMA 双缓冲 */

/*********************** 内部函数声明 **********************/

static void usart1_dma_switch_buffer(uint8_t **rx_buf, uint16_t *rx_len);

/************************ 函数定义 ************************/

/*
*********************************************************************************************************
*   函 数 名: remote_control_init
*   功能说明: 启动遥控器 USART1 DMA 接收
*   形    参: 无
*   返 回 值: 无
*   说    明: 必须在 xRCRawDataQueue 创建完成后调用。
*********************************************************************************************************
*/
void remote_control_init(void)
{
    RC_init();
}

/*
*********************************************************************************************************
*   函 数 名: USART1_IRQHandlerCallBack
*   功能说明: USART1 中断回调，处理 SBUS 空闲中断并投递原始帧
*   形    参: 无
*   返 回 值: 无
*   说    明: 该函数应由 USART1_IRQHandler() 中调用。
*********************************************************************************************************
*/
void USART1_IRQHandlerCallBack(void)
{
    uint16_t this_time_rx_len = 0U;
    uint8_t *p_rx_buf = NULL;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* 清除 IDLE 标志，避免反复进入空闲中断。 */
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        /* 切换 DMA 双缓冲，并获得刚接收完成的缓冲区和长度。 */
        usart1_dma_switch_buffer(&p_rx_buf, &this_time_rx_len);

        /* 当前最小测试只接收完整 25 字节 T15/SBUS 帧。 */
        if ((this_time_rx_len == RC_FRAME_LENGTH) &&
            (p_rx_buf != NULL) &&
            (xRCRawDataQueue != NULL))
        {
            xQueueOverwriteFromISR(xRCRawDataQueue,
                                   p_rx_buf,
                                   &xHigherPriorityTaskWoken);

            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    /* RXNE/错误标志兜底清除，实际数据由 DMA 搬运。 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
}

/*
*********************************************************************************************************
*   函 数 名: RC_init
*   功能说明: 初始化 USART1 DMA 双缓冲接收并打开 IDLE 中断
*   形    参: 无
*   返 回 值: 无
*   说    明: USART1 参数仍由 CubeMX 生成代码配置，本函数只接管 DMA 双缓冲接收。
*********************************************************************************************************
*/
void RC_init(void)
{
    /* 允许 USART1 RX 使用 DMA。 */
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    /* 打开 USART1 空闲中断，用于判断 SBUS 一帧接收结束。 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    /* 修改 DMA 配置前必须先关闭 DMA。 */
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while ((hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) != RESET)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    /* 外设地址：USART1 数据寄存器。 */
    hdma_usart1_rx.Instance->PAR = (uint32_t)&(USART1->DR);

    /* DMA 双缓冲地址。 */
    hdma_usart1_rx.Instance->M0AR = (uint32_t)s_sbus_rx_buf[0];
    hdma_usart1_rx.Instance->M1AR = (uint32_t)s_sbus_rx_buf[1];

    /* 单个 DMA 缓冲区长度。 */
    hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

    /* 从 Memory0 开始接收。 */
    hdma_usart1_rx.Instance->CR &= ~DMA_SxCR_CT;//

    /* 打开 DMA 双缓冲模式。 */
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    /* 启动 DMA。 */
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

/*
*********************************************************************************************************
*   函 数 名: RC_unable
*   功能说明: 关闭 USART1 遥控 DMA 接收
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void RC_unable(void)
{
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
}

/*
*********************************************************************************************************
*   函 数 名: RC_restart
*   功能说明: 重启 USART1 遥控 DMA 接收
*   形    参: dma_buf_num       - 保留形参，当前固定使用 SBUS_RX_BUF_NUM
*   返 回 值: 无
*********************************************************************************************************
*/
void RC_restart(uint16_t dma_buf_num)
{
    (void)dma_buf_num;

    RC_unable();
    RC_init();
}

/*
*********************************************************************************************************
*   函 数 名: usart1_dma_switch_buffer
*   功能说明: 在 USART1 IDLE 中断中切换 DMA 双缓冲，并返回刚接收完成的缓冲区
*   形    参: rx_buf            - 输出：刚接收完成的数据缓冲区指针
*             rx_len            - 输出：本次接收长度
*   返 回 值: 无
*********************************************************************************************************
*/
static void usart1_dma_switch_buffer(uint8_t **rx_buf, uint16_t *rx_len)
{
    if ((rx_buf == NULL) || (rx_len == NULL))
    {
        return;
    }

    *rx_buf = NULL;
    *rx_len = 0U;

    if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
        /* 当前 DMA 正在使用 Memory0，本次完成的数据位于 Memory0。 */
        __HAL_DMA_DISABLE(&hdma_usart1_rx);

        *rx_len = (uint16_t)(SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR);
        hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

        /* 下一帧切换到 Memory1。 */
        hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

        __HAL_DMA_ENABLE(&hdma_usart1_rx);

        *rx_buf = s_sbus_rx_buf[0];
    }
    else
    {
        /* 当前 DMA 正在使用 Memory1，本次完成的数据位于 Memory1。 */
        __HAL_DMA_DISABLE(&hdma_usart1_rx);

        *rx_len = (uint16_t)(SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR);
        hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

        /* 下一帧切换到 Memory0。 */
        hdma_usart1_rx.Instance->CR &= ~DMA_SxCR_CT;

        __HAL_DMA_ENABLE(&hdma_usart1_rx);

        *rx_buf = s_sbus_rx_buf[1];
    }
}
