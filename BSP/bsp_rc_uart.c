#include "bsp_rc_uart.h"
#include "usart.h"
#include "dma.h"
#include "rc_task.h"

#include "FreeRTOS.h"
#include "queue.h"

/*
*********************************************************************************************************
*                                      外部变量
*********************************************************************************************************
*/

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/*
 * 遥控原始帧队列。
 *
 * 注意：
 * 1. 该队列由 rc_task.c 创建。
 * 2. BSP_RcUart_Start() 必须在 xRCRawDataQueue 创建之后调用。
 * 3. 如果队列尚未创建，USART1 中断会直接丢弃本帧，避免 HardFault。
 */
extern QueueHandle_t xRCRawDataQueue;

/*
*********************************************************************************************************
*                                      模块变量
*********************************************************************************************************
*/

static uint8_t s_rc_uart_rx_buf[BSP_RC_UART_DMA_BUF_NUM][BSP_RC_UART_RX_BUF_LEN];

/*
*********************************************************************************************************
*   函 数 名: BSP_RcUart_Start
*   功能说明: 启动遥控器 USART1 DMA 双缓冲接收
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_RcUart_Start(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while ((hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) != 0U)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR  = (uint32_t)&huart1.Instance->DR;
    hdma_usart1_rx.Instance->M0AR = (uint32_t)s_rc_uart_rx_buf[0];
    hdma_usart1_rx.Instance->M1AR = (uint32_t)s_rc_uart_rx_buf[1];
    hdma_usart1_rx.Instance->NDTR = BSP_RC_UART_RX_BUF_LEN;

    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
    hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_rx));
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart1_rx));
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_usart1_rx));
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_usart1_rx));
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_usart1_rx));

    __HAL_DMA_ENABLE(&hdma_usart1_rx);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_RcUart_Stop
*   功能说明: 停止遥控器 USART1 DMA 接收
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_RcUart_Stop(void)
{
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
}

/*
*********************************************************************************************************
*   函 数 名: BSP_RcUart_Restart
*   功能说明: 重新启动遥控器 USART1 DMA 接收
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void BSP_RcUart_Restart(void)
{
    BSP_RcUart_Stop();
    BSP_RcUart_Start();
}

/*
*********************************************************************************************************
*   函 数 名: USART1_IRQHandlerCallBack
*   功能说明: USART1 空闲中断回调，收到完整遥控帧后写入 FreeRTOS 队列
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void USART1_IRQHandlerCallBack(void)
{
    uint16_t rx_len = 0U;
    uint8_t *rx_buf = NULL;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /*
     * 优先处理 IDLE。
     * DMA 接收时 RXNE/IDLE 可能同时出现；如果先因 RXNE 返回，会丢掉完整 SBUS 帧。
     * 没有 IDLE 时，仅清理 RXNE/PE 等异常标志。
     */
    if ((huart1.Instance->SR & UART_FLAG_IDLE) == 0U)
    {
        if ((huart1.Instance->SR & UART_FLAG_RXNE) != 0U)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart1);
        }
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(&huart1);

    /*
     * 判断当前 DMA 正在使用哪个缓冲区。
     *
     * CT = 0：当前目标为 Memory0，本次取 Memory0。
     * CT = 1：当前目标为 Memory1，本次取 Memory1。
     *
     * 这里沿用原工程的 IDLE 双缓冲切换方式，仅修正命名和队列判空。
     */
    if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);

        rx_len = BSP_RC_UART_RX_BUF_LEN - hdma_usart1_rx.Instance->NDTR;
        rx_buf = s_rc_uart_rx_buf[0];

        hdma_usart1_rx.Instance->NDTR = BSP_RC_UART_RX_BUF_LEN;
        hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

        __HAL_DMA_ENABLE(&hdma_usart1_rx);
    }
    else
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);

        rx_len = BSP_RC_UART_RX_BUF_LEN - hdma_usart1_rx.Instance->NDTR;
        rx_buf = s_rc_uart_rx_buf[1];

        hdma_usart1_rx.Instance->NDTR = BSP_RC_UART_RX_BUF_LEN;
        hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);

        __HAL_DMA_ENABLE(&hdma_usart1_rx);
    }

    /*
     * 当前只接收 T15/SBUS 25 字节定长帧。
     * 队列未创建时直接丢弃，避免中断访问空队列。
     */
    if ((rx_len == BSP_RC_UART_FRAME_LEN) && (rx_buf != NULL) && (xRCRawDataQueue != NULL))
    {
        xQueueOverwriteFromISR(xRCRawDataQueue,
                               rx_buf,
                               &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
