#ifndef __BSP_RC_UART_H
#define __BSP_RC_UART_H

#include "main.h"
#include <stdint.h>

/*
*********************************************************************************************************
*                                      宏定义
*********************************************************************************************************
*/

/*
 * 遥控串口 DMA 单缓冲区长度。
 * 缓冲区必须大于协议帧长，否则 25 字节 SBUS 帧刚好填满缓冲区时，
 * DMA 可能先自动切换双缓冲，IDLE 中断里会读到错误的 NDTR/CT 状态。
 */
#define BSP_RC_UART_RX_BUF_LEN        50U

/* 当前 T15/SBUS 一帧有效长度 */
#define BSP_RC_UART_FRAME_LEN         25U

/* DMA 双缓冲数量 */
#define BSP_RC_UART_DMA_BUF_NUM       2U

/*
*********************************************************************************************************
*                                      函数声明
*********************************************************************************************************
*/

void BSP_RcUart_Start(void);
void BSP_RcUart_Stop(void);
void BSP_RcUart_Restart(void);

/*
 * USART1 中断回调
 * 由 Core/Src/stm32f4xx_it.c 的 USART1_IRQHandler() 调用。
 */
void USART1_IRQHandlerCallBack(void);

#endif /* __BSP_RC_UART_H */
