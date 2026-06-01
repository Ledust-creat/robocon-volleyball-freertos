#ifndef __BSP_USART_H
#define __BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"

/************************* 宏定义 *************************/

#ifndef RC_FRAME_LENGTH
#define RC_FRAME_LENGTH                 25U     /* T15/SBUS 一帧有效长度 */
#endif

#ifndef SBUS_RX_BUF_NUM
#define SBUS_RX_BUF_NUM                 50U     /* USART1 DMA 单缓冲长度 */
#endif

/************************ 函数声明 ************************/

void remote_control_init(void);
void RC_init(void);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);
void USART1_IRQHandlerCallBack(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H */
