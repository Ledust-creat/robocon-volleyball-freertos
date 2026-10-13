#ifndef __INITIALIZE_H__
#define __INITIALIZE_H__
#include <stdbool.h>
/* 设备/CAN初始化结果；由main决定失败处理，不依赖启动模块。 */
bool All_Init(void);
void All_Clear(void);

#endif
