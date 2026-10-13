#ifndef __START_TASK_H
#define __START_TASK_H
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <stdint.h>
/* 启动诊断：1设备、2mutex、3启动任务、4队列、5业务任务；失败路径不会返回。 */
extern volatile uint32_t g_startup_error;
/* 仅main/启动编排层记录失败并终止；业务模块通过返回值报告错误。 */
void START_Fail(uint32_t error_code);
void TASK_START(void);
	
#endif /* __START_TASK_H */
