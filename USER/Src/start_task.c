/*  Copyright (s) FOSU
 *  All rights reserved
 * 
 * 文件名称：start_task.c
 * 摘要：FreeRTOS启动任务，负责创建底盘控制、遥控解析等核心业务任务，完成初始化后自删除
 *  
 * 修改历史     版本号        Author       修改内容
 *--------------------------------------------------
 * 2026.01.20      v01         FTW      创建文件
 *--------------------------------------------------
*/
#include "start_task.h"

#include "chassis_task.h"
#include "hit_task.h"
#include "rc_task.h"
#include "bsp_can.h"
#include "dve_dji_motor.h"

// 启动错误码：1设备，2mutex，3启动任务，4队列，5业务任务；仅编排层写入。
volatile uint32_t g_startup_error = 0U;

/*
*********************************************************************************************************
*   函 数 名: START_Fail
*   功能说明: 记录不可恢复的启动失败，关闭业务输出并进入错误处理
*   形    参: error_code - 启动错误码1~5
*   返 回 值: 不返回
*   说    明: 零电流发送为尽力提交；调度器/mutex/总线未就绪时可能失败，不等于电机已停。
*********************************************************************************************************
*/
void START_Fail(uint32_t error_code)
{
    g_startup_error = error_code;
    (void)DVE_DjiMotor_ClearAllCurrent();
    (void)DVE_DjiMotor_SendAllCurrent();
    Error_Handler();
    for (;;) { }
}

/* 启动任务配置参数 */
#define START_TASK_SIZE 128 	 // 任务堆栈大小
#define START_TASK_PRIO 28    	 // 启动优先级28，高于RC27/底盘26/击打25；正常创建阶段不阻塞或主动让出CPU

TaskHandle_t start_Task_Handle;  // 启动任务句柄：用于创建任务和自删除操作

static void START_CheckCreate(Task_CreateResult_t result)
{
    // 业务模块报告失败原因，整机终止策略与诊断码映射只由启动编排层决定。
    if (result != TASK_CREATE_OK)
        START_Fail((result == TASK_CREATE_ERR_QUEUE) ? 4U : 5U);
}

/*
*********************************************************************************************************
*   函 数 名: start_task
*   功能说明: 初始化CAN发送同步并创建底盘、击打、遥控业务资源
*   形    参: parm - 未使用的任务参数
*   返 回 值: 无；全部创建成功后删除自身，失败由本编排层统一处理。
*   说    明: 优先级28高于所有业务任务；正常创建路径不阻塞、不主动让出CPU。
*********************************************************************************************************
*/
void start_task(void *parm)
{
    (void)parm;
    if (BSP_CAN_InitTxSync() != BSP_CAN_OK) START_Fail(2U);

	/*  创建底盘控制任务：初始化底盘相关队列和控制任务 */
	START_CheckCreate(chassis_task_creat());
	/*  创建击打任务：初始化击打命令队列和0x1FF组控制任务 */
	START_CheckCreate(hit_task_creat());
	/*  创建遥控解析任务：初始化遥控队列和解析任务 */
	START_CheckCreate(rc_task_creat());
    // 正常创建路径保持最高业务优先级，所有资源成功后删除自身，业务任务才开始运行。
	
	/*  删除自身任务：启动任务仅做初始化，完成后释放资源（也可传NULL删除自身） */
	vTaskDelete(start_Task_Handle);
	
	

}
/**********************************************************************
 * 函数名称： TASK_START
 * 功能描述： 创建启动任务，配置任务的函数、名称、堆栈大小、优先级等参数，是系统任务的入口
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2026/01/20	     V1.0	  FTW	      创建
 **********************************************************************/
void TASK_START(void)
{
	/* 创建启动任务（FreeRTOS标准接口）：优先级最高，确保先完成核心任务初始化 */
	if (xTaskCreate((TaskFunction_t)start_task,					//任务函数
				(const char *)"start_task",					//任务名称
				(configSTACK_DEPTH_TYPE)START_TASK_SIZE, 	//堆栈大小（动态内存申请）
				(void * )NULL,								//传送任务函数的参数
				(UBaseType_t)START_TASK_PRIO,				//优先级
				(TaskHandle_t *) &start_Task_Handle ) != pdPASS) START_Fail(3U);		//任务句柄
				
						
}
