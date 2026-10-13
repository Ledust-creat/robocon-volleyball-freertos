#include "includes.h"
#include "pid.h"
#include <stdint.h>
#include "bsp_can.h"
#include "dve_dji_motor.h"
extern motor_info_t damiao[MotorCount];
/*
*********************************************************************************************************
*   函 数 名: All_Init
*   功能说明: 初始化设备对象和默认CAN接收链路
*   形    参: 无；在外设初始化后、调度器启动前调用
*   返 回 值: true表示设备与CAN就绪；false表示初始化失败，由main处理
*   说    明: 先初始化DVE再开启CAN接收，避免ISR首帧基准被后续设备初始化清除。
*********************************************************************************************************
*/
bool All_Init(void){
    if (DVE_DjiMotor_Init(&hcan2) != DVE_DJI_OK) return false;
    if (BSP_CAN_InitDefault() != BSP_CAN_OK) return false;
    return true;
}
void All_Clear(void){
//    //Set_dm_disable(&hcan1, 0X00);
//    damiao[0].KP = 0.0f;//150.0f;
//	damiao[0].KD = 0.0f;
//	damiao[0].tor = 0.0f;//-1.65
//	damiao[0].angle=0.0f;
//    for(int i=0;i<MotorCount;i++){
//        pid_reset(&C620[i].Speed_pid, 0.0f, 0.0f, 0.0f);
//        pid_reset(&C620[i].Angel_pid, 0.0f, 0.0f, 0.0f);
//        pid_reset(&damiao[i].Speed_pid, 0.0f, 0.0f, 0.0f);
//    }
//    pid_reset(&car_pid, 0.0f, 0.0f, 0.0f);
//    pid_reset(&C620_angle.Speed_pid, 0.0f, 0.0f, 0.0f);
}
