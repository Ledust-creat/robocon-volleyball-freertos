#include "includes.h"
#include "pid.h"
#include <stdint.h>
#include "bsp_can.h"
#include "dve_dji_motor.h"
extern motor_info_t damiao[MotorCount];
void All_Init(){
	BSP_CAN_InitDefault( );
	DVE_DjiMotor_Init(&hcan2);	
}
void All_Clear(){
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