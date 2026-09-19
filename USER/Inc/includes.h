#ifndef __INCLUDES_H
#define __INCLUDES_H


#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "pid.h"

#include "tim.h"
#include "main.h" 
#include "motor_can.h"
#include "pid_tim.h"
//#include "car_ctrl.h"
//#include "bsp_usart.h"
#include "Initialize.h"
#include "rc_task.h"
#include "math_utils.h"
#include "jy901p.h"
#include "JY901P_Calibrate.h"
//#include "N630.h"
extern motor_info_t C620[MotorCount];
extern pid_t car_pid;
extern motor_info_t C6xx[MotorCount];
extern motor_info_t C620_angle;
extern motor_info_t damiao[MotorCount];
extern uint16_t PID_Calc_Flag;
extern float car_x,car_y,car_w;
extern float car_tarx,car_tary,car_tarw;
//extern JY901P_DataStruct gyro_data;
//extern int16_t Z_zeropoint;


#define MAX_CAR_SPEED 1000.0f
//底盘匀加/减速参数，T=10ms
#define SPEED_UP_TICKS (60u) // 从静止加速到最大速度需要的控制周期数
#define SPEED_DOWN_TICKS (70u) // 从最大速度减速到静止需要的控制周期数

#endif
