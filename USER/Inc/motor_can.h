#ifndef __MOTOR_CAN_H
#define __MOTOR_CAN_H

#include "main.h"
#include "can.h"
#include "pid.h"
/*



*/
#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
#define MotorCount 4

//角度归一化
typedef struct{
	int16_t Voltage;//电压值
	uint16_t Angle;//机械角度
	int16_t Speed;//转速
	int16_t Torque;//实际扭矩
	uint8_t Temp;//温度
	
}RxMsg_t;

typedef struct{
	pid_t 				Speed_pid;
	pid_t 				Angel_pid;
	
	//控制角度的参数
	uint16_t			FirstEntre;
	double			Target;//目标角度
	uint16_t 			lastRead;//上一次读取值
	uint16_t 			currentRead;//当前读取值
	uint16_t 			Zero;//上电后的第一个位置做为零点
	int32_t 			totalAngle;//总角度
	float				encoderAngle;//经过处理的电机角度
	int16_t				Current;//输出电流
	float				out;//输出电压

	float				angle;//目标角度
	float				speed;//目标速度
	float            	KP;
	float            	KD;
	float            	tor;
	uint32_t         	ID     ;//电机id
	
	RxMsg_t 			Rxmsg;
}motor_info_t;



void can_filter_init(void);
void can1_filter_init(void);
void can2_fliter_init(void);


void Set_voltagec(CAN_HandleTypeDef* hcan,int16_t vlotage[]);
void Set_voltage_angle(CAN_HandleTypeDef* hcan,int16_t vlotage[]);

void Set_dm(CAN_HandleTypeDef* hcan,int16_t g);
void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID);
void Set_dm_Angle(CAN_HandleTypeDef* hcan,float angle1,float angle2,float angle3,float angle4);
void Set_dm_enable(CAN_HandleTypeDef* hcan,uint8_t ID);
void MIT_Calc(motor_info_t *motor,int16_t target_torque,int32_t target_Angle,int16_t target_speed);

#endif

