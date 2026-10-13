
#include "includes.h"
#include "can.h"
#include "motor_can.h"
#include <stdint.h>
uint16_t PID_Calc_Flag = 0;

 int16_t k; //用于led闪烁检查是否正常进入定时器中断
// 发球状态机在遥控器模块中推进，这里按固定周期调用
extern void remote_control_serve_update(void);
/************************ 定时器更新中断回调函数 ************************/
/*
*********************************************************************************************************
*   函 数 名: HAL_TIM_PeriodElapsedCallback
*   功能说明: 处理HAL定时器周期回调，维护TIM4毫秒时基及旧TIM3指示逻辑
*   形    参: htim - 触发更新中断的定时器句柄
*   返 回 值: 无
*   说    明: TIM4仅递增HAL tick；SysTick由FreeRTOS维护，控制计算已移至周期任务。
*********************************************************************************************************
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // TIM4每1ms递增HAL tick；SysTick归FreeRTOS，避免漏计或重复计时。
    if (htim->Instance == TIM4)
    {
        HAL_IncTick();
        return;
    }
    static int16_t voltages[4];
	static int16_t voltage_angle[1];
    if (htim == &htim3)  // 确认是PID定时器的更新中断
    {
		/*led闪烁*/
		k=k+1;
		if(k>500)
		{
			HAL_GPIO_TogglePin(led1_GPIO_Port,led1_Pin);
			
			k=0;
		}
		/*机械臂达妙电机控制原代码*/
		// 每 10ms 更新一次发球动作阶段
//		remote_control_serve_update();
//		Set_dm(&hcan1,0);
//		Set_dm(&hcan1,1);
		/*底盘控制原代码*/		
//        for(int i=0;i<MotorCount;i++)
//        {
//            voltages[i]=(int16_t)C620[i].Speed_pid.out;
//            
//        }
//        Set_voltagec(&hcan1,voltages);
//		comm_can_set_rpm(001, C620[0].Speed_pid.out);
		/*陀螺仪*/
//		JY901P_ReadAllData(&gyro_data);//读取陀螺仪数据
//        pid_calc(&car_pid, gyro_data.Gyro_Z-Z_zeropoint, 0); // 假设控制角速度为0
//		 float num[]={				//gyro_data.Gyro_X,
//            gyro_data.Gyro_Y,
//            gyro_data.Gyro_Z,
//            gyro_data.Acc_X,
//            gyro_data.Acc_Y,
//            gyro_data.Acc_Z,
//            gyro_data.Angle_X,
//            gyro_data.Angle_Y,
//            gyro_data.Angle_Z
//        };
//        Vofa_JustFloat(num, 3);//串口打印
//        car_w=car_pid.out;

    }
//	if(hcan1.ErrorCode!=0)//避免can总线错误导致死机
//	{
//		HAL_CAN_DeInit(&hcan1);
//		HAL_CAN_Init(&hcan1);
//		HAL_CAN_Start(&hcan1);
//	
//	
//	}
//    if(hcan2.ErrorCode!=0)//避免can总线错误导致死机
//	{
//		HAL_CAN_DeInit(&hcan2);
//		HAL_CAN_Init(&hcan2);
//		HAL_CAN_Start(&hcan2);
//	
//	
//	}
    //在这里可以添加角度环的中断处理逻辑
//    if(htim == &htim14)  // 确认是PID定时器的更新中断
//    {
//        pid_calc(&C620_angle.Speed_pid,C620_angle.Speed_pid.get,C620_angle.Speed_pid.set);
//        voltage_angle[0]=(int16_t)C620_angle.Speed_pid.out;
//        Set_voltage_angle(&hcan2,voltage_angle);
//    } 
}

/************************ 错误处理函数（可选） ************************/
#ifdef USE_FULL_ASSERT
void Error_Handler(void)
{
    // 可添加LED闪烁、串口打印等错误提示逻辑
    while(1)
    {
    }
}
#endif
