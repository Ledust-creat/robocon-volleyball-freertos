#include "includes.h"
#define POSITION_PID 1 // 位置式
#define DELTA_PID 2	   // 增量式
#define PID_MODE 1

/**
 * @brief 将浮点数限制在指定的绝对值范围内
 *
 * @param x 指向要限制的浮点数的指针
 * @param limit 限制的绝对值边界
 *
 * @note 此函数会修改*x的值，使其保持在[-limit, limit]范围内
 */
static void abs_limit(float *x, int32_t limit)
{
	/* 如果值超过正边界，则设置为正边界值 */
	if (*x > limit)
		*x = limit;
	/* 如果值超过负边界，则设置为负边界值 */
	if (*x < -limit)
		*x = -limit;
}

/**
 * @brief 初始化PID控制器参数
 * @param pid 指向PID控制器结构体的指针
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @param max_out 输出最大值限制
 * @param integral_limit 积分限幅值
 */
void pid_init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit)
{
	/* 设置PID控制器的基本参数 */
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxout = max_out;
	pid->integral_limit = integral_limit;

	/* 设置输出死区为5 */
	pid->output_deadband = 5;
}

/*
*********************************************************************************************************
*   函 数 名: pid_reset
*   功能说明: 更新PID增益并清除输出、积分和误差历史
*   形    参: pid - 控制器；p/i/d - 保留或重新指定的增益
*   返 回 值: 无
*   说    明: 不改变最大输出和积分限幅；调用方若继续控制，应重新提供当前目标和测量。
*********************************************************************************************************
*/
void pid_reset(pid_t *pid, float p, float i, float d)
{
	/* 设置PID三个参数 */
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;

	/* 初始化PID各项输出为0 */
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
	pid->set = 0;
	pid->get = 0;
	// 同时清除误差历史，避免停机后重新使能沿用上次误差产生微分冲击。
	pid->error[NOW_ERR] = 0;
	pid->error[LAST_ERR] = 0;
	pid->error[LLAST_ERR] = 0;
}

/**
 * PID控制器计算函数
 * 根据当前值和设定值计算PID输出
 *
 * @param pid PID控制器结构体指针，包含PID参数和状态信息
 * @param get 当前反馈值（实际测量值）
 * @param set 目标设定值
 * @return PID控制器输出值，如果在死区范围内则返回0
 */
float pid_calc(pid_t *pid, float get, float set)
{
	// 更新当前值和设定值
	pid->get = get;
	pid->set = set;
	// 计算当前误差值
	pid->error[NOW_ERR] = set - get;

#if (PID_MODE == POSITION_PID)
	// 位置式PID计算
	pid->pout = pid->kp * pid->error[NOW_ERR];							// 比例项计算
	pid->iout += pid->ki * pid->error[NOW_ERR];							// 积分项累加计算
	pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]); // 微分项计算

	abs_limit(&(pid->iout), pid->integral_limit); // 积分项限幅
	pid->out = pid->pout + pid->iout + pid->dout; // PID输出合成
	abs_limit(&(pid->out), pid->maxout);		  // 输出值限幅
#elif (PID_MODE == DELTA_PID)
	// 增量式PID计算
	pid->pout = pid->kp * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);							// 比例项计算
	pid->iout = pid->ki * pid->error[NOW_ERR];													// 积分项计算
	pid->dout = pid->kd * (pid->error[NOW_ERR] * pid->error[LAST_ERR] + pid->error[LLAST_ERR]); // 微分项计算

	pid->out += pid->pout + pid->iout + pid->dout; // 增量式输出累加
	abs_limit(&(pid->out), pid->maxout);		   // 输出值限幅
#endif

	// 更新历史误差值，用于下次计算
	pid->error[LLAST_ERR] = pid->error[LAST_ERR];
	pid->error[LAST_ERR] = pid->error[NOW_ERR];

	// 死区处理：如果输出值在死区范围内则返回0
	if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
		return 0;
	else
		return pid->out;
}



//  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
//    return 0;
//  else

/**
 * @brief PID结构体初始化函数
 * @param pid 指向PID结构体的指针
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @param max_out 输出值上限
 * @param integral_limit 积分限幅值
 * @param init_status 初始化状态标识，决定初始化方式
 * @return 无返回值
 *
 * 该函数根据初始化状态标识来初始化PID控制器，支持两种初始化模式：
 * 1. 正常初始化模式：同时初始化和复位PID参数
 * 2. 调试初始化模式：仅初始化PID参数
 */
void PID_Struct_Init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit, INIT_STATUS init_status)
{
	/* 正常初始化模式 */
	if (init_status == INIT)
	{
		pid->f_pid_init = pid_init;
		pid->f_pid_reset = pid_reset;

		pid->f_pid_init(pid, p, i, d, max_out, integral_limit);
		pid->f_pid_reset(pid, p, i, d);
	}
	else
	{
		/* 调试初始化模式 */
		pid->f_pid_init = pid_init;
		pid->f_pid_init(pid, p, i, d, max_out, integral_limit);
	}
}
/*
*********************************************************************************************************
*   函 数 名: PID_PROCESS_Double
*   功能说明: 执行角度外环和速度内环的串级PID计算
*   形    参: pid_Angle/pid_speed - 两级控制器；target/Angle_get - 外环目标/反馈；speed_get - 内环反馈，击打采用deg/rpm
*   返 回 值: 速度环输出控制字，限幅由控制器配置决定
*   说    明: 角度环输出按现有标定作为速度环目标；直接调用pid_calc，避免未初始化函数指针。
*********************************************************************************************************
*/
int16_t PID_PROCESS_Double(pid_t *pid_Angle,pid_t *pid_speed,float target, float Angle_get, float speed_get)
{
	//position		

	(void)pid_calc(pid_Angle, Angle_get, target);
	//speed

	(void)pid_calc(pid_speed, speed_get, pid_Angle->out);
	return pid_speed->out;
}

