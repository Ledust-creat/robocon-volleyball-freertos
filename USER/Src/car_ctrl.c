#include "car_ctrl.h"
#include "includes.h"

 motor_info_t C620[MotorCount];
 pid_t car_pid;
/************************ 麦轮机器人参数配置 ************************/
// 物理参数（根据实际机器人修改）
#define WHEEL_RADIUS    0.077f     // 轮子半径（米），示例：5cm
#define WHEEL_BASE_HALF  0.430f     // 轮距半长（米），示例：15cm
#define WHEEL_TRACK_HALF 0.385f   // 轴距半宽（米），示例：15cm
#define MAX_WHEEL_SPEED 10000     // 轮子最大转速（如PWM占空比最大值、编码器脉冲/秒）

float car_x,car_y,car_w;
float car_tarx,car_tary,car_tarw;

// 轮子转速结构体（存储四个轮子的目标转速）
typedef struct {
    int16_t fl;  // 左前轮转速
    int16_t fr;  // 右前轮转速
    int16_t rr;  // 右后轮转速
    int16_t rl;  // 左后轮转速
} WheelSpeed_t;

/************************ 麦轮移动核心函数 ************************/
/**
 * @brief  麦轮机器人速度逆解，计算四轮目标转速
 * @param  vx: x轴平移速度（前=正，后=负），单位：m/s
 * @param  vy: y轴平移速度（左=正，右=负），单位：m/s
 * @param  wz: 旋转速度（顺时针=正，逆时针=负），单位：rad/s
 * @retval 四轮目标转速（已限幅，可直接输出到电机驱动）
 */
void MecanumWheel_Move(float vx, float vy, float wz,motor_info_t *moto_goal)
{
    WheelSpeed_t wheel_speed = {0};
    float omega_fl, omega_fr, omega_rr, omega_rl;
    float max_omega = 0.0f;
    float scale = 1.0f;
	
    // 1. 运动学逆解：计算原始转速（rad/s）
    float lw_sum = WHEEL_BASE_HALF + WHEEL_TRACK_HALF;  // L+W
    omega_fl = (vx - vy - lw_sum * wz) / WHEEL_RADIUS;
    omega_fr = (vx + vy + lw_sum * wz) / WHEEL_RADIUS;
    omega_rr = (vx - vy + lw_sum * wz) / WHEEL_RADIUS;
    omega_rl = (vx + vy - lw_sum * wz) / WHEEL_RADIUS;

    // 2. 速度归一化（防止超过最大转速）
    max_omega = fmaxf(fmaxf(fabsf(omega_fl), fabsf(omega_fr)),
                      fmaxf(fabsf(omega_rr), fabsf(omega_rl)));
    if (max_omega > MAX_WHEEL_SPEED) {
        scale = MAX_WHEEL_SPEED / max_omega;  // 计算缩放系数
    }

    // 3. 缩放并转换为整数转速（适配电机驱动）
    wheel_speed.fl = (int16_t)(omega_fl * scale);
    wheel_speed.fr = (int16_t)(omega_fr * scale);
    wheel_speed.rr = (int16_t)(omega_rr * scale);
    wheel_speed.rl = (int16_t)(omega_rl * scale);

	moto_goal[0].Speed_pid.set = -wheel_speed.rr;
	moto_goal[1].Speed_pid.set = -wheel_speed.fr;
	moto_goal[2].Speed_pid.set = wheel_speed.fl;
	moto_goal[3].Speed_pid.set = wheel_speed.rl;
}
