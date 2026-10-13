
/**
  ******************************************************************************
  * @file    chassis_task.c
  * @brief   底盘控制任务
  *
  * @note
  *          本文件基于当前新的 BSP_CAN + DVE_DjiMotor 底层重构版本编写。
  *          底盘任务不再使用旧的 C620[] 全局数组、xChassisFeedbackQueue、Set_voltagec()。
  *
  *          当前数据流：
  *          1. rc_task 解析遥控器，得到 vx / vy / wz；
  *          2. rc_task 通过 xChassisCtrlQueue 写入最新底盘速度命令；
  *          3. chassis_task 周期读取速度命令；
  *          4. chassis_task 内部完成麦克纳姆轮解算和速度 PID；
  *          5. chassis_task 从 DVE_DjiMotor 读取 0x201~0x204 反馈；
  *          6. chassis_task 通过 DVE_DjiMotor_SetCurrent() 写入电流缓存；
  *          7. chassis_task 通过 DVE_DjiMotor_SendCurrentGroup() 发送 0x200 控制帧。
  *
  *          DVE_DjiMotor 只负责 DJI 协议反馈解析、电流缓存和 CAN 控制帧打包，
  *          PID、底盘运动学、遥控超时保护均放在本任务层。
  ******************************************************************************
  */

#include "chassis_task.h"

#include "main.h"
#include "pid.h"
#include "dve_dji_motor.h"
#include <stdbool.h>

/* 宏定义 --------------------------------------------------------------------*/

#define CHASSIS_TASK_PRIO              (26U)       /* 底盘任务优先级 */
#define CHASSIS_TASK_SIZE              (256U)      /* 底盘任务栈大小，单位为 StackType_t */
#define CHASSIS_TASK_PERIOD_MS         (10U)        /* 底盘速度环周期，单位 ms */

#define CHASSIS_CMD_TIMEOUT_MS         (100U)      /* 遥控命令超时时间，超时后目标速度清零 */
#define CHASSIS_MOTOR_OFFLINE_MS       (100U)      /* 电机反馈超时时间，超时后对应电机输出清零 */

#define CHASSIS_MOTOR_NUM              (4U)        /* 底盘 3508 数量，当前为 0x201~0x204 */
#define CHASSIS_HEARTBEAT_PERIOD       (100U)      /* 任务心跳翻转周期，单位：任务周期次数 */

/*
 * 麦克纳姆底盘机械参数。
 * 当前数值沿用原 car_ctrl.c，速度单位也沿用原工程标定。
 * 如果后续要做真实 m/s 和 rpm 换算，再统一修改这里和遥控速度比例。
 */
#define CHASSIS_WHEEL_RADIUS           (0.077f)    /* 轮半径，单位 m */
#define CHASSIS_WHEEL_BASE_HALF        (0.430f)    /* 前后轮距一半，单位 m */
#define CHASSIS_WHEEL_TRACK_HALF       (0.385f)    /* 左右轮距一半，单位 m */
#define CHASSIS_MAX_WHEEL_SPEED        (10000.0f)  /* 单轮目标速度限幅，沿用原工程 */

/*
 * 底盘速度环 PID 参数。
 * 当前先沿用原工程 C620[i].Speed_pid 初始化参数。
 * 后续实车调参时优先修改这里，不要到函数内部改魔法数字。
 */
#define CHASSIS_SPEED_PID_KP           (2.0f)
#define CHASSIS_SPEED_PID_KI           (0.3f)
#define CHASSIS_SPEED_PID_KD           (0.0f)
#define CHASSIS_SPEED_PID_MAX_OUT      (10000)
#define CHASSIS_SPEED_PID_I_LIMIT      (16000)

/* 类型定义 ------------------------------------------------------------------*/

typedef struct
{
    int16_t target_speed[CHASSIS_MOTOR_NUM];       /* 四个底盘电机目标速度 */
} Chassis_WheelTarget_t;

/* 全局变量 ------------------------------------------------------------------*/

TaskHandle_t chassis_Task_Handle = NULL;
QueueHandle_t xChassisCtrlQueue = NULL;

/* 私有变量 ------------------------------------------------------------------*/

static pid_t s_chassis_speed_pid[CHASSIS_MOTOR_NUM];

static const DVE_DjiMotorId_t s_chassis_motor_id[CHASSIS_MOTOR_NUM] =
{
    DJI_MOTOR_CHASSIS_1,                            /* CAN2 ID 0x201 */
    DJI_MOTOR_CHASSIS_2,                            /* CAN2 ID 0x202 */
    DJI_MOTOR_CHASSIS_3,                            /* CAN2 ID 0x203 */
    DJI_MOTOR_CHASSIS_4                             /* CAN2 ID 0x204 */
};

/* 私有函数声明 --------------------------------------------------------------*/

static void chassis_task(void *parm);
static void chassis_pid_init(void);
static void chassis_pid_reset_one(uint8_t index);
static void chassis_calc_wheel_target(const Chassis_MoveCmd_t *cmd,
                                      Chassis_WheelTarget_t *target);
static void chassis_set_zero_current(void);
static float abs_f32(float value);
static float max4_f32(float a, float b, float c, float d);
static float clamp_f32(float value, float min_value, float max_value);
static int16_t float_to_i16(float value);

/* 函数定义 ------------------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: chassis_task_creat
*   功能说明: 创建本模块命令队列与业务任务，并向调用者报告结果
*   形    参: 无；仅系统启动阶段调用一次
*   返 回 值: OK表示成功；ERR_QUEUE表示队列失败；ERR_TASK表示任务失败且队列已释放
*   说    明: 调用者保证创建阶段不被业务任务抢占；本模块不依赖启动模块或决定整机故障策略。
*********************************************************************************************************
*/
Task_CreateResult_t chassis_task_creat(void)
{
    xChassisCtrlQueue = xQueueCreate(1, sizeof(Chassis_MoveCmd_t));
    if (xChassisCtrlQueue == NULL) return TASK_CREATE_ERR_QUEUE;


    if (xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"chassis_task",
                (configSTACK_DEPTH_TYPE)CHASSIS_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)CHASSIS_TASK_PRIO,
                (TaskHandle_t *)&chassis_Task_Handle) != pdPASS)
    {
        // 创建失败时只释放本模块刚分配的队列，不调用上层启动或故障处理函数。
        vQueueDelete(xChassisCtrlQueue);
        xChassisCtrlQueue = NULL;
        return TASK_CREATE_ERR_TASK;
    }
    return TASK_CREATE_OK;
}

/*
*********************************************************************************************************
*   函 数 名: chassis_task
*   功能说明: 底盘控制任务主体
*   形    参: parm              - FreeRTOS 任务参数，未使用
*   返 回 值: 无
*   运行周期: CHASSIS_TASK_PERIOD_MS
*   主要流程: 1. 接收最新底盘速度命令；
*             2. 命令超时则目标速度清零；
*             3. 麦克纳姆轮运动学解算四轮目标速度；
*             4. 读取 DVE_DjiMotor 中的 0x201~0x204 反馈；
*             5. 执行四个速度环 PID；
*             6. 写入 DJI 电流缓存；
*             7. 发送 0x200 控制帧。
*********************************************************************************************************
*/
static void chassis_task(void *parm)
{
    TickType_t xLastWakeTime;
    TickType_t last_cmd_tick;
    Chassis_MoveCmd_t curr_cmd = {0};
    Chassis_MoveCmd_t new_cmd = {0};
    Chassis_WheelTarget_t wheel_target = {0};
    DVE_DjiMotorFeedback_t feedback = {0};
    uint16_t heartbeat_cnt = 0U;

    (void)parm;

    // 启动编排保证队列和设备先就绪，任务本身不查询启动状态。
    xLastWakeTime = xTaskGetTickCount();
    last_cmd_tick = xTaskGetTickCount();

    chassis_pid_init();
    // 控制任务首次运行时提交零电流，创建函数只分配资源，不进入CAN等待事务。
    chassis_set_zero_current();
	
    for (;;)
    {
        /*
         * 1. 读取遥控任务写入的底盘速度命令。
         *    队列长度为 1，rc_task 建议使用 xQueueOverwrite()，因此这里只保留最新命令。
         */
        if (xQueueReceive(xChassisCtrlQueue, &new_cmd, 0) == pdTRUE)
        {
            curr_cmd = new_cmd;
            last_cmd_tick = xTaskGetTickCount();
        }

        /*
         * 2. 遥控命令超时保护。
         *    超过指定时间没有新命令，认为遥控/上层控制断联，底盘目标速度清零。
         */
        if ((xTaskGetTickCount() - last_cmd_tick) > pdMS_TO_TICKS(CHASSIS_CMD_TIMEOUT_MS))
        {
            curr_cmd.enable = 0U;
            curr_cmd.vx = 0.0f;
            curr_cmd.vy = 0.0f;
            curr_cmd.wz = 0.0f;
        }

        /* 3. 根据当前 vx/vy/wz 计算四个底盘电机目标速度。 */
        // 失控的enable=0清电流和PID历史；正常使能且零速仍由原速度环控制。
        if (curr_cmd.enable != 0U)
        {
            chassis_calc_wheel_target(&curr_cmd, &wheel_target);

            /*
             * 4. 四个底盘电机分别做速度环。
             *    DVE 层只提供反馈缓存和电流发送接口，PID 仍然在任务层完成。
             */
            for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
            {
                if (DVE_DjiMotor_IsOnline(s_chassis_motor_id[i], CHASSIS_MOTOR_OFFLINE_MS) == true)
                {
                    if (DVE_DjiMotor_GetFeedback(s_chassis_motor_id[i], &feedback) == DVE_DJI_OK)
                    {
                        (void)pid_calc(&s_chassis_speed_pid[i],
                                       (float)feedback.speed_rpm,
                                       (float)wheel_target.target_speed[i]);

                        (void)DVE_DjiMotor_SetCurrent(s_chassis_motor_id[i],
                                                      (int32_t)s_chassis_speed_pid[i].out);
                    }
                    else
                    {
                        /* 理论上在线判断通过后读取失败概率很低，失败时保守清零该路输出。 */
                        (void)DVE_DjiMotor_SetCurrent(s_chassis_motor_id[i], 0);
                        chassis_pid_reset_one(i);
                    }
                }
                else
                {
                    /* 电机离线：清零该路输出并重置 PID，避免积分残留。 */
                    (void)DVE_DjiMotor_SetCurrent(s_chassis_motor_id[i], 0);
                    chassis_pid_reset_one(i);
                }
            }
        }
        else
        {
            for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
            {
                (void)DVE_DjiMotor_SetCurrent(s_chassis_motor_id[i], 0);
                chassis_pid_reset_one(i);
            }
        }
        /* 5. 统一发送 0x200 控制帧，控制 0x201~0x204 四个底盘电机。 */
        (void)DVE_DjiMotor_SendCurrentGroup(DVE_DJI_TX_GROUP_200);
		
        /* 6. 任务心跳，仅用于观察底盘任务是否仍在运行。 */
        heartbeat_cnt++;
        if (heartbeat_cnt >= CHASSIS_HEARTBEAT_PERIOD)
        {
			#if defined(led3_GPIO_Port) && defined(led3_Pin)
						HAL_GPIO_TogglePin(led3_GPIO_Port, led3_Pin);
			#endif
						heartbeat_cnt = 0U;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
    }
}

/*
*********************************************************************************************************
*   函 数 名: chassis_pid_init
*   功能说明: 初始化四个底盘速度环 PID
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void chassis_pid_init(void)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
        PID_Struct_Init(&s_chassis_speed_pid[i],
                        CHASSIS_SPEED_PID_KP,
                        CHASSIS_SPEED_PID_KI,
                        CHASSIS_SPEED_PID_KD,
                        CHASSIS_SPEED_PID_MAX_OUT,
                        CHASSIS_SPEED_PID_I_LIMIT,
                        INIT);
    }
}

/*
*********************************************************************************************************
*   函 数 名: chassis_pid_reset_one
*   功能说明: 重置指定底盘电机速度环 PID
*   形    参: index             - 底盘电机下标，范围 0~3
*   返 回 值: 无
*********************************************************************************************************
*/
static void chassis_pid_reset_one(uint8_t index)
{
    if (index >= CHASSIS_MOTOR_NUM)
    {
        return;
    }

    pid_reset(&s_chassis_speed_pid[index],
              CHASSIS_SPEED_PID_KP,
              CHASSIS_SPEED_PID_KI,
              CHASSIS_SPEED_PID_KD);
}

/*
*********************************************************************************************************
*   函 数 名: chassis_calc_wheel_target
*   功能说明: 根据底盘速度命令计算四个麦克纳姆轮目标速度
*   形    参: cmd               - 底盘速度命令
*             target            - 四个电机目标速度输出
*   返 回 值: 无
*   说    明: 当前计算和符号关系沿用原 car_ctrl.c：
*             motor0 = -rr，motor1 = -fr，motor2 = fl，motor3 = rl。
*             如果实车运动方向反了，优先改这里，不要改 DVE_DjiMotor 层。
*********************************************************************************************************
*/
static void chassis_calc_wheel_target(const Chassis_MoveCmd_t *cmd,
                                      Chassis_WheelTarget_t *target)
{
    float omega_fl;
    float omega_fr;
    float omega_rr;
    float omega_rl;
    float max_omega;
    float scale = 1.0f;
    float lw_sum;

    if ((cmd == NULL) || (target == NULL))
    {
        return;
    }

    lw_sum = CHASSIS_WHEEL_BASE_HALF + CHASSIS_WHEEL_TRACK_HALF;

    /*
     * 麦克纳姆轮逆运动学。
     * 这里保持原工程算法，不额外做 rad/s 到 rpm 的真实物理换算，避免改变原遥控标定。
     */
    omega_fl = (cmd->vx - cmd->vy - lw_sum * cmd->wz) / CHASSIS_WHEEL_RADIUS;
    omega_fr = (cmd->vx + cmd->vy + lw_sum * cmd->wz) / CHASSIS_WHEEL_RADIUS;
    omega_rr = (cmd->vx - cmd->vy + lw_sum * cmd->wz) / CHASSIS_WHEEL_RADIUS;
    omega_rl = (cmd->vx + cmd->vy - lw_sum * cmd->wz) / CHASSIS_WHEEL_RADIUS;

    /* 四轮整体等比例限幅，避免某一路目标过大导致运动方向比例被破坏。 */
    max_omega = max4_f32(abs_f32(omega_fl),
                         abs_f32(omega_fr),
                         abs_f32(omega_rr),
                         abs_f32(omega_rl));

    if (max_omega > CHASSIS_MAX_WHEEL_SPEED)
    {
        scale = CHASSIS_MAX_WHEEL_SPEED / max_omega;
    }
    
    omega_fl = clamp_f32(omega_fl * scale, -CHASSIS_MAX_WHEEL_SPEED, CHASSIS_MAX_WHEEL_SPEED);
    omega_fr = clamp_f32(omega_fr * scale, -CHASSIS_MAX_WHEEL_SPEED, CHASSIS_MAX_WHEEL_SPEED);
    omega_rr = clamp_f32(omega_rr * scale, -CHASSIS_MAX_WHEEL_SPEED, CHASSIS_MAX_WHEEL_SPEED);
    omega_rl = clamp_f32(omega_rl * scale, -CHASSIS_MAX_WHEEL_SPEED, CHASSIS_MAX_WHEEL_SPEED);

    target->target_speed[0] = float_to_i16(-omega_rr);
    target->target_speed[1] = float_to_i16(-omega_fr);
    target->target_speed[2] = float_to_i16( omega_fl);
    target->target_speed[3] = float_to_i16( omega_rl);
}

/*
*********************************************************************************************************
*   函 数 名: chassis_set_zero_current
*   功能说明: 清零四个底盘电机输出电流，并尝试立即发送 0x200 控制帧
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void chassis_set_zero_current(void)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
        (void)DVE_DjiMotor_SetCurrent(s_chassis_motor_id[i], 0);
    }

    (void)DVE_DjiMotor_SendCurrentGroup(DVE_DJI_TX_GROUP_200);
}

/*
*********************************************************************************************************
*   函 数 名: abs_f32
*   功能说明: 计算 float 绝对值
*   形    参: value             - 输入值
*   返 回 值: float             - 绝对值
*********************************************************************************************************
*/
static float abs_f32(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/*
*********************************************************************************************************
*   函 数 名: max4_f32
*   功能说明: 计算四个 float 中的最大值
*   形    参: a,b,c,d           - 输入值
*   返 回 值: float             - 最大值
*********************************************************************************************************
*/
static float max4_f32(float a, float b, float c, float d)
{
    float max_value = a;

    if (b > max_value)
    {
        max_value = b;
    }

    if (c > max_value)
    {
        max_value = c;
    }

    if (d > max_value)
    {
        max_value = d;
    }

    return max_value;
}

/*
*********************************************************************************************************
*   函 数 名: clamp_f32
*   功能说明: 将 float 限制在指定范围内
*   形    参: value             - 输入值
*             min_value         - 下限
*             max_value         - 上限
*   返 回 值: float             - 限幅后的值
*********************************************************************************************************
*/
static float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/*
*********************************************************************************************************
*   函 数 名: float_to_i16
*   功能说明: 将 float 转换为 int16_t，带简单四舍五入
*   形    参: value             - 输入浮点数
*   返 回 值: int16_t           - 转换结果
*********************************************************************************************************
*/
static int16_t float_to_i16(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}
