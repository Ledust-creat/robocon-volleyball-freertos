#include "hit_task.h"
#include "dve_dji_motor.h"
#include "pid.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdbool.h>
#include <stdint.h>

/* 私有类型定义 --------------------------------------------------------------*/

typedef enum
{
    HIT_STATE_IDLE = 0,                /* 空闲，目标保持在回零角度 */
    HIT_STATE_PUT_ANGLE,               /* SF 按下，进入当前预设击球角度 */
    HIT_STATE_RETURN,                  /* SF 松开，平滑回到回零角度 */
    HIT_STATE_DISABLE                  /* 停机保护，0x1FF 组输出清零 */
} Hit_State_t;

typedef struct
{
    DVE_DjiMotorId_t dji_id;           /* 对应 DVE_DjiMotor 软件编号 */
    pid_t angle_pid;                   /* 击球电机角度环 PID */
    pid_t speed_pid;                   /* 击球电机速度环 PID */
    float target_angle;                /* 当前目标累计角度 */
    float total_angle;                 /* 多圈累计角度反馈 */
    float last_angle_deg;              /* 上一次单圈角度反馈，单位 deg */
    uint8_t angle_inited;              /* 多圈角度累计初始化标志 */
} Hit_MotorCtrl_t;

/* 私有常量 ------------------------------------------------------------------*/

/*
 * 击球动作角度表。
 * 行：SWC 三档预设；列：0x206、0x207、0x208 三个击球电机。
 * 表内数值已经乘以方向修正和 HIT_SCALE，可直接作为累计角度目标。
 */
static const float s_hit_angle_table[HIT_PRESET_NUM][HIT_MOTOR_NUM] =
{
    /* 预设 0：大角度击球，适合需要更大击球行程的情况。 */
    {
        HIT_MOTOR1_DIR * HIT_PRESET0_ANGLE_DEG * HIT_SCALE,      /* 0x206 目标角度 */
        HIT_MOTOR2_DIR * HIT_PRESET0_ANGLE_DEG * HIT_SCALE,      /* 0x207 目标角度 */
        HIT_MOTOR3_DIR * HIT_PRESET0_ANGLE_DEG * HIT_SCALE       /* 0x208 目标角度 */
    },

    /* 预设 1：中等角度击球，作为常用击球角度。 */
    {
        HIT_MOTOR1_DIR * HIT_PRESET1_ANGLE_DEG * HIT_SCALE,      /* 0x206 目标角度 */
        HIT_MOTOR2_DIR * HIT_PRESET1_ANGLE_DEG * HIT_SCALE,      /* 0x207 目标角度 */
        HIT_MOTOR3_DIR * HIT_PRESET1_ANGLE_DEG * HIT_SCALE       /* 0x208 目标角度 */
    },

    /* 预设 2：小角度击球，适合轻动作或调试阶段。 */
    {
        HIT_MOTOR1_DIR * HIT_PRESET2_ANGLE_DEG * HIT_SCALE,      /* 0x206 目标角度 */
        HIT_MOTOR2_DIR * HIT_PRESET2_ANGLE_DEG * HIT_SCALE,      /* 0x207 目标角度 */
        HIT_MOTOR3_DIR * HIT_PRESET2_ANGLE_DEG * HIT_SCALE       /* 0x208 目标角度 */
    },
};

/*
 * 击球回零角度表。
 * SF 松开后，三个击球电机的目标角度会按 HIT_RETURN_STEP 逐步逼近该表。
 */
static const float s_hit_reset_angle[HIT_MOTOR_NUM] =
{
    HIT_MOTOR1_DIR * HIT_RESET_ANGLE_DEG * HIT_SCALE,            /* 0x206 回零角度 */
    HIT_MOTOR2_DIR * HIT_RESET_ANGLE_DEG * HIT_SCALE,            /* 0x207 回零角度 */
    HIT_MOTOR3_DIR * HIT_RESET_ANGLE_DEG * HIT_SCALE             /* 0x208 回零角度 */
};

/* 私有变量 ------------------------------------------------------------------*/

QueueHandle_t xHitCmdQueue = NULL;

static TaskHandle_t s_hit_task_handle = NULL;
static Hit_State_t s_hit_state = HIT_STATE_DISABLE;
static uint8_t s_hit_preset_index = 0U;
static float s_pitch_speed_set = HIT_PITCH_SPEED_ZERO;
static uint32_t s_last_cmd_ms = 0U;

static pid_t s_pitch_speed_pid;
static Hit_MotorCtrl_t s_hit_motor[HIT_MOTOR_NUM] =
{
    {.dji_id = DJI_MOTOR_HIT_1},
    {.dji_id = DJI_MOTOR_HIT_2},
    {.dji_id = DJI_MOTOR_HIT_3},
};

/* 私有函数声明 --------------------------------------------------------------*/

static void Hit_InitControl(void);
static void Hit_UpdateCommand(const Hit_Cmd_t *cmd);
static void Hit_UpdateFeedback(void);
static void Hit_UpdateHitMotorFeedback(uint8_t index);
static void Hit_UpdateTargetByState(void);
static uint8_t Hit_MoveTargetSmooth(uint8_t index, float target, float step_size);
static void Hit_OutputControl(void);
static void Hit_OutputStop(void);
static int16_t Hit_LimitOutput(int32_t value, int16_t limit);

/* 函数定义 ------------------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: hit_task_creat
*   功能说明: 创建击球任务和击球命令队列
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void hit_task_creat(void)
{
    xHitCmdQueue = xQueueCreate(1, sizeof(Hit_Cmd_t));
    configASSERT(xHitCmdQueue != NULL);

    xTaskCreate((TaskFunction_t)hit_task,
                (const char *)"hit_task",
                (configSTACK_DEPTH_TYPE)HIT_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)HIT_TASK_PRIO,
                (TaskHandle_t *)&s_hit_task_handle);
}

/*
*********************************************************************************************************
*   函 数 名: hit_task
*   功能说明: 击球任务，完成 0x205 速度控制和 0x206~0x208 击球角度串级控制
*   形    参: parm              - FreeRTOS 任务参数，未使用
*   返 回 值: 无
*********************************************************************************************************
*/
void hit_task(void *parm)
{
    TickType_t last_wake_tick;
    Hit_Cmd_t cmd;
    uint32_t now_ms;

    (void)parm;

    Hit_InitControl();
    last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        now_ms = HAL_GetTick();

        /* 读取遥控任务下发的最新击球命令，队列长度为 1，只保留最新命令。 */
        if (xQueueReceive(xHitCmdQueue, &cmd, 0U) == pdTRUE)
        {
			/* 根据遥控命令更新击球状态 */
            Hit_UpdateCommand(&cmd);
            s_last_cmd_ms = now_ms;
        }

        /* 遥控命令超时后，只清 0x1FF 组，避免影响底盘 0x200 组。 */
        if ((now_ms - s_last_cmd_ms) > HIT_CMD_TIMEOUT_MS)
        {
            s_hit_state = HIT_STATE_DISABLE;
        }
		/* 更新 0x205~0x208 的反馈缓存到 PID 输入 */
        Hit_UpdateFeedback();

        if (s_hit_state == HIT_STATE_DISABLE)
        {
            Hit_OutputStop();
        }
        else
        {
			/* 根据击球状态机更新三个击球电机目标角度 */
            Hit_UpdateTargetByState();
            Hit_OutputControl();
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(HIT_TASK_PERIOD_MS));
    }
}

/* 私有函数定义 --------------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: Hit_InitControl
*   功能说明: 初始化击球任务内部 PID、目标角度和状态
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void Hit_InitControl(void)
{
    PID_Struct_Init(&s_pitch_speed_pid,
                    10.0f,
                    0.3f,
                    0.0f,
                    HIT_PITCH_OUTPUT_LIMIT,
                    16000,
                    INIT);

    for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++)
    {
        PID_Struct_Init(&s_hit_motor[i].speed_pid,
                        10.5f,
                        0.0f,
                        0.15f,
                        HIT_OUTPUT_LIMIT,
                        16000,
                        INIT);

        PID_Struct_Init(&s_hit_motor[i].angle_pid,
                        7.0f,
                        0.0f,
                        0.02f,
                        30000,
                        16000,
                        INIT);

        /* 初始目标使用回零角度，避免任务启动后直接跳到击球角度。 */
        s_hit_motor[i].target_angle = s_hit_reset_angle[i];
        s_hit_motor[i].total_angle = 0.0f;
        s_hit_motor[i].last_angle_deg = 0.0f;
        s_hit_motor[i].angle_inited = 0U;
    }

    s_hit_preset_index = 0U;
    s_pitch_speed_set = HIT_PITCH_SPEED_ZERO;
    s_last_cmd_ms = HAL_GetTick();
    s_hit_state = HIT_STATE_IDLE;
}

/*
*********************************************************************************************************
*   函 数 名: Hit_UpdateCommand
*   功能说明: 根据遥控命令更新击球状态、预设编号和 0x205 目标速度
*   形    参: cmd               - 击球命令
*   返 回 值: 无
*********************************************************************************************************
*/
static void Hit_UpdateCommand(const Hit_Cmd_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    if (cmd->enable == 0U)
    {
        s_hit_state = HIT_STATE_DISABLE;
        s_pitch_speed_set = HIT_PITCH_SPEED_ZERO;
        return;
    }

    /* 预设编号越界时钳制到最后一档，避免数组越界。 */
    if (cmd->preset_index >= HIT_PRESET_NUM)
    {
        s_hit_preset_index = HIT_PRESET_NUM - 1U;
    }
    else
    {
        s_hit_preset_index = cmd->preset_index;
    }

    s_pitch_speed_set = cmd->pitch_speed_set;

    if (cmd->hit_pressed != 0U)
    {
        s_hit_state = HIT_STATE_PUT_ANGLE;
    }
    else if (s_hit_state != HIT_STATE_IDLE)
    {
        s_hit_state = HIT_STATE_RETURN;
    }
    else
    {
        s_hit_state = HIT_STATE_IDLE;
    }
}

/*
*********************************************************************************************************
*   函 数 名: Hit_UpdateFeedback
*   功能说明: 更新 0x205~0x208 的反馈缓存到 PID 输入
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void Hit_UpdateFeedback(void)
{
    DVE_DjiMotorFeedback_t pitch_feedback;

    if (DVE_DjiMotor_GetFeedback(DJI_MOTOR_PITCH, &pitch_feedback) == DVE_DJI_OK)
    {
        s_pitch_speed_pid.get = (float)pitch_feedback.speed_rpm;
    }

    for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++)
    {
        Hit_UpdateHitMotorFeedback(i);
    }
}

/*
*********************************************************************************************************
*   函 数 名: Hit_UpdateHitMotorFeedback
*   功能说明: 更新单个击球电机的多圈累计角度和转速反馈
*   形    参: index             - 击球电机下标，0~2 对应 0x206~0x208
*   返 回 值: 无
*********************************************************************************************************
*/
static void Hit_UpdateHitMotorFeedback(uint8_t index)
{
    DVE_DjiMotorFeedback_t feedback;
    float current_angle_deg;
    float delta_angle_deg;

    if (index >= HIT_MOTOR_NUM)
    {
        return;
    }

    if (DVE_DjiMotor_GetFeedback(s_hit_motor[index].dji_id, &feedback) != DVE_DJI_OK)
    {
        return;
    }
//	if (feedback.online == false)
//	{
//		return;
//	}
	/* 在任务里按需更新 angle_deg，不在 CAN 中断里做浮点运算。 */
	DVE_DjiMotor_UpdateAngleDeg(&feedback);	
    current_angle_deg = feedback.angle_deg;

    /* 第一次收到反馈时只建立基准，不产生角度突变。 */
    if (s_hit_motor[index].angle_inited == 0U)
    {
        s_hit_motor[index].last_angle_deg = current_angle_deg;
        s_hit_motor[index].total_angle = 0.0f;
        s_hit_motor[index].angle_inited = 1U;
    }

    delta_angle_deg = current_angle_deg - s_hit_motor[index].last_angle_deg;

    /* 单圈角度跨 0/360 时做修正，得到连续多圈角度。 */
    if (delta_angle_deg > 180.0f)
    {
        delta_angle_deg -= 360.0f;
    }
    else if (delta_angle_deg < -180.0f)
    {
        delta_angle_deg += 360.0f;
    }

    s_hit_motor[index].total_angle += delta_angle_deg;
    s_hit_motor[index].last_angle_deg = current_angle_deg;

    s_hit_motor[index].angle_pid.get = s_hit_motor[index].total_angle;
    s_hit_motor[index].speed_pid.get = (float)feedback.speed_rpm;
}

/*
*********************************************************************************************************
*   函 数 名: Hit_UpdateTargetByState
*   功能说明: 根据击球状态机更新三个击球电机目标角度
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void Hit_UpdateTargetByState(void)
{
    uint8_t all_done;

    switch (s_hit_state)
    {
    case HIT_STATE_PUT_ANGLE:
        /* SF 按下时，直接使用 SWC 选择的当前预设角度。 */
        for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++)
        {
            s_hit_motor[i].target_angle = s_hit_angle_table[s_hit_preset_index][i];
        }
        break;

    case HIT_STATE_RETURN:
        /* SF 松开后，不突变目标角度，而是逐步回到回零角度。 */
        all_done = 1U;
        for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++)
        {
            if (Hit_MoveTargetSmooth(i, s_hit_reset_angle[i], HIT_RETURN_STEP) == 0U)
            {
                all_done = 0U;
            }
        }

        if (all_done != 0U)
        {
            s_hit_state = HIT_STATE_IDLE;
        }
        break;

    case HIT_STATE_IDLE:
    default:
        break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: Hit_MoveTargetSmooth
*   功能说明: 将指定击球电机目标角度按固定步长平滑逼近目标值
*   形    参: index             - 击球电机下标
*             target            - 目标累计角度
*             step_size         - 每周期最大变化量
*   返 回 值: 1                 - 已到达目标
*             0                 - 仍在逼近目标
*********************************************************************************************************
*/
static uint8_t Hit_MoveTargetSmooth(uint8_t index, float target, float step_size)
{
    float diff;

    if (index >= HIT_MOTOR_NUM)
    {
        return 1U;
    }

    diff = target - s_hit_motor[index].target_angle;

    if (diff > step_size)
    {
        s_hit_motor[index].target_angle += step_size;
        return 0U;
    }

    if (diff < -step_size)
    {
        s_hit_motor[index].target_angle -= step_size;
        return 0U;
    }

    s_hit_motor[index].target_angle = target;
    return 1U;
}

/*
*********************************************************************************************************
*   函 数 名: Hit_OutputControl
*   功能说明: 计算 0x205~0x208 输出电流并发送 0x1FF 控制帧
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void Hit_OutputControl(void)
{
    int32_t output;

    /* 0x205 使用速度环，控制机械臂/俯仰电机。 */
    if (DVE_DjiMotor_IsOnline(DJI_MOTOR_PITCH, HIT_MOTOR_OFFLINE_MS) == true)
    {
        output = (int32_t)pid_calc(&s_pitch_speed_pid,
                                   s_pitch_speed_pid.get,
                                   s_pitch_speed_set);
        output = Hit_LimitOutput(output, HIT_PITCH_OUTPUT_LIMIT);
        (void)DVE_DjiMotor_SetCurrent(DJI_MOTOR_PITCH, output);
    }
    else
    {
        (void)DVE_DjiMotor_SetCurrent(DJI_MOTOR_PITCH, 0);
    }

    /* 0x206~0x208 使用角度环 + 速度环串级 PID。 */
    for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++)
    {
        if (DVE_DjiMotor_IsOnline(s_hit_motor[i].dji_id, HIT_MOTOR_OFFLINE_MS) == true)
        {
            output = PID_PROCESS_Double(&s_hit_motor[i].angle_pid,
                                        &s_hit_motor[i].speed_pid,
                                        s_hit_motor[i].target_angle,
                                        s_hit_motor[i].angle_pid.get,
                                        s_hit_motor[i].speed_pid.get);
            output = Hit_LimitOutput(output, HIT_OUTPUT_LIMIT);
            (void)DVE_DjiMotor_SetCurrent(s_hit_motor[i].dji_id, output);
        }
        else
        {
            (void)DVE_DjiMotor_SetCurrent(s_hit_motor[i].dji_id, 0);
        }
    }

    (void)DVE_DjiMotor_SendCurrentGroup(DVE_DJI_TX_GROUP_1FF);
}

/*
*********************************************************************************************************
*   函 数 名: Hit_OutputStop
*   功能说明: 清除并发送 0x1FF 组零电流输出
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void Hit_OutputStop(void)
{
    /* 只清击球控制组，不能影响底盘 0x200 组。 */
    (void)DVE_DjiMotor_ClearGroupCurrent(DVE_DJI_TX_GROUP_1FF);
    (void)DVE_DjiMotor_SendCurrentGroup(DVE_DJI_TX_GROUP_1FF);
}

/*
*********************************************************************************************************
*   函 数 名: Hit_LimitOutput
*   功能说明: 对输出电流控制值进行限幅
*   形    参: value             - 输入输出值
*             limit             - 正负限幅值
*   返 回 值: int16_t            - 限幅后的输出值
*********************************************************************************************************
*/
static int16_t Hit_LimitOutput(int32_t value, int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return (int16_t)value;
}
