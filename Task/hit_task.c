
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
    HIT_STATE_WAIT_REMOTE,           /* 等待安全拨杆确认，0x1FF组保持零电流 */
    HIT_STATE_POSITION_FAULT,        /* 相对位置不可信，完整重启前锁定整个击打组 */
    HIT_STATE_DISABLE                  /* 停机保护，0x1FF 组输出清零 */
} Hit_State_t;

typedef struct
{
    DVE_DjiMotorId_t dji_id;           /* 对应 DVE_DjiMotor 软件编号 */
    pid_t angle_pid;                   /* 击球电机角度环 PID */
    pid_t speed_pid;                   /* 击球电机速度环 PID */
    float target_angle;                /* 当前目标累计角度 */
    float total_angle;                 /* 多圈累计角度反馈 */
    uint8_t angle_inited;              /* 是否读取过有效位置反馈；计圈由DVE接收路径负责 */
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
// 三个位置电机均有有效首帧且反馈新鲜，才允许重新使能击打组。
static bool s_positions_ready = false;
// 已观察到安全拨杆确认；任何新的命令失控都会清除，位置故障不能靠它解除。
static bool s_remote_confirmed = false;

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
static void Hit_ResetPids(void);
static void Hit_EnterStop(Hit_State_t state);
static void Hit_ControlStep(void);
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
*   功能说明: 创建本模块命令队列与业务任务，并向调用者报告结果
*   形    参: 无；仅系统启动阶段调用一次
*   返 回 值: OK表示成功；ERR_QUEUE表示队列失败；ERR_TASK表示任务失败且队列已释放
*   说    明: 调用者保证创建阶段不被业务任务抢占；本模块不依赖启动模块或决定整机故障策略。
*********************************************************************************************************
*/
Task_CreateResult_t hit_task_creat(void)
{
    xHitCmdQueue = xQueueCreate(1, sizeof(Hit_Cmd_t));
    if (xHitCmdQueue == NULL) return TASK_CREATE_ERR_QUEUE;

    if (xTaskCreate((TaskFunction_t)hit_task,
                (const char *)"hit_task",
                (configSTACK_DEPTH_TYPE)HIT_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)HIT_TASK_PRIO,
                (TaskHandle_t *)&s_hit_task_handle) != pdPASS)
    {
        // 创建失败时只释放本模块刚分配的队列，不调用上层启动或故障处理函数。
        vQueueDelete(xHitCmdQueue);
        xHitCmdQueue = NULL;
        return TASK_CREATE_ERR_TASK;
    }
    return TASK_CREATE_OK;
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
    (void)parm;
    Hit_InitControl();
    last_wake_tick = xTaskGetTickCount();
    for (;;)
    {
        Hit_ControlStep();
        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(HIT_TASK_PERIOD_MS));
    }
}

/*
*********************************************************************************************************
*   函 数 名: Hit_ControlStep
*   功能说明: 执行一个10ms控制周期，分别处理遥控超时和位置故障
*   形    参: 无；读取最新命令队列及设备反馈快照
*   返 回 值: 无；失能、等待确认和位置故障状态均提交0x1FF零电流
*   说    明: 失能期间仍读取位置反馈；命令失控可确认恢复，位置故障只能完整重启恢复。
*********************************************************************************************************
*/
static void Hit_ControlStep(void)
{
    Hit_Cmd_t cmd;
    // 先检查位置可信度，避免当周期新遥控命令重新使能已故障的机构。
    Hit_UpdateFeedback();
    if (xQueueReceive(xHitCmdQueue, &cmd, 0U) == pdTRUE)
    {
        s_last_cmd_ms = HAL_GetTick();
        Hit_UpdateCommand(&cmd);
    }
    if ((HAL_GetTick() - s_last_cmd_ms) > HIT_CMD_TIMEOUT_MS)
        Hit_EnterStop(HIT_STATE_WAIT_REMOTE);
    if ((s_hit_state == HIT_STATE_DISABLE) ||
        (s_hit_state == HIT_STATE_WAIT_REMOTE) ||
        (s_hit_state == HIT_STATE_POSITION_FAULT))
        // 软件清零后仍需提交CAN帧；提交失败由BSP统计，下一周期继续提交当前零输出。
        Hit_OutputStop();
    else
    {
        Hit_UpdateTargetByState();
        Hit_OutputControl();
    }
}

/*
*********************************************************************************************************
*   函 数 名: Hit_ResetPids
*   功能说明: 保留控制增益和当前测量，清除击打组PID误差历史与积分输出
*   形    参: 无
*   返 回 值: 无
*   说    明: 不重置设备编码器原点；恢复当前测量仅用于后续计算，旧控制历史不会沿用。
*********************************************************************************************************
*/
static void Hit_ResetPids(void)
{
    float pitch_get = s_pitch_speed_pid.get;
    pid_reset(&s_pitch_speed_pid, s_pitch_speed_pid.kp,
              s_pitch_speed_pid.ki, s_pitch_speed_pid.kd);
    s_pitch_speed_pid.get = pitch_get;
    for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++)
    {
        float speed_get = s_hit_motor[i].speed_pid.get;
        pid_reset(&s_hit_motor[i].angle_pid, s_hit_motor[i].angle_pid.kp,
                  s_hit_motor[i].angle_pid.ki, s_hit_motor[i].angle_pid.kd);
        pid_reset(&s_hit_motor[i].speed_pid, s_hit_motor[i].speed_pid.kp,
                  s_hit_motor[i].speed_pid.ki, s_hit_motor[i].speed_pid.kd);
        s_hit_motor[i].angle_pid.get = s_hit_motor[i].total_angle;
        s_hit_motor[i].speed_pid.get = speed_get;
    }
}

/*
*********************************************************************************************************
*   函 数 名: Hit_EnterStop
*   功能说明: 进入失能状态，清PID历史、0x205目标速度和遥控确认
*   形    参: state - 待进入的停机状态
*   返 回 值: 无；已有位置故障时保持故障锁存
*   说    明: 本函数更新状态，实际零电流帧由当周期Hit_OutputStop提交。
*********************************************************************************************************
*/
static void Hit_EnterStop(Hit_State_t state)
{
    // 相对位置已不可信时，遥控恢复和其他停机原因都不能覆盖故障锁存。
    if (s_hit_state == HIT_STATE_POSITION_FAULT) return;
    if (s_hit_state != state) Hit_ResetPids();
    s_hit_state = state;
    s_pitch_speed_set = HIT_PITCH_SPEED_ZERO;
    s_remote_confirmed = false;
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
        s_hit_motor[i].angle_inited = 0U;
    }

    s_hit_preset_index = 0U;
    s_pitch_speed_set = HIT_PITCH_SPEED_ZERO;
    s_last_cmd_ms = HAL_GetTick();
    s_positions_ready = false;
    s_remote_confirmed = false;
    // 上电不直接保持回零角，先等待安全遥控确认及三个真实位置反馈。
    s_hit_state = HIT_STATE_DISABLE;
}

/*
*********************************************************************************************************
*   函 数 名: Hit_UpdateCommand
*   功能说明: 处理最新击打命令、安全拨杆确认和重新使能目标
*   形    参: cmd - RC任务发布的最新命令；pitch_speed_set单位rpm
*   返 回 值: 无；位置故障保持锁存，未确认或反馈未就绪时继续停机
*   说    明: 重新使能时清PID历史，并从实际位置开始回准备位；确认帧本身不产生动作。
*********************************************************************************************************
*/
static void Hit_UpdateCommand(const Hit_Cmd_t *cmd)
{
    // 位置故障必须完整重启；新的有效遥控命令也不能解除。
    if ((cmd == NULL) || (s_hit_state == HIT_STATE_POSITION_FAULT)) return;
    if (cmd->enable == 0U)
    {
        Hit_EnterStop(HIT_STATE_WAIT_REMOTE);
        // RC确认帧允许记录安全条件，但本帧enable=0仍保持停机。
        if ((cmd->safe_to_arm != 0U) && (cmd->hit_pressed == 0U) &&
            (cmd->pitch_speed_set == HIT_PITCH_SPEED_ZERO)) s_remote_confirmed = true;
        return;
    }
    if ((s_hit_state == HIT_STATE_DISABLE) || (s_hit_state == HIT_STATE_WAIT_REMOTE))
    {
        // 覆盖队列可能跳过单帧确认；未观察到确认时继续停机，直到安全拨杆状态被读取。
        if (!s_remote_confirmed)
        {
            // 未观察到独立确认帧时，安全的使能帧也仅确认一次，下一帧再允许动作。
            if ((cmd->safe_to_arm != 0U) && (cmd->hit_pressed == 0U) &&
                (cmd->pitch_speed_set == HIT_PITCH_SPEED_ZERO)) s_remote_confirmed = true;
            return;
        }
        // 安全拨杆确认和三个电机反馈就绪缺一不可，不使用全零缓存建立机构位置。
        if (!s_positions_ready) return;
        Hit_ResetPids();
        for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++)
            // 从失能期间测得的实际位置开始回零，不跳回失控前的旧目标。
            s_hit_motor[i].target_angle = s_hit_motor[i].total_angle;
        s_hit_state = HIT_STATE_RETURN;
    }
    s_hit_preset_index = (cmd->preset_index < HIT_PRESET_NUM) ?
                         cmd->preset_index : (HIT_PRESET_NUM - 1U);
    s_pitch_speed_set = cmd->pitch_speed_set;
    if (cmd->hit_pressed != 0U) s_hit_state = HIT_STATE_PUT_ANGLE;
    else if (s_hit_state != HIT_STATE_IDLE) s_hit_state = HIT_STATE_RETURN;
}

/*
*********************************************************************************************************
*   函 数 名: Hit_UpdateFeedback
*   功能说明: 读取0x205速度及三个击打位置反馈，更新当周期就绪状态
*   形    参: 无
*   返 回 值: 无；0x205离线清其PID，任一位置反馈失效由子函数锁定击打组
*   说    明: 失能期间仍读取位置，保持遥控恢复后的相对原点连续性。
*********************************************************************************************************
*/
static void Hit_UpdateFeedback(void)
{
    DVE_DjiMotorFeedback_t pitch_feedback;
    s_positions_ready = true;
    if ((DVE_DjiMotor_GetFeedback(DJI_MOTOR_PITCH, &pitch_feedback) == DVE_DJI_OK) &&
        pitch_feedback.online &&
        ((HAL_GetTick() - pitch_feedback.last_update_ms) <= HIT_MOTOR_OFFLINE_MS))
        s_pitch_speed_pid.get = (float)pitch_feedback.speed_rpm;
    else
        pid_reset(&s_pitch_speed_pid, s_pitch_speed_pid.kp,
                  s_pitch_speed_pid.ki, s_pitch_speed_pid.kd);
    for (uint8_t i = 0U; i < HIT_MOTOR_NUM; i++) Hit_UpdateHitMotorFeedback(i);
}

/*
*********************************************************************************************************
*   函 数 名: Hit_UpdateHitMotorFeedback
*   功能说明: 读取单个击打位置电机的逐帧累计反馈并检查可信度
*   形    参: index - 击打电机下标0~2，对应0x206~0x208
*   返 回 值: 无；无首帧时等待，位置失效或超过100ms未反馈时锁定击打组
*   说    明: total_angle为相对首个真实反馈的电机轴角度，单位deg，不是绝对机械零点。
*********************************************************************************************************
*/
static void Hit_UpdateHitMotorFeedback(uint8_t index)
{
    DVE_DjiMotorFeedback_t feedback;
    if (index >= HIT_MOTOR_NUM) return;
    if ((DVE_DjiMotor_GetFeedback(s_hit_motor[index].dji_id, &feedback) != DVE_DJI_OK) ||
        !feedback.online)
    {
        s_positions_ready = false;
        return;
    }
    // 超时、半圈歧义或累计溢出影响机械参考，锁定整个0x1FF组而非只停单个位置电机。
    if (!feedback.position_valid ||
        ((HAL_GetTick() - feedback.last_update_ms) > HIT_MOTOR_OFFLINE_MS))
    {
        s_positions_ready = false;
        Hit_EnterStop(HIT_STATE_POSITION_FAULT);
        return;
    }
    s_hit_motor[index].angle_inited = 1U;
    // 编码器累计计数转换为电机轴角度deg；机构目标已乘HIT_SCALE，任务不再自行计圈。
    s_hit_motor[index].total_angle = (float)feedback.total_ecd * 360.0f / DVE_DJI_ECD_RANGE;
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
        pid_reset(&s_pitch_speed_pid, s_pitch_speed_pid.kp,
                  s_pitch_speed_pid.ki, s_pitch_speed_pid.kd);
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
            Hit_EnterStop(HIT_STATE_POSITION_FAULT);
            // 软件清零后仍需提交CAN帧；提交失败由BSP统计，下一周期继续提交当前零输出。
        Hit_OutputStop();
            return;
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
