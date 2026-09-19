/**
  ******************************************************************************
  * @file    dve_dji_motor.c
  * @brief   DJI 协议电机设备层实现
  *
  * @note
  *          本文件只负责 DJI 协议电机的反馈解析、状态缓存和电流控制帧打包。
  *          当前工程中：
  *          CAN2 ID 0x201~0x204 为底盘 3508；
  *
  *          本层不保存 PID，不处理底盘/击球状态机。
  *          上层 Task 周期读取反馈、计算 PID，再调用本层设置电流并发送。
  ******************************************************************************
  */

#include "dve_dji_motor.h"
#include <string.h>

/* 私有类型定义 --------------------------------------------------------------*/

typedef struct
{
    uint16_t feedback_id;           /* 真实反馈 ID，例如 0x201 */
    uint16_t tx_group_id;           /* 控制帧 ID：0x200 或 0x1FF */
    uint8_t tx_index;               /* 在 8 字节控制帧中的第几个 int16_t 位置 */
} DVE_DjiMotorConfig_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;        /* 所属 CAN 句柄 */
    uint16_t feedback_id;           /* 真实反馈 ID */
    uint16_t tx_group_id;           /* 所属控制帧 ID */
    uint8_t tx_index;               /* 控制帧内下标 0~3 */

    DVE_DjiMotorFeedback_t feedback;/* 最近一次反馈缓存 */
    int16_t output_current;         /* 待发送电流缓存 */
} DVE_DjiMotorObj_t;

/* 私有常量 ------------------------------------------------------------------*/

static const DVE_DjiMotorConfig_t s_dji_motor_cfg[DJI_MOTOR_NUM] =
{
    [DJI_MOTOR_CHASSIS_1] = {0x201U, DVE_DJI_TX_ID_GROUP_200, 0U},
    [DJI_MOTOR_CHASSIS_2] = {0x202U, DVE_DJI_TX_ID_GROUP_200, 1U},
    [DJI_MOTOR_CHASSIS_3] = {0x203U, DVE_DJI_TX_ID_GROUP_200, 2U},
    [DJI_MOTOR_CHASSIS_4] = {0x204U, DVE_DJI_TX_ID_GROUP_200, 3U},

    [DJI_MOTOR_PITCH]     = {0x205U, DVE_DJI_TX_ID_GROUP_1FF, 0U},
    [DJI_MOTOR_HIT_1]     = {0x206U, DVE_DJI_TX_ID_GROUP_1FF, 1U},
    [DJI_MOTOR_HIT_2]     = {0x207U, DVE_DJI_TX_ID_GROUP_1FF, 2U},
    [DJI_MOTOR_HIT_3]     = {0x208U, DVE_DJI_TX_ID_GROUP_1FF, 3U},
};

/* 私有变量 ------------------------------------------------------------------*/

static DVE_DjiMotorObj_t s_dji_motor[DJI_MOTOR_NUM];
static CAN_HandleTypeDef *s_dji_hcan = NULL;
static bool s_dji_inited = false;

/* 私有函数声明 --------------------------------------------------------------*/

static bool is_valid_id(DVE_DjiMotorId_t id);
static int16_t clamp_current(int32_t current);
static int8_t find_motor_by_feedback_id(uint16_t std_id);
static void pack_current_data(uint8_t data[8], const int16_t current[4]);
static void lock_irq(uint32_t *primask);
static void unlock_irq(uint32_t primask);
static DVE_DjiMotorStatus_t can_status_to_dji_status(BSP_CAN_Status_t status);

/* 函数定义 ------------------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_Init
*   功能说明: 初始化 DJI 电机对象表，绑定 CAN 句柄、反馈 ID 和控制帧位置
*   形    参: hcan              - DJI 电机所在 CAN 句柄，当前工程通常传入 &hcan2
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return DVE_DJI_ERR_PARAM;
    }

    /* 保存 DJI 电机所在 CAN 句柄，后续发送控制帧和接收过滤都会用到。 */
    s_dji_hcan = hcan;

    /* 根据静态配置表初始化每一个 DJI 电机的软件对象。 */
    for (uint8_t i = 0U; i < (uint8_t)DJI_MOTOR_NUM; i++)
    {
        s_dji_motor[i].hcan = hcan;
        s_dji_motor[i].feedback_id = s_dji_motor_cfg[i].feedback_id;
        s_dji_motor[i].tx_group_id = s_dji_motor_cfg[i].tx_group_id;
        s_dji_motor[i].tx_index = s_dji_motor_cfg[i].tx_index;

        /* 初始化时输出电流清零，反馈缓存清零。 */
        s_dji_motor[i].output_current = 0;
        memset(&s_dji_motor[i].feedback, 0, sizeof(DVE_DjiMotorFeedback_t));
    }

    s_dji_inited = true;

    return DVE_DJI_OK;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_RxCallback
*   功能说明: DJI 电机 CAN 反馈解析入口，在 CAN 接收中断分发函数中调用
*   形    参: hcan              - 触发接收中断的 CAN 句柄
*             rx_header         - CAN 接收帧头，包含 StdId、IDE、RTR、DLC 等信息
*             rx_data           - CAN 接收数据区，经典 CAN 最大 8 字节
*   返 回 值: true              - 已识别并解析 DJI 电机反馈帧
*             false             - 不是 DJI 电机反馈帧或参数异常
*********************************************************************************************************
*/
bool DVE_DjiMotor_RxCallback(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *rx_header,
                             const uint8_t rx_data[8])
{
    uint16_t std_id;
    int8_t motor_index;
    DVE_DjiMotorFeedback_t fb;

    if ((hcan == NULL) || (rx_header == NULL) || (rx_data == NULL))
    {
        return false;
    }

    /* 如果模块已经初始化，只接受绑定 CAN 句柄上的 DJI 电机反馈。 */
    if ((s_dji_inited == true) && (hcan != s_dji_hcan))
    {
        return false;
    }

    /* DJI 电机反馈必须是标准数据帧，且长度固定为 8 字节。 */
    if ((rx_header->IDE != CAN_ID_STD) ||
        (rx_header->RTR != CAN_RTR_DATA) ||
        (rx_header->DLC != DVE_DJI_FEEDBACK_LEN))
    {
        return false;
    }

    std_id = (uint16_t)rx_header->StdId;

    /* 当前只接收 0x201~0x208，其他 ID 直接丢弃。 */
    if ((std_id < DVE_DJI_STDID_MIN) || (std_id > DVE_DJI_STDID_MAX))
    {
        return false;
    }

    /* 根据真实反馈 ID 找到软件对象下标。 */
    motor_index = find_motor_by_feedback_id(std_id);
    if (motor_index < 0)
    {
        return false;
    }

    /*
     * DJI 反馈帧格式：
     * data[0..1]：机械角度 ecd，范围 0~8191；
     * data[2..3]：转速 speed_rpm；
     * data[4..5]：反馈转矩电流 current；
     * data[6]   ：电机温度 temp；
     * data[7]   ：保留。
     */
    fb.ecd = (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
	//    fb.angle_deg = (float)fb.ecd * 360.0f / DVE_DJI_ECD_RANGE;		//不直接浮点换算避免中断过长，要使用角度时先用DVE_DjiMotor_UpdateAngleDeg
    fb.speed_rpm = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    fb.current = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    fb.temp = rx_data[6];
    fb.last_update_ms = HAL_GetTick();
    fb.online = true;

    /* 反馈由中断更新，写入内容较少，保持中断函数轻量。 */
    s_dji_motor[(uint8_t)motor_index].feedback = fb;

    return true;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_GetFeedback
*   功能说明: 读取指定 DJI 电机的反馈副本，避免上层直接访问内部静态对象
*   形    参: id                - DJI 电机软件编号
*             feedback          - 用于保存反馈副本的指针
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_GetFeedback(DVE_DjiMotorId_t id,
                                              DVE_DjiMotorFeedback_t *feedback)
{
    uint32_t primask;

    if ((feedback == NULL) || (is_valid_id(id) == false))
    {
        return DVE_DJI_ERR_PARAM;
    }

    if (s_dji_inited == false)
    {
        return DVE_DJI_ERR_NOT_INIT;
    }

    /*
     * 反馈缓存由 CAN 中断更新。
     * 这里短暂关中断，只复制一份结构体，避免读到一半被中断改写。
     */
    lock_irq(&primask);
    *feedback = s_dji_motor[(uint8_t)id].feedback;
    unlock_irq(primask);

    return DVE_DJI_OK;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_UpdateAngleDeg
*   功能说明: 根据反馈结构体中的 ecd 更新 angle_deg
*   形    参: fb                - DJI 电机反馈结构体指针
*   返 回 值: DVE_DJI_OK        - 更新成功
*             DVE_DJI_ERROR     - 参数错误
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_UpdateAngleDeg(DVE_DjiMotorFeedback_t *fb)
{
    if (fb == NULL)
    {
        return DVE_DJI_ERR_PARAM;
    }

    fb->angle_deg = ((float)fb->ecd * 360.0f) / DVE_DJI_ECD_RANGE;

    return DVE_DJI_OK;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_GetSpeedRpm
*   功能说明: 快速读取指定 DJI 电机转速
*   形    参: id                - DJI 电机软件编号
*   返 回 值: int16_t            - 电机转速，单位 rpm；读取失败时返回 0
*********************************************************************************************************
*/
int16_t DVE_DjiMotor_GetSpeedRpm(DVE_DjiMotorId_t id)
{
    DVE_DjiMotorFeedback_t feedback;

    if (DVE_DjiMotor_GetFeedback(id, &feedback) != DVE_DJI_OK)
    {
        return 0;
    }

    return feedback.speed_rpm;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_IsOnline
*   功能说明: 根据最近一次反馈时间判断电机是否在线
*   形    参: id                - DJI 电机软件编号
*             timeout_ms        - 超时时间，单位 ms
*   返 回 值: true              - 在线
*             false             - 离线或参数异常
*********************************************************************************************************
*/
bool DVE_DjiMotor_IsOnline(DVE_DjiMotorId_t id, uint32_t timeout_ms)
{
    DVE_DjiMotorFeedback_t feedback;
    uint32_t now_ms;

    if (DVE_DjiMotor_GetFeedback(id, &feedback) != DVE_DJI_OK)
    {
        return false;
    }

    if (feedback.online == false)
    {
        return false;
    }

    now_ms = HAL_GetTick();

    return ((now_ms - feedback.last_update_ms) <= timeout_ms);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_SetCurrent
*   功能说明: 设置指定 DJI 电机的输出电流缓存，不立即发送 CAN 帧
*   形    参: id                - DJI 电机软件编号
*             current           - 期望输出电流控制值，内部会限幅到 ±DVE_DJI_CURRENT_MAX
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_SetCurrent(DVE_DjiMotorId_t id,
                                             int32_t current)
{
    uint32_t primask;

    if (is_valid_id(id) == false)
    {
        return DVE_DJI_ERR_PARAM;
    }

    if (s_dji_inited == false)
    {
        return DVE_DJI_ERR_NOT_INIT;
    }

    /* 输出电流缓存可能被发送函数读取，写入时短暂保护。 */
    lock_irq(&primask);
    s_dji_motor[(uint8_t)id].output_current = clamp_current(current);
    unlock_irq(primask);

    return DVE_DJI_OK;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_ClearCurrent
*   功能说明: 清除指定 DJI 电机的输出电流缓存
*   形    参: id                - DJI 电机软件编号
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_ClearCurrent(DVE_DjiMotorId_t id)
{
    return DVE_DjiMotor_SetCurrent(id, 0);
}
/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_ClearGroupCurrent
*   功能说明: 清除指定 DJI 控制组的输出电流缓存
*   形    参: group             - DVE_DJI_TX_GROUP_200 或 DVE_DJI_TX_GROUP_1FF
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_ClearGroupCurrent(DVE_DjiTxGroup_t group)
{
    uint16_t tx_id;
    uint32_t primask;

    if (s_dji_inited == false)
    {
        return DVE_DJI_ERR_NOT_INIT;
    }

    if (group == DVE_DJI_TX_GROUP_200)
    {
        tx_id = DVE_DJI_TX_ID_GROUP_200;
    }
    else if (group == DVE_DJI_TX_GROUP_1FF)
    {
        tx_id = DVE_DJI_TX_ID_GROUP_1FF;
    }
    else
    {
        return DVE_DJI_ERR_PARAM;
    }

    /* 只清除指定控制组，避免底盘保护误清击球组，或击球保护误清底盘组。 */
    lock_irq(&primask);
    for (uint8_t i = 0U; i < (uint8_t)DJI_MOTOR_NUM; i++)
    {
        if (s_dji_motor[i].tx_group_id == tx_id)
        {
            s_dji_motor[i].output_current = 0;
        }
    }
    unlock_irq(primask);

    return DVE_DJI_OK;
}
/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_ClearAllCurrent
*   功能说明: 清除所有 DJI 电机的输出电流缓存
*   形    参: 无
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_ClearAllCurrent(void)
{
    uint32_t primask;

    if (s_dji_inited == false)
    {
        return DVE_DJI_ERR_NOT_INIT;
    }

    /* 清空全部输出缓存，常用于急停、失控保护或初始化阶段。 */
    lock_irq(&primask);
    for (uint8_t i = 0U; i < (uint8_t)DJI_MOTOR_NUM; i++)
    {
        s_dji_motor[i].output_current = 0;
    }
    unlock_irq(primask);

    return DVE_DJI_OK;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_SendCurrentGroup
*   功能说明: 发送一组 DJI 电流控制帧，可发送 0x200 或 0x1FF
*   形    参: group             - DVE_DJI_TX_GROUP_200 或 DVE_DJI_TX_GROUP_1FF
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_SendCurrentGroup(DVE_DjiTxGroup_t group)
{
    uint16_t tx_id;
    int16_t current[DVE_DJI_GROUP_MOTOR_NUM] = {0};
    uint8_t data[DVE_DJI_CONTROL_LEN] = {0};
    uint32_t primask;

    if (s_dji_inited == false)
    {
        return DVE_DJI_ERR_NOT_INIT;
    }

    /* 根据组号选择真实 CAN 控制帧 ID。 */
    if (group == DVE_DJI_TX_GROUP_200)
    {
        tx_id = DVE_DJI_TX_ID_GROUP_200;
    }
    else if (group == DVE_DJI_TX_GROUP_1FF)
    {
        tx_id = DVE_DJI_TX_ID_GROUP_1FF;
    }
    else
    {
        return DVE_DJI_ERR_PARAM;
    }

    /*
     * 从内部电流缓存中取出属于本组控制帧的电机。
     * 0x200 控制 0x201~0x204；0x1FF 控制 0x205~0x208。
     */
    lock_irq(&primask);
    for (uint8_t i = 0U; i < (uint8_t)DJI_MOTOR_NUM; i++)
    {
        if ((s_dji_motor[i].tx_group_id == tx_id) &&
            (s_dji_motor[i].tx_index < DVE_DJI_GROUP_MOTOR_NUM))
        {
            current[s_dji_motor[i].tx_index] = s_dji_motor[i].output_current;
        }
    }
    unlock_irq(primask);

    /* 将 4 个 int16_t 电流值打包为 8 字节 CAN 数据。 */
    pack_current_data(data, current);

    return can_status_to_dji_status(BSP_CAN_SendStd8(s_dji_hcan, tx_id, data));
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DjiMotor_SendAllCurrent
*   功能说明: 依次发送 0x200 和 0x1FF 两组 DJI 电流控制帧
*   形    参: 无
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
DVE_DjiMotorStatus_t DVE_DjiMotor_SendAllCurrent(void)
{
    DVE_DjiMotorStatus_t status;

    status = DVE_DjiMotor_SendCurrentGroup(DVE_DJI_TX_GROUP_200);
    if (status != DVE_DJI_OK)
    {
        return status;
    }

    return DVE_DjiMotor_SendCurrentGroup(DVE_DJI_TX_GROUP_1FF);
}

/* 私有函数定义 --------------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: is_valid_id
*   功能说明: 判断 DJI 电机软件编号是否合法
*   形    参: id                - DJI 电机软件编号
*   返 回 值: true              - 合法
*             false             - 非法
*********************************************************************************************************
*/
static bool is_valid_id(DVE_DjiMotorId_t id)
{
    return ((uint32_t)id < (uint32_t)DJI_MOTOR_NUM);
}

/*
*********************************************************************************************************
*   函 数 名: clamp_current
*   功能说明: 对 DJI 电机电流控制值进行限幅
*   形    参: current           - 输入电流控制值
*   返 回 值: int16_t            - 限幅后的电流控制值
*********************************************************************************************************
*/
static int16_t clamp_current(int32_t current)
{
    if (current > DVE_DJI_CURRENT_MAX)
    {
        return DVE_DJI_CURRENT_MAX;
    }

    if (current < -DVE_DJI_CURRENT_MAX)
    {
        return -DVE_DJI_CURRENT_MAX;
    }

    return (int16_t)current;
}

/*
*********************************************************************************************************
*   函 数 名: find_motor_by_feedback_id
*   功能说明: 根据真实 CAN 反馈 ID 查找 DJI 电机软件对象下标
*   形    参: std_id            - CAN 标准帧 ID，例如 0x201
*   返 回 值: >=0               - 对应的软件对象下标
*             -1                - 未找到
*********************************************************************************************************
*/
static int8_t find_motor_by_feedback_id(uint16_t std_id)
{
    for (uint8_t i = 0U; i < (uint8_t)DJI_MOTOR_NUM; i++)
    {
        if (s_dji_motor[i].feedback_id == std_id)
        {
            return (int8_t)i;
        }
    }

    return -1;
}

/*
*********************************************************************************************************
*   函 数 名: pack_current_data
*   功能说明: 将 4 个 int16_t 电流值打包成 DJI 电调控制帧的 8 字节数据
*   形    参: data              - 输出 8 字节数据缓存
*             current           - 4 个电机电流值，顺序对应控制帧中的 4 个位置
*   返 回 值: 无
*********************************************************************************************************
*/
static void pack_current_data(uint8_t data[8], const int16_t current[4])
{
    uint16_t raw;

    if ((data == NULL) || (current == NULL))
    {
        return;
    }

    for (uint8_t i = 0U; i < DVE_DJI_GROUP_MOTOR_NUM; i++)
    {
        raw = (uint16_t)current[i];
        data[2U * i]      = (uint8_t)((raw >> 8) & 0xFFU);
        data[2U * i + 1U] = (uint8_t)(raw & 0xFFU);
    }
}

/*
*********************************************************************************************************
*   函 数 名: lock_irq
*   功能说明: 保存当前中断状态并关闭全局中断，用于短临界区保护
*   形    参: primask           - 用于保存 PRIMASK 的指针
*   返 回 值: 无
*********************************************************************************************************
*/
static void lock_irq(uint32_t *primask)
{
    if (primask != NULL)
    {
        *primask = __get_PRIMASK();
        __disable_irq();
    }
}

/*
*********************************************************************************************************
*   函 数 名: unlock_irq
*   功能说明: 根据进入临界区前的 PRIMASK 状态恢复全局中断
*   形    参: primask           - 进入临界区前保存的 PRIMASK
*   返 回 值: 无
*********************************************************************************************************
*/
static void unlock_irq(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/*
*********************************************************************************************************
*   函 数 名: can_status_to_dji_status
*   功能说明: 将 BSP_CAN 层返回值转换为 DVE_DJI 层返回值，避免上层直接依赖 BSP 状态码
*   形    参: status            - BSP_CAN_Status_t 返回值
*   返 回 值: DVE_DjiMotorStatus_t
*********************************************************************************************************
*/
static DVE_DjiMotorStatus_t can_status_to_dji_status(BSP_CAN_Status_t status)
{
    if (status == BSP_CAN_OK)
    {
        return DVE_DJI_OK;
    }

    if (status == BSP_CAN_ERR_PARAM)
    {
        return DVE_DJI_ERR_PARAM;
    }

    return DVE_DJI_ERR_CAN;
}
