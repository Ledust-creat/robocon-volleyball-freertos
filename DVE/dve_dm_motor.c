/**
  ******************************************************************************
  * @file    dve_dm_motor.c
  * @brief   达妙 DM 电机设备层实现
  *
  * @note
  *          本文件属于 DVE(Device) 层，只负责达妙电机 CAN 协议封装。
  *          外部只通过 DM_MOTOR_xxx 编号控制电机，不需要维护 DVE_DmMotor_t 对象。
  *
  *          当前保留：
  *          1. 使能 / 失能 / 设零点；
  *          2. MIT 模式；
  *          3. 速度模式；
  *          4. 位置模式；
  *          5. 反馈回调接口预留，暂不解析反馈。
  ******************************************************************************
  */

#include "dve_dm_motor.h"
#include <string.h>

/* 私有宏定义 ----------------------------------------------------------------*/

#define DVE_DM_CLAMP(x, min, max)    (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

/*
 * 当前默认 DM 电机 CAN ID。
 * 如果后续实际 ID 不是 0x00/0x01/0x02，只需要改这里，不需要改 Task 层。
 */
#define DVE_DM_LIFT_1_CAN_ID         (0x00U)
#define DVE_DM_LIFT_2_CAN_ID         (0x01U)
#define DVE_DM_LIFT_3_CAN_ID         (0x02U)

/* 私有类型定义 --------------------------------------------------------------*/

typedef struct
{
    uint16_t can_id;                 /* 达妙电机基础 CAN ID */
} DVE_DmMotorConfig_t;

/* 私有变量 ------------------------------------------------------------------*/

static CAN_HandleTypeDef *s_dm_hcan = NULL;

static const DVE_DmMotorConfig_t s_dm_motor_cfg[DM_MOTOR_NUM] =
{
    [DM_MOTOR_LIFT_1] = { DVE_DM_LIFT_1_CAN_ID },
    [DM_MOTOR_LIFT_2] = { DVE_DM_LIFT_2_CAN_ID },
    [DM_MOTOR_LIFT_3] = { DVE_DM_LIFT_3_CAN_ID },
};

/* 私有函数声明 --------------------------------------------------------------*/

static bool is_valid_id(DVE_DmMotorId_t id);
static uint16_t get_can_id(DVE_DmMotorId_t id);
static DVE_DmMotorStatus_t send_special_cmd(DVE_DmMotorId_t id, uint8_t cmd);
static void pack_float_le(uint8_t data[4], float value);
static DVE_DmMotorStatus_t can_status_to_dm_status(BSP_CAN_Status_t status);

/* 函数定义 ------------------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_Init
*   功能说明: 初始化 DM 电机模块，绑定 DM 电机所在 CAN 总线
*   形    参: hcan              - DM 电机所在 CAN 句柄，当前工程一般为 &hcan1
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
DVE_DmMotorStatus_t DVE_DmMotor_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return DVE_DM_ERR_PARAM;
    }

    /*
     * DM 电机对象由 DVE 层内部配置表管理。
     * 上层 Task 只需要使用 DM_MOTOR_LIFT_x 编号，不需要再定义结构体对象。
     */
    s_dm_hcan = hcan;

    return DVE_DM_OK;
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_Enable
*   功能说明: 发送 DM 电机使能命令，使电机进入可控制状态
*   形    参: id                - DM 电机软件编号
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
DVE_DmMotorStatus_t DVE_DmMotor_Enable(DVE_DmMotorId_t id)
{
    return send_special_cmd(id, DVE_DM_CMD_ENABLE);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_Disable
*   功能说明: 发送 DM 电机失能命令，关闭电机主动输出
*   形    参: id                - DM 电机软件编号
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
DVE_DmMotorStatus_t DVE_DmMotor_Disable(DVE_DmMotorId_t id)
{
    return send_special_cmd(id, DVE_DM_CMD_DISABLE);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_SetZero
*   功能说明: 发送 DM 电机设零点命令，将当前位置设置为零点
*   形    参: id                - DM 电机软件编号
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
DVE_DmMotorStatus_t DVE_DmMotor_SetZero(DVE_DmMotorId_t id)
{
    return send_special_cmd(id, DVE_DM_CMD_ZERO);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_SetMit
*   功能说明: 发送 DM 电机 MIT 模式控制命令
*   形    参: id                - DM 电机软件编号
*             pos_rad           - 目标位置，单位 rad
*             vel_rad_s         - 目标速度，单位 rad/s
*             kp                - 位置刚度
*             kd                - 速度阻尼
*             torque_nm         - 前馈力矩，单位 Nm
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
DVE_DmMotorStatus_t DVE_DmMotor_SetMit(DVE_DmMotorId_t id,
                                       float pos_rad,
                                       float vel_rad_s,
                                       float kp,
                                       float kd,
                                       float torque_nm)
{
    uint8_t data[8] = {0};
    uint16_t pos_tmp;
    uint16_t vel_tmp;
    uint16_t kp_tmp;
    uint16_t kd_tmp;
    uint16_t tor_tmp;
    uint16_t std_id;
    BSP_CAN_Status_t can_status;

    if (s_dm_hcan == NULL)
    {
        return DVE_DM_ERR_NOT_INIT;
    }

    if (is_valid_id(id) == false)
    {
        return DVE_DM_ERR_PARAM;
    }

    /*
     * 1. 物理量限幅。
     *    MIT 协议不是直接发送 float，而是把物理量压缩成整数。
     *    如果不先限幅，超范围数据会导致打包结果失真。
     */
    pos_rad   = DVE_DM_CLAMP(pos_rad,   DVE_DM_POS_MIN_RAD,   DVE_DM_POS_MAX_RAD);
    vel_rad_s = DVE_DM_CLAMP(vel_rad_s, DVE_DM_VEL_MIN_RAD_S, DVE_DM_VEL_MAX_RAD_S);
    kp        = DVE_DM_CLAMP(kp,        DVE_DM_KP_MIN,        DVE_DM_KP_MAX);
    kd        = DVE_DM_CLAMP(kd,        DVE_DM_KD_MIN,        DVE_DM_KD_MAX);
    torque_nm = DVE_DM_CLAMP(torque_nm, DVE_DM_TORQUE_MIN_NM, DVE_DM_TORQUE_MAX_NM);

    /*
     * 2. float 物理量压缩为 MIT 协议整数。
     *    pos:    16 bit
     *    vel:    12 bit
     *    kp:     12 bit
     *    kd:     12 bit
     *    torque: 12 bit
     */
    pos_tmp = float_to_uint(pos_rad,   DVE_DM_POS_MIN_RAD,   DVE_DM_POS_MAX_RAD,   DVE_DM_POS_BITS);
    vel_tmp = float_to_uint(vel_rad_s, DVE_DM_VEL_MIN_RAD_S, DVE_DM_VEL_MAX_RAD_S, DVE_DM_VEL_BITS);
    kp_tmp  = float_to_uint(kp,        DVE_DM_KP_MIN,        DVE_DM_KP_MAX,        DVE_DM_KP_BITS);
    kd_tmp  = float_to_uint(kd,        DVE_DM_KD_MIN,        DVE_DM_KD_MAX,        DVE_DM_KD_BITS);
    tor_tmp = float_to_uint(torque_nm, DVE_DM_TORQUE_MIN_NM, DVE_DM_TORQUE_MAX_NM, DVE_DM_TORQUE_BITS);

    /*
     * 3. 按达妙 MIT 协议打包 8 字节。
     *    data[0] : pos[15:8]
     *    data[1] : pos[7:0]
     *    data[2] : vel[11:4]
     *    data[3] : vel[3:0] + kp[11:8]
     *    data[4] : kp[7:0]
     *    data[5] : kd[11:4]
     *    data[6] : kd[3:0] + torque[11:8]
     *    data[7] : torque[7:0]
     */
    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)(pos_tmp & 0xFFU);
    data[2] = (uint8_t)(vel_tmp >> 4);
    data[3] = (uint8_t)(((vel_tmp & 0x0FU) << 4) | (kp_tmp >> 8));
    data[4] = (uint8_t)(kp_tmp & 0xFFU);
    data[5] = (uint8_t)(kd_tmp >> 4);
    data[6] = (uint8_t)(((kd_tmp & 0x0FU) << 4) | (tor_tmp >> 8));
    data[7] = (uint8_t)(tor_tmp & 0xFFU);

    /*
     * 4. MIT 模式使用基础 CAN ID：0x000 + can_id。
     */
    std_id = (uint16_t)(DVE_DM_CAN_ID_OFFSET_MIT + get_can_id(id));
    can_status = BSP_CAN_SendStd8(s_dm_hcan, std_id, data);

    return can_status_to_dm_status(can_status);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_SetSpeed
*   功能说明: 发送 DM 电机速度模式控制命令
*   形    参: id                - DM 电机软件编号
*             speed_rad_s       - 目标速度，单位 rad/s，实际含义以达妙协议配置为准
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
DVE_DmMotorStatus_t DVE_DmMotor_SetSpeed(DVE_DmMotorId_t id,
                                         float speed_rad_s)
{
    uint8_t data[4] = {0};
    uint16_t std_id;
    BSP_CAN_Status_t can_status;

    if (s_dm_hcan == NULL)
    {
        return DVE_DM_ERR_NOT_INIT;
    }

    if (is_valid_id(id) == false)
    {
        return DVE_DM_ERR_PARAM;
    }

    /*
     * 速度模式对应旧函数 Set_dm_speed():
     * StdId = 0x200 + ID
     * DLC   = 4
     * Data  = float speed 的小端字节序
     */
    pack_float_le(data, speed_rad_s);

    std_id = (uint16_t)(DVE_DM_CAN_ID_OFFSET_SPEED + get_can_id(id));
    can_status = BSP_CAN_SendStd(s_dm_hcan, std_id, data, DVE_DM_SPEED_FRAME_LEN);

    return can_status_to_dm_status(can_status);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_SetPosition
*   功能说明: 发送 DM 电机位置模式控制命令
*   形    参: id                - DM 电机软件编号
*             pos_rad           - 目标位置，单位 rad，实际含义以达妙协议配置为准
*             vel_rad_s         - 目标速度/速度约束，单位 rad/s，实际含义以达妙协议配置为准
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
DVE_DmMotorStatus_t DVE_DmMotor_SetPosition(DVE_DmMotorId_t id,
                                            float pos_rad,
                                            float vel_rad_s)
{
    uint8_t data[8] = {0};
    uint16_t std_id;
    BSP_CAN_Status_t can_status;

    if (s_dm_hcan == NULL)
    {
        return DVE_DM_ERR_NOT_INIT;
    }

    if (is_valid_id(id) == false)
    {
        return DVE_DM_ERR_PARAM;
    }

    /*
     * 位置模式对应旧函数 Set_dm_pos():
     * StdId = 0x100 + ID
     * DLC   = 8
     * Data[0..3] = float pos 的小端字节序
     * Data[4..7] = float vel 的小端字节序
     */
    pack_float_le(&data[0], pos_rad);
    pack_float_le(&data[4], vel_rad_s);

    std_id = (uint16_t)(DVE_DM_CAN_ID_OFFSET_POS + get_can_id(id));
    can_status = BSP_CAN_SendStd8(s_dm_hcan, std_id, data);

    return can_status_to_dm_status(can_status);
}

/*
*********************************************************************************************************
*   函 数 名: DVE_DmMotor_RxCallback
*   功能说明: DM 电机反馈解析入口，当前只预留接口，不解析反馈
*   形    参: hcan              - 触发接收中断的 CAN 句柄
*             rx_header         - CAN 接收帧头
*             rx_data           - CAN 接收数据区
*   返 回 值: false             - 当前未解析任何 DM 反馈帧
*********************************************************************************************************
*/
bool DVE_DmMotor_RxCallback(CAN_HandleTypeDef *hcan,
                            const CAN_RxHeaderTypeDef *rx_header,
                            const uint8_t rx_data[8])
{
    (void)hcan;
    (void)rx_header;
    (void)rx_data;

    /*
     * 反馈解析暂时不写。
     * 后续确认具体达妙型号、反馈 ID 和字节格式后，再在这里补充解析逻辑。
     */
    return false;
}

/* 私有函数定义 --------------------------------------------------------------*/

/*
*********************************************************************************************************
*   函 数 名: is_valid_id
*   功能说明: 判断 DM 电机软件编号是否合法
*   形    参: id                - DM 电机软件编号
*   返 回 值: true              - 编号合法
*             false             - 编号非法
*********************************************************************************************************
*/
static bool is_valid_id(DVE_DmMotorId_t id)
{
    return ((uint32_t)id < (uint32_t)DM_MOTOR_NUM);
}

/*
*********************************************************************************************************
*   函 数 名: get_can_id
*   功能说明: 根据 DM 电机软件编号获取真实基础 CAN ID
*   形    参: id                - DM 电机软件编号
*   返 回 值: uint16_t          - 真实基础 CAN ID
*********************************************************************************************************
*/
static uint16_t get_can_id(DVE_DmMotorId_t id)
{
    return s_dm_motor_cfg[id].can_id;
}

/*
*********************************************************************************************************
*   函 数 名: send_special_cmd
*   功能说明: 发送 DM 电机特殊命令，包括使能、失能、设零点
*   形    参: id                - DM 电机软件编号
*             cmd               - 特殊命令字，支持 0xFC/0xFD/0xFE
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
static DVE_DmMotorStatus_t send_special_cmd(DVE_DmMotorId_t id, uint8_t cmd)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U};
    uint16_t std_id;
    BSP_CAN_Status_t can_status;

    if (s_dm_hcan == NULL)
    {
        return DVE_DM_ERR_NOT_INIT;
    }

    if (is_valid_id(id) == false)
    {
        return DVE_DM_ERR_PARAM;
    }

    if ((cmd != DVE_DM_CMD_ENABLE) &&
        (cmd != DVE_DM_CMD_DISABLE) &&
        (cmd != DVE_DM_CMD_ZERO))
    {
        return DVE_DM_ERR_PARAM;
    }

    /*
     * 达妙特殊命令格式：
     * data[0..6] = 0xFF
     * data[7]    = 0xFC / 0xFD / 0xFE
     */
    data[7] = cmd;

    std_id = (uint16_t)(DVE_DM_CAN_ID_OFFSET_MIT + get_can_id(id));
    can_status = BSP_CAN_SendStd8(s_dm_hcan, std_id, data);

    return can_status_to_dm_status(can_status);
}

/*
*********************************************************************************************************
*   函 数 名: pack_float_le
*   功能说明: 将 float 按小端字节序打包到 4 字节数组
*   形    参: data              - 4 字节输出缓冲区
*             value             - 待打包 float 值
*   返 回 值: 无
*********************************************************************************************************
*/
static void pack_float_le(uint8_t data[4], float value)
{
    /*
     * STM32F4 是小端架构。
     * 这里使用 memcpy，避免直接指针强转带来的严格别名规则问题。
     */
    memcpy(data, &value, sizeof(float));
}

/*
*********************************************************************************************************
*   函 数 名: can_status_to_dm_status
*   功能说明: 将 BSP_CAN 返回值转换为 DVE_DM 返回值，避免上层直接依赖 BSP 状态码
*   形    参: status            - BSP_CAN 返回值
*   返 回 值: DVE_DmMotorStatus_t
*********************************************************************************************************
*/
static DVE_DmMotorStatus_t can_status_to_dm_status(BSP_CAN_Status_t status)
{
    if (status == BSP_CAN_OK)
    {
        return DVE_DM_OK;
    }

    return DVE_DM_ERR_CAN;
}
