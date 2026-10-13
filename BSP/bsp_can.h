#ifndef BSP_CAN_H_
#define BSP_CAN_H_

#include "main.h"
#include "can.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   文件说明
*********************************************************************************************************
* 文件名称：bsp_can.h
* 功能定位：CAN 外设通用 BSP 层接口。
* - CAN1：预留给 DM 达妙电机；
* - CAN2：连接 DJI 协议电机，即 M3508 + C620 电调，当前使用 ID 0x201~0x208。
*/

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/

/* 经典 CAN 单帧最大数据长度。STM32F4 bxCAN 不是 CAN-FD，因此 DLC 最大只能是 8。 */
#define BSP_CAN_MAX_DLC                   (8U)

/*
 * STM32F4 双 CAN 共用一组滤波器组。
 * 常见划分方式：
 * - CAN1 使用 FilterBank 0~13；
 * - CAN2 使用 FilterBank 14~27；
 * - SlaveStartFilterBank = 14 表示从 14 开始分给 CAN2。
 */
#define BSP_CAN_FILTER_BANK_CAN1          (0U)
#define BSP_CAN_FILTER_BANK_CAN2          (14U)
#define BSP_CAN_FILTER_BANK_SPLIT         (14U)

/* 兼容旧命名。如果你已经在别的文件里用了旧宏名，暂时不会编译报错。 */
#define BSP_CAN1_FILTER_BANK              BSP_CAN_FILTER_BANK_CAN1
#define BSP_CAN2_FILTER_BANK              BSP_CAN_FILTER_BANK_CAN2
#define BSP_CAN_SLAVE_START_FILTER_BANK   BSP_CAN_FILTER_BANK_SPLIT

/*
*********************************************************************************************************
*   数据类型
*********************************************************************************************************
*/

/*
 * BSP_CAN_Status_t
 * 统一返回值，避免上层直接依赖 HAL_StatusTypeDef。
 * 后面如果要统计错误，也可以直接根据这个枚举做判断。
 */
typedef enum
{
    BSP_CAN_OK = 0,          /* 操作成功 */
    BSP_CAN_ERR_PARAM,       /* 参数错误，例如 hcan/data 为空，ID 越界，DLC 超过 8 */
    BSP_CAN_ERR_HAL,         /* HAL 库函数返回错误 */
    BSP_CAN_ERR_NOT_INIT,    /* 发送mutex尚未就绪或创建失败 */
    BSP_CAN_ERR_CONTEXT,     /* 当前上下文不能等待mutex */
    BSP_CAN_ERR_BUSY         /* 获取锁超时或发送邮箱已满 */
} BSP_CAN_Status_t;

typedef struct
{
    uint32_t submitted;        /* HAL已提交邮箱的累计次数，不是总线完成次数 */
    uint32_t lock_timeout;     /* 等待发送mutex超时次数 */
    uint32_t mailbox_full;     /* 持锁检查时无空闲邮箱的次数 */
    uint32_t hal_error;        /* HAL提交失败次数 */
    uint32_t context_error;    /* ISR/屏蔽中断/调度器状态拒绝次数 */
} BSP_CAN_TxStats_t;

/* 启动任务创建发送mutex；发送接口仅供调度器运行后的任务调用。 */
BSP_CAN_Status_t BSP_CAN_InitTxSync(void);
/* 短临界区复制统计快照；不清除计数，不依赖发送mutex。 */
BSP_CAN_Status_t BSP_CAN_GetTxStats(CAN_HandleTypeDef *hcan, BSP_CAN_TxStats_t *stats);

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/

/* 配置全接收滤波器。当前阶段先全接收，后续总线负载高时再改成精确 ID 过滤。 */
BSP_CAN_Status_t BSP_CAN_ConfigFilterAllPass(CAN_HandleTypeDef *hcan,
                                             uint32_t filter_bank,
                                             uint32_t slave_start_filter_bank);

/* 启动 CAN 外设，并打开 FIFO0 接收中断。 */
BSP_CAN_Status_t BSP_CAN_Start(CAN_HandleTypeDef *hcan);

/* 当前工程默认初始化：CAN1=达妙预留，CAN2=DJI 3508/C620。 */
BSP_CAN_Status_t BSP_CAN_InitDefault(void);

/*
 * 发送 11bit 标准帧。
 * dlc 表示本帧实际发送字节数，范围 0~8。
 * data 在 dlc > 0 时不能为空；dlc = 0 时允许 data = NULL。
 */
BSP_CAN_Status_t BSP_CAN_SendStd(CAN_HandleTypeDef *hcan,
                                 uint16_t std_id,
                                 const uint8_t *data,
                                 uint8_t dlc);

/*
 * 发送 29bit 扩展帧。
 * dlc 表示本帧实际发送字节数，范围 0~8。
 */
BSP_CAN_Status_t BSP_CAN_SendExt(CAN_HandleTypeDef *hcan,
                                 uint32_t ext_id,
                                 const uint8_t *data,
                                 uint8_t dlc);

/*
 * 8 字节标准帧快捷接口。
 * 这不是 BSP 层默认 8 字节，而是给 DJI/DM 这类“协议固定 8 字节”的 DVE 层使用。
 */
BSP_CAN_Status_t BSP_CAN_SendStd8(CAN_HandleTypeDef *hcan,
                                  uint16_t std_id,
                                  const uint8_t data[8]);

/* 8 字节扩展帧快捷接口，当前工程暂时不用，保留给后续扩展设备。 */
BSP_CAN_Status_t BSP_CAN_SendExt8(CAN_HandleTypeDef *hcan,
                                  uint32_t ext_id,
                                  const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H_ */
