#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/*
*********************************************************************************************************
*   模块变量：下标0对应CAN1，下标1对应CAN2；两条物理总线互不占用对方的发送锁。
*   mutex只保护发送事务；设备层反馈快照和本层统计副本分别使用PRIMASK短临界区。
*********************************************************************************************************
*/
static StaticSemaphore_t s_tx_mutex_storage[2];
static SemaphoreHandle_t s_tx_mutex[2];
static BSP_CAN_TxStats_t s_tx_stats[2];
static bool s_tx_sync_ready = false;

static int32_t BSP_CAN_BusIndex(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) return 0;
    if (hcan == &hcan2) return 1;
    return -1;
}

// 统计可能在获取mutex前更新；不依赖调度器，并保留调用者原有的中断屏蔽状态。
static void BSP_CAN_Count(uint32_t *counter)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    (*counter)++;
    if (primask == 0U) __enable_irq();
}

/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_InitTxSync
*   功能说明: 为两条物理CAN总线创建支持优先级继承的静态发送mutex
*   形    参: 无
*   返 回 值: OK表示就绪；CONTEXT表示ISR调用；NOT_INIT表示创建失败
*   说    明: 由启动任务串行调用，必须先于发送任务；创建成功后重复调用不重新创建。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_InitTxSync(void)
{
    if (__get_IPSR() != 0U) return BSP_CAN_ERR_CONTEXT;
    if (s_tx_sync_ready) return BSP_CAN_OK;
    for (uint32_t i = 0U; i < 2U; i++)
    {
        s_tx_mutex[i] = xSemaphoreCreateMutexStatic(&s_tx_mutex_storage[i]);
        if (s_tx_mutex[i] == NULL) return BSP_CAN_ERR_NOT_INIT;
    }
    s_tx_sync_ready = true;
    return BSP_CAN_OK;
}

/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_GetTxStats
*   功能说明: 返回指定CAN总线的一致性发送统计副本
*   形    参: hcan - CAN1/CAN2句柄；stats - 调用者提供的副本存储区
*   返 回 值: OK表示复制成功；PARAM表示句柄或输出指针无效
*   说    明: submitted统计HAL提交次数，不代表报文已在总线上成功发送。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_GetTxStats(CAN_HandleTypeDef *hcan, BSP_CAN_TxStats_t *stats)
{
    int32_t bus = BSP_CAN_BusIndex(hcan);
    uint32_t primask;
    if ((bus < 0) || (stats == NULL)) return BSP_CAN_ERR_PARAM;
    primask = __get_PRIMASK();
    __disable_irq();
    *stats = s_tx_stats[bus];
    if (primask == 0U) __enable_irq();
    return BSP_CAN_OK;
}

/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_Submit
*   功能说明: 串行完成邮箱选择、寄存器写入与发送请求提交
*   形    参: hcan - 总线句柄；header - 已校验的帧头；data - DLC个有效字节
*   返 回 值: OK表示已提交；BUSY表示锁超时/邮箱满；其余状态表示初始化、上下文或HAL错误
*   说    明: 仅供任务调用；最多等1 tick，提交结束即释放锁，不等待总线发送完成。
*********************************************************************************************************
*/
static BSP_CAN_Status_t BSP_CAN_Submit(CAN_HandleTypeDef *hcan,
                                      const CAN_TxHeaderTypeDef *header,
                                      const uint8_t *data)
{
    int32_t bus = BSP_CAN_BusIndex(hcan);
    uint8_t payload[BSP_CAN_MAX_DLC] = {0};
    uint32_t mailbox = 0U;
    BSP_CAN_Status_t status;
    if ((bus < 0) || ((header->DLC > 0U) && (data == NULL))) return BSP_CAN_ERR_PARAM;
    // ISR、关中断、屏蔽内核中断或调度器未运行时不能等待mutex，统一拒绝提交。
    if ((__get_IPSR() != 0U) || (__get_PRIMASK() != 0U) ||
        (__get_BASEPRI() != 0U) || (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING))
    {
        BSP_CAN_Count(&s_tx_stats[bus].context_error);
        return BSP_CAN_ERR_CONTEXT;
    }
    if (!s_tx_sync_ready) return BSP_CAN_ERR_NOT_INIT;
    // F4 HAL即使DLC不足8也读取全部8字节；补零缓冲兼容短帧及DLC=0/data=NULL。
    if (header->DLC > 0U) memcpy(payload, data, header->DLC);
    // 只等待1个RTOS tick（当前1ms）；获取失败不调用HAL，也不忙等重试。
    if (xSemaphoreTake(s_tx_mutex[bus], (TickType_t)1U) != pdTRUE)
    {
        BSP_CAN_Count(&s_tx_stats[bus].lock_timeout);
        return BSP_CAN_ERR_BUSY;
    }
    // 邮箱检查和HAL选择/写入在同一把锁内，避免两个任务选中同一空邮箱。
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        BSP_CAN_Count(&s_tx_stats[bus].mailbox_full);
        status = BSP_CAN_ERR_BUSY;
    }
    else if (HAL_CAN_AddTxMessage(hcan, header, payload, &mailbox) != HAL_OK)
    {
        BSP_CAN_Count(&s_tx_stats[bus].hal_error);
        status = BSP_CAN_ERR_HAL;
    }
    else
    {
        BSP_CAN_Count(&s_tx_stats[bus].submitted);
        status = BSP_CAN_OK;
    }
    // 成功、邮箱满和HAL错误都经过此出口，硬件后续发送不再占用mutex。
    (void)xSemaphoreGive(s_tx_mutex[bus]);
    return status;
}

/*
*********************************************************************************************************
*   文件说明
*********************************************************************************************************
* 文件名称：bsp_can.c
* 功能定位：CAN 外设通用 BSP 层实现。
*
* 本文件只做和 CAN 外设直接相关的事情：
* 1. 配置滤波器；
* 2. 启动 CAN；
* 3. 打开 CAN 接收中断；
* 4. 封装标准帧/扩展帧发送。
*
* 不允许在本文件中加入：
* - 3508 / C620 / DM 电机协议解析；
* - 底盘、击球、抬球等业务逻辑；
* - PID 计算；
* - FreeRTOS 队列收发。
*
* 重要修正：
* - BSP_CAN_SendStd() / BSP_CAN_SendExt() 必须传入 dlc；
* - BSP 层不默认发送 8 字节；
* - DJI/DM 固定 8 字节协议由 DVE 层调用 BSP_CAN_SendStd8()。
*/

/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_ConfigFilterAllPass
*   功能说明: 配置 CAN 全接收滤波器
*   形    参: hcan                    - CAN 句柄，例如 &hcan1 / &hcan2
*             filter_bank             - 当前 CAN 使用的滤波器编号
*             slave_start_filter_bank - CAN2 起始滤波器编号，STM32F4 双 CAN 常用 14
*   返 回 值: BSP_CAN_OK              - 配置成功
*             BSP_CAN_ERR_PARAM       - hcan 为空
*             BSP_CAN_ERR_HAL         - HAL_CAN_ConfigFilter 调用失败
*   说    明:
*             当前阶段使用“全接收”策略，所有标准帧/扩展帧都会进入 FIFO0。
*             这样方便调试，不会因为滤波 ID 配错导致完全收不到电机反馈。
*             如果后续总线负载明显变大，再改成按 0x201~0x208、DM ID 精确过滤。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_ConfigFilterAllPass(CAN_HandleTypeDef *hcan,
                                             uint32_t filter_bank,
                                             uint32_t slave_start_filter_bank)
{
    CAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return BSP_CAN_ERR_PARAM;
    }

    /*
     * IDMASK + 32bit + 全 0 掩码：
     * FilterId = 0，FilterMask = 0，表示不关心任何 ID 位，即全部通过。
     */
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;

    /*
     * FilterBank 指定当前 CAN 使用哪一个滤波器组。
     * 双 CAN 时 CAN2 必须使用 SlaveStartFilterBank 后面的组。
     */
    filter.FilterBank = filter_bank;
    filter.SlaveStartFilterBank = slave_start_filter_bank;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    return BSP_CAN_OK;
}


/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_Start
*   功能说明: 启动 CAN 外设，并打开 FIFO0 消息挂起中断
*   形    参: hcan              - CAN 句柄
*   返 回 值: BSP_CAN_OK        - 启动成功
*             BSP_CAN_ERR_PARAM - hcan 为空
*             BSP_CAN_ERR_HAL   - HAL 启动或中断使能失败
*   说    明:
*             本函数只启动 CAN 和接收中断，不做滤波器配置。
*             因此调用顺序建议为：
*             BSP_CAN_ConfigFilterAllPass() -> BSP_CAN_Start()。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_Start(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return BSP_CAN_ERR_PARAM;
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    return BSP_CAN_OK;
}


/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_InitDefault
*   功能说明: 按当前排球机器人默认硬件分配初始化 CAN1/CAN2
*   形    参: 无
*   返 回 值: BSP_CAN_OK        - 初始化成功
*             BSP_CAN_ERR_HAL   - 任意一路 CAN 初始化失败
*   说    明:
*             当前约定：
*             - CAN1：DM 达妙电机预留；
*             - CAN2：DJI 3508/C620 电机，接 0x201~0x208。
*
*             这个函数建议放在 All_Init() 中调用。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_InitDefault(void)
{
    if (BSP_CAN_ConfigFilterAllPass(&hcan1,
                                    BSP_CAN_FILTER_BANK_CAN1,
                                    BSP_CAN_FILTER_BANK_SPLIT) != BSP_CAN_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    if (BSP_CAN_ConfigFilterAllPass(&hcan2,
                                    BSP_CAN_FILTER_BANK_CAN2,
                                    BSP_CAN_FILTER_BANK_SPLIT) != BSP_CAN_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    if (BSP_CAN_Start(&hcan1) != BSP_CAN_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    if (BSP_CAN_Start(&hcan2) != BSP_CAN_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    return BSP_CAN_OK;
}


/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_SendStd
*   功能说明: 发送 11bit 标准帧 CAN 报文
*   形    参: hcan              - CAN 句柄
*             std_id            - 标准帧 ID，范围 0x000~0x7FF
*             data              - 数据指针；dlc > 0 时不能为空；dlc = 0 时可为 NULL
*             dlc               - 实际发送字节数，范围 0~8
*   返 回 值: BSP_CAN_OK        - 发送成功写入邮箱
*             BSP_CAN_ERR_PARAM - 参数错误
*             BSP_CAN_ERR_BUSY  - 等待发送mutex超时或三个发送邮箱均满
*             BSP_CAN_ERR_NOT_INIT - 发送mutex未就绪
*             BSP_CAN_ERR_CONTEXT  - ISR、屏蔽中断或调度器未运行时调用
*             BSP_CAN_ERR_HAL   - HAL_CAN_AddTxMessage 调用失败
*   说    明:
*             本函数是通用 BSP 发送接口，不假设所有协议都是 8 字节。
*             发送仅供任务调用；共享mutex保护邮箱提交，不等待总线发送完成。例如：
*             - DJI/DM 电机协议固定 8 字节，应由 DVE 层调用 BSP_CAN_SendStd8()；
*             - 其他自定义设备如果只发 2 字节，应调用 BSP_CAN_SendStd(..., 2)。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_SendStd(CAN_HandleTypeDef *hcan,
                                 uint16_t std_id,
                                 const uint8_t *data,
                                 uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    if ((hcan == NULL) || (std_id > 0x7FFU) || (dlc > BSP_CAN_MAX_DLC))
        return BSP_CAN_ERR_PARAM;
    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    return BSP_CAN_Submit(hcan, &header, data);
}


/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_SendExt
*   功能说明: 发送 29bit 扩展帧 CAN 报文
*   形    参: hcan              - CAN 句柄
*             ext_id            - 扩展帧 ID，范围 0x00000000~0x1FFFFFFF
*             data              - 数据指针；dlc > 0 时不能为空；dlc = 0 时可为 NULL
*             dlc               - 实际发送字节数，范围 0~8
*   返 回 值: BSP_CAN_OK        - 发送成功写入邮箱
*             BSP_CAN_ERR_PARAM - 参数错误
*             BSP_CAN_ERR_BUSY  - 等待发送mutex超时或三个发送邮箱均满
*             BSP_CAN_ERR_NOT_INIT - 发送mutex未就绪
*             BSP_CAN_ERR_CONTEXT  - ISR、屏蔽中断或调度器未运行时调用
*             BSP_CAN_ERR_HAL   - HAL_CAN_AddTxMessage 调用失败
*   说    明:
*             当前主发球机器人暂时主要使用标准帧。
*             扩展帧接口先保留，方便后续接其他 CAN 设备。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_SendExt(CAN_HandleTypeDef *hcan,
                                 uint32_t ext_id,
                                 const uint8_t *data,
                                 uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    if ((hcan == NULL) || (ext_id > 0x1FFFFFFFU) || (dlc > BSP_CAN_MAX_DLC))
        return BSP_CAN_ERR_PARAM;
    header.ExtId = ext_id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    return BSP_CAN_Submit(hcan, &header, data);
}


/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_SendStd8
*   功能说明: 发送固定 8 字节标准帧
*   形    参: hcan              - CAN 句柄
*             std_id            - 标准帧 ID
*             data              - 8 字节数据
*   返 回 值: BSP_CAN_Status_t
*   说    明:
*             这是快捷封装，不代表 BSP 层默认所有 CAN 帧都是 8 字节。
*             DJI 3508/C620 控制帧、DM MIT 控制帧都是固定 8 字节，所以 DVE 层可以用这个接口。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_SendStd8(CAN_HandleTypeDef *hcan,
                                  uint16_t std_id,
                                  const uint8_t data[8])
{
    return BSP_CAN_SendStd(hcan, std_id, data, 8U);
}


/*
*********************************************************************************************************
*   函 数 名: BSP_CAN_SendExt8
*   功能说明: 发送固定 8 字节扩展帧
*   形    参: hcan              - CAN 句柄
*             ext_id            - 扩展帧 ID
*             data              - 8 字节数据
*   返 回 值: BSP_CAN_Status_t
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_SendExt8(CAN_HandleTypeDef *hcan,
                                  uint32_t ext_id,
                                  const uint8_t data[8])
{
    return BSP_CAN_SendExt(hcan, ext_id, data, 8U);
}
