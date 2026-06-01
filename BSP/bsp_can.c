#include "bsp_can.h"

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
*             如果后续总线负载明显变大，再改成按 0x201~0x205、DM ID 精确过滤。
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
*             - CAN2：DJI 3508/C620 电机，接 0x201~0x205。
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
*             BSP_CAN_ERR_BUSY  - 三个发送邮箱均满
*             BSP_CAN_ERR_HAL   - HAL_CAN_AddTxMessage 调用失败
*   说    明:
*             本函数是通用 BSP 发送接口，不假设所有协议都是 8 字节。
*             例如：
*             - DJI/DM 电机协议固定 8 字节，应由 DVE 层调用 BSP_CAN_SendStd8()；
*             - 其他自定义设备如果只发 2 字节，应调用 BSP_CAN_SendStd(..., 2)。
*********************************************************************************************************
*/
BSP_CAN_Status_t BSP_CAN_SendStd(CAN_HandleTypeDef *hcan,
                                 uint16_t std_id,
                                 const uint8_t *data,
                                 uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;

    if ((hcan == NULL) || (std_id > 0x7FFU) || (dlc > BSP_CAN_MAX_DLC))
    {
        return BSP_CAN_ERR_PARAM;
    }

    /* dlc = 0 是合法的远程/空数据帧场景；但只要 dlc > 0，data 必须有效。 */
    if ((dlc > 0U) && (data == NULL))
    {
        return BSP_CAN_ERR_PARAM;
    }

    /* 邮箱满时不阻塞等待，直接返回 BUSY，让上层自己决定是否重发。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return BSP_CAN_ERR_BUSY;
    }

    tx_header.StdId = std_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &tx_mailbox) != HAL_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    return BSP_CAN_OK;
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
*             BSP_CAN_ERR_BUSY  - 三个发送邮箱均满
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
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0;

    if ((hcan == NULL) || (ext_id > 0x1FFFFFFFU) || (dlc > BSP_CAN_MAX_DLC))
    {
        return BSP_CAN_ERR_PARAM;
    }

    if ((dlc > 0U) && (data == NULL))
    {
        return BSP_CAN_ERR_PARAM;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return BSP_CAN_ERR_BUSY;
    }

    tx_header.StdId = 0U;
    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &tx_mailbox) != HAL_OK)
    {
        return BSP_CAN_ERR_HAL;
    }

    return BSP_CAN_OK;
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
