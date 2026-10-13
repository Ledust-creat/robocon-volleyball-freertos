#ifndef HOST_CAN_H
#define HOST_CAN_H
#include "main.h"
typedef struct { unsigned bus; } CAN_HandleTypeDef;
typedef struct {
    uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC; } CAN_RxHeaderTypeDef;
typedef struct {
    uint32_t FilterActivation, FilterMode, FilterScale, FilterIdHigh, FilterIdLow;
    uint32_t FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment, FilterBank;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
#define CAN_ID_STD 0U
#define CAN_ID_EXT 1U
#define CAN_RTR_DATA 0U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 0U
#define CAN_RX_FIFO0 0U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define ENABLE 1U
#define HAL_OK 0
#define HAL_ERROR 1
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *filter);
int HAL_CAN_Start(CAN_HandleTypeDef *can);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t mask);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, const CAN_TxHeaderTypeDef *header,
                         const uint8_t data[8], uint32_t *mailbox);
#endif
