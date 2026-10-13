#ifndef __DVE_T15_H
#define __DVE_T15_H

#include <stdint.h>

/*
*********************************************************************************************************
*                                      宏定义
*********************************************************************************************************
*/

/* T15/SBUS 帧格式 */
#define DVE_T15_FRAME_LEN                 25U
#define DVE_T15_FRAME_HEAD                0x0FU
#define DVE_T15_FRAME_TAIL                0x00U
#define DVE_T15_FRAME_TAIL_INDEX          (DVE_T15_FRAME_LEN - 1U)

/* T15 通道配置 */
#define DVE_T15_CH_NUM                    10U
#define DVE_T15_CH_RAW_MASK               0x07FFU
#define DVE_T15_CH_OFFSET                 992

/* T15 拨杆配置 */
#define DVE_T15_SWITCH_VALUE_MIN          (-800)
#define DVE_T15_SWITCH_VALUE_MAX          (800)
#define DVE_T15_SWITCH_DEADZONE           (100)

/*
*********************************************************************************************************
*                                      数据类型
*********************************************************************************************************
*/

/* T15 三档拨杆位置 */
typedef enum
{
    DVE_T15_SWITCH_UNKNOWN = 0,
    DVE_T15_SWITCH_UP,
    DVE_T15_SWITCH_MID,
    DVE_T15_SWITCH_DOWN
} DVE_T15_SwitchPos_t;

/* T15 拨杆编号 */
typedef enum
{
    DVE_T15_SWITCH_SWA = 0,
    DVE_T15_SWITCH_SWB,
    DVE_T15_SWITCH_SWC,
    DVE_T15_SWITCH_SWD,
    DVE_T15_SWITCH_SE,
    DVE_T15_SWITCH_SF
} DVE_T15_SwitchId_t;

/* T15 解析结果 */
typedef struct
{
    int16_t ch[DVE_T15_CH_NUM];                         /* 通道值，已减去中值偏移 */

    DVE_T15_SwitchPos_t swa;                            /* SWA 拨杆状态 */
    DVE_T15_SwitchPos_t swb;                            /* SWB 拨杆状态 */
    DVE_T15_SwitchPos_t swc;                            /* SWC 拨杆状态 */
    DVE_T15_SwitchPos_t swd;                            /* SWD 拨杆状态 */
    DVE_T15_SwitchPos_t se;                             /* SE  拨杆状态 */
    DVE_T15_SwitchPos_t sf;                             /* SF  拨杆状态 */

    uint8_t failsafe;             /* SBUS第23字节bit3；置位时帧不可用于控制 */
    uint8_t frame_lost;           /* SBUS第23字节bit2；本工程采用立即停机策略 */
    uint8_t frame_valid;                                /* 当前帧有效标志 */
} DVE_T15_Data_t;

/*
*********************************************************************************************************
*                                      函数声明
*********************************************************************************************************
*/

uint8_t DVE_T15_ParseFrame(const uint8_t *frame, uint16_t len, DVE_T15_Data_t *out);
DVE_T15_SwitchPos_t DVE_T15_DetectSwitch(int16_t value);

#endif
