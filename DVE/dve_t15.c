#include "dve_t15.h"
#include <string.h>

/*
*********************************************************************************************************
*                                      内部函数
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*   函 数 名: DVE_T15_DetectSwitch
*   功能说明: 根据通道值判断 T15 三档拨杆位置
*   形    参: value - 已减去中值偏移后的通道值
*   返 回 值: DVE_T15_SwitchPos_t
*********************************************************************************************************
*/
DVE_T15_SwitchPos_t DVE_T15_DetectSwitch(int16_t value)
{
    if (value <= (DVE_T15_SWITCH_VALUE_MIN + DVE_T15_SWITCH_DEADZONE))
    {
        return DVE_T15_SWITCH_UP;
    }

    if (value >= (DVE_T15_SWITCH_VALUE_MAX - DVE_T15_SWITCH_DEADZONE))
    {
        return DVE_T15_SWITCH_DOWN;
    }

    return DVE_T15_SWITCH_MID;
}

/*
*********************************************************************************************************
*                                      函数定义
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*   函 数 名: DVE_T15_ParseFrame
*   功能说明: 解析 T15/SBUS 原始帧，输出通道值和拨杆状态
*   形    参: frame - 原始 25 字节 SBUS 帧
*             len   - 原始帧长度
*             out   - 解析结果
*   返 回 值: 1 - 解析成功
*             0 - 解析失败
*********************************************************************************************************
*/
uint8_t DVE_T15_ParseFrame(const uint8_t *frame, uint16_t len, DVE_T15_Data_t *out)
{
    if ((frame == NULL) || (out == NULL))
    {
        return 0U;
    }

    memset(out, 0, sizeof(DVE_T15_Data_t));

    if (len != DVE_T15_FRAME_LEN)
    {
        return 0U;
    }

    if ((frame[0] != DVE_T15_FRAME_HEAD) ||
        (frame[DVE_T15_FRAME_TAIL_INDEX] != DVE_T15_FRAME_TAIL))
    {
        return 0U;
    }

    out->ch[0] = (int16_t)(((frame[1] | (frame[2] << 8)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[1] = (int16_t)((((frame[2] >> 3) | (frame[3] << 5)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[2] = (int16_t)((((frame[3] >> 6) | (frame[4] << 2) | (frame[5] << 10)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[3] = (int16_t)((((frame[5] >> 1) | (frame[6] << 7)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[4] = (int16_t)((((frame[6] >> 4) | (frame[7] << 4)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[5] = (int16_t)((((frame[7] >> 7) | (frame[8] << 1) | (frame[9] << 9)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[6] = (int16_t)((((frame[9] >> 2) | (frame[10] << 6)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[7] = (int16_t)((((frame[10] >> 5) | (frame[11] << 3)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[8] = (int16_t)(((frame[12] | (frame[13] << 8)) & 0x07FF) - DVE_T15_CH_OFFSET);
    out->ch[9] = (int16_t)((((frame[13] >> 3) | (frame[14] << 5)) & 0x07FF) - DVE_T15_CH_OFFSET);

    /*
     * 当前工程约定:
     * ch[4] -> SWA
     * ch[5] -> SWB
     * ch[6] -> SWC
     * ch[7] -> SWD
     * ch[8] -> SE
     * ch[9] -> SF
     */
    out->swa = DVE_T15_DetectSwitch(out->ch[4]);
    out->swb = DVE_T15_DetectSwitch(out->ch[5]);
    out->swc = DVE_T15_DetectSwitch(out->ch[6]);
    out->swd = DVE_T15_DetectSwitch(out->ch[7]);
    out->se  = DVE_T15_DetectSwitch(out->ch[8]);
    out->sf  = DVE_T15_DetectSwitch(out->ch[9]);



    out->frame_valid = 1U;

    return 1U;
}