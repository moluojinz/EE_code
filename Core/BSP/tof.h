//
// Created by ShiF on 2023/5/15.
//

#ifndef EE_CODE_TOF_H
#define EE_CODE_TOF_H



#define ATK_MS53L1M_EOK         0   /* 没有错误 */
#define ATK_MS53L1M_ERROR       1   /* 错误 */
#define ATK_MS53L1M_ETIMEOUT    2   /* 超时错误 */
#define ATK_MS53L1M_EFRAME      3   /* 帧错误 */
#define ATK_MS53L1M_ECRC        4   /* CRC校验错误 */
#define ATK_MS53L1M_EOPT        5   /* 操作错误 */

void    tof_Receive_Data(int16_t com_data);
void TOF_UartInit(void);

extern uint32_t tof_data;
#endif //EE_CODE_TOF_H
