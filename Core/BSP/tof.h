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


#define TX_BUF_SIZE 128
#define tof_rxbuf_size 128
extern uint8_t tof_send_buf[TX_BUF_SIZE];
extern uint8_t tof_Cx,tof_Cy,tof_Cz,tof_Cw;
extern char tofRvBuff[tof_rxbuf_size];  //存放串口2（调试用）接收的第一手数据
extern char tofBuff[tof_rxbuf_size];    //进行一定变换

extern int16_t tof_start_flag ;
extern uint8_t tof_rxbuff;

extern uint32_t tof_data;
#endif //EE_CODE_TOF_H
