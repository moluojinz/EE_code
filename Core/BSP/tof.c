//
// Created by ShiF on 2023/5/15.
//
#include "stm32f1xx_hal.h"
#include "tof_usart.h"
#include "tof.h"
#include "string.h"
#include "usart.h"

#define TX_BUF_SIZE 128
#define tof_rxbuf_size 128
uint8_t tof_send_buf[TX_BUF_SIZE];
uint8_t tof_Cx,tof_Cy,tof_Cz,tof_Cw;
uint32_t tof_data;
char tofRvBuff[tof_rxbuf_size] = { 0 };  //存放串口2（调试用）接收的第一手数据
char tofBuff[tof_rxbuf_size] = { 0 };    //进行一定变换

int16_t tof_start_flag = 0;
uint8_t tof_rxbuff;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    int16_t tem;
    if(huart->Instance== USART2)
    {
        tem = tof_rxbuff;
        tof_Receive_Data(tem);
    }
    HAL_UART_Receive_IT(&huart2,(void *)&tof_rxbuff,1);
}
void TOF_UartInit(void)
{
    HAL_UART_Receive_IT(&huart2, (void*)&tofRvBuff, 1);
}

void   tof_Receive_Data(int16_t com_data) {

    uint8_t i;
    static uint8_t RxCounter1 = 0;
    static uint16_t RxBuffer1[7] = {0};
    static uint8_t RxState = 0;

    if (RxState == 0 && com_data == 0x64)  //0x2c帧头
    {

        RxState = 1;
        RxBuffer1[RxCounter1++] = com_data;
    } else if (RxState == 1 && com_data == 0x3A)  //0x12帧头
    {
        RxState = 2;
        RxBuffer1[RxCounter1++] = com_data;
    } else if (RxState == 2) {
        RxBuffer1[RxCounter1++] = com_data;

        if (RxCounter1 == 7 && com_data == 0x20)       //RxBuffer1接受满了,接收数据结束
        {

            tof_Cx = RxBuffer1[RxCounter1 - 5];
            tof_Cy = RxBuffer1[RxCounter1 - 4];
            tof_Cz = RxBuffer1[RxCounter1 - 3];
            tof_Cw = RxBuffer1[RxCounter1 - 2];
            if (tof_Cx < 48) {
                tof_Cx = 0;
            } else {
                tof_Cx -= 48;
            }
            if (tof_Cy < 48) {
                tof_Cy = 0;
            } else {
                tof_Cy -= 48;
            }
            if (tof_Cz < 48) {
                tof_Cz = 0;
            } else {
                tof_Cz -= 48;
            }
            if (tof_Cw < 48) {
                tof_Cw = 0;
            } else {
                tof_Cw -= 48;
            }
            RxCounter1 = 0;
            RxState = 0;
            tof_data = tof_Cx*1000+tof_Cy*100+tof_Cz*10+tof_Cw;
        } else if (RxCounter1 > 7) {
            RxState = 0;
            RxCounter1 = 0;
            for (i = 0; i < 7; i++) {
                RxBuffer1[i] = 0x00;      //将存放数据数组清零
            }

        }
    } else   //接收异常
    {
        RxState = 0;
        RxCounter1 = 0;
        for (i = 0; i < 7; i++) {
            RxBuffer1[i] = 0x00;      //将存放数据数组清零
        }
    }
}





