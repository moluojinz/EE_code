//
// Created by ShiF on 2023/5/9.
//
#include "mp3.h"
#include "usart.h"
#include "stm32f1xx_hal.h"


void mp3_start(void)
{
    uint8_t mp3[]={0x31,0x0A};
    HAL_UART_Transmit(&huart2,(uint8_t *)mp3,sizeof(mp3),0xFFFF);

}

void mp3_stop(void)
{
    uint8_t mp3[]={0x7E,0xFF,0x06,0x0E,0x00,0x00,0x00,0xFE,0xED,0xEF};
    HAL_UART_Transmit(&huart2,(uint8_t *)mp3,sizeof(mp3),0xFFFF);
}
