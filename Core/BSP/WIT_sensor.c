//
// Created by ShiF on 2023/7/13.
//
#include "bsp_headfile.h"
#include "usart.h"
#include "string.h"

WIT_sensor_USART WT9011G4K_data;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 	stcMag;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SQ stcQ;

void WIT_sensor_USART_Init(WIT_sensor_USART *Data){
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    HAL_UART_Receive_IT(&huart3, (void *) &WT9011G4K_data.RxBuffer, 1);
//    HAL_UART_Receive_DMA(&huart3,WT9011G4K_data.RxBuffer,RXBUFFER_LEN);
    for(uint16_t i=0; i < RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;
    Data->frame_head = 0x55;
    Data->Rx_flag = 0;
    Data->Rx_len = 0;
}

void WIT_sensor_receive(void){
    if(WT9011G4K_data.Rx_len < RXBUFFER_LEN) return;   	//如果位数不对

    for(uint8_t i=0;i<9;i++)
    {
        if(WT9011G4K_data.RxBuffer[i*11]!= WT9011G4K_data.frame_head) return;	//如果帧头不对
        switch(WT9011G4K_data.RxBuffer[i*11+1])
        {
            case 0x51:
                memcpy(&stcAcc,&WT9011G4K_data.RxBuffer[2 + i*11],8);
                for(uint8_t j = 0; j < 3; j++)
                    WT9011G4K_data.acc.a[j] = (float)stcAcc.a[j]/32768*16;
                break;

            case 0x52:
                memcpy(&stcGyro,&WT9011G4K_data.RxBuffer[2 + i*11],8);
                for(uint8_t j = 0; j < 3; j++)
                    WT9011G4K_data.w.w[j] = (float)stcGyro.w[j]/32768*2000;
                break;

            case 0x53: //角度
                memcpy(&stcAngle,&WT9011G4K_data.RxBuffer[2 + i*11],8);
                for(uint8_t j = 0; j < 3; j++)
                    WT9011G4K_data.angle.angle[j] = (float)stcAngle.Angle[j]/32768*180;
                break;

            case 0x54:	//磁场解算
                memcpy(&stcMag,&WT9011G4K_data.RxBuffer[2 + i*11],8);
                for(uint8_t j = 0; j < 3; j++)
                    WT9011G4K_data.h.h[j] = (float)stcMag.h[j];
                break;

            case 0x55:	//D0-D3端口状态
                break;

            case 0x56:	//气压高度
                memcpy(&stcPress,&WT9011G4K_data.RxBuffer[2 + i*11],8);
                WT9011G4K_data.lPressure.lPressure = (float)stcPress.lPressure;
                WT9011G4K_data.lPressure.lAltitude = (float)stcPress.lAltitude/100;
                break;

            case 0x57:	//经纬度
                memcpy(&stcLonLat.lLat,&WT9011G4K_data.RxBuffer[2 + i*11],4);
                WT9011G4K_data.lLon.lLat = (float)stcLonLat.lLat/10000000+(double)(stcLonLat.lLat % 10000000)/1e5;
                WT9011G4K_data.lLon.lLat = (float)stcLonLat.lLon/10000000+(double)(stcLonLat.lLon % 10000000)/1e5;
                break;

            case 0x58:	//GPS
                break;

            case 0x59:	//四元数
                memcpy(&stcQ,&WT9011G4K_data.RxBuffer[2 + i*11],8);
                for(uint8_t j = 0; j < 4; j++)
                    WT9011G4K_data.q.q[j] = (float)stcQ.q[j]/32768;
                break;
        }
    }
}