//
// Created by ShiF on 2023/5/12.
//
#include "openmv.h"
#include "stdio.h"
#include "usart.h"
#include "stm32f1xx_hal.h"


int8_t  mv_Cx=0, mv_Cy=0, mv_Cw=0, mv_Ch=0;
int16_t error_angle,distance_target;


int8_t uart3_rxbuff;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    int16_t tem;
//    if(huart->Instance== USART2)
//    {
//        tem = uart3_rxbuff;
//        Openmv_Receive_Data(tem);
//    }
//    HAL_UART_Receive_IT(&huart2,(void *)&uart3_rxbuff,1);
//}
void openmv_Init(void)
{
    HAL_UART_Receive_IT(&huart2,(void *)&uart3_rxbuff,1);
}

void   Openmv_Receive_Data(int16_t com_data)
{

    uint8_t i;
    static uint8_t RxCounter1=0;//计数
    static int16_t RxBuffer1[10]={0};
    static uint8_t RxState = 0;
    static uint8_t RxFlag1 = 0;

    if(RxState==0&&com_data==0x2C)  //0x2c帧头
    {

        RxState=1;
        RxBuffer1[RxCounter1++]=com_data;
        //HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    }

    else if(RxState==1&&com_data==0x12)  //0x12帧头
    {
        //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        RxState=2;
        RxBuffer1[RxCounter1++]=com_data;
    }
    else if(RxState==2)
    {

        RxBuffer1[RxCounter1++]=com_data;
        if(RxCounter1>=10||com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
        {
            RxState=3;
            RxFlag1=1;

            mv_Cx=RxBuffer1[RxCounter1-5];
            mv_Cy=RxBuffer1[RxCounter1-4];
            mv_Cw=RxBuffer1[RxCounter1-3];
            mv_Ch=RxBuffer1[RxCounter1-2];

            error_angle=mv_Cy;
            distance_target=mv_Cy;
//               if(RxState==1)
//          {
//            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//          }
//          else if(RxState!=1&&RxState!=0)
//          {
//            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
//          }
        }
    }

    else if(RxState==3)		//检测是否接受到结束标志
    {
        if(RxBuffer1[RxCounter1-1] == 0x5B)
        {

            RxFlag1 = 0;
            RxCounter1 = 0;
            RxState = 0;

        }
        else   //接收错误
        {
            RxState = 0;
            RxCounter1=0;
            for(i=0;i<10;i++)
            {
                RxBuffer1[i]=0x00;      //将存放数据数组清零
            }
        }
    }

    else   //接收异常
    {
        RxState = 0;
        RxCounter1=0;
        for(i=0;i<10;i++)
        {
            RxBuffer1[i]=0x00;      //将存放数据数组清零
        }
    }
}

/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart2, &ch, 1, 0xffff);
    return ch;
}
