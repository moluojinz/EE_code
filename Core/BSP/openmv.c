//
// Created by ShiF on 2023/5/12.
//
#include "openmv.h"
#include "stdio.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "bsp_headfile.h"

int16_t mv_Cx = 0, mv_Cy = 0, mv_Cw = 0, mv_Ch = 0;
int16_t error_angle, distance_target;
int16_t stop_flag;

int8_t uart2_rxbuff;

/*****MV2---usart3******/
int16_t mv_Cx_2 = 0, mv_Cy_2 = 0, mv_Cw_2 = 0, mv_Ch_2 = 0;
int16_t error_angle_mv2, distance_target_mv2,config_mv2;
int16_t stop_flag_mv2;

int8_t uart3_rxbuff;
/*****MV2---usart3******/

extern DMA_HandleTypeDef hdma_usart3_rx;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    int16_t tem;
    int16_t u3_tem;
    if(huart->Instance== USART2)
    {
        tem = uart2_rxbuff;
        Openmv_Receive_Data(tem);
        DEBUGC_UartIrqHandler(&huart2);
        DEBUGC_UartIdleCallback(&huart2);
    }
//    HAL_UART_Receive_IT(&huart2,(void *)&uart2_rxbuff,1);
    if(huart->Instance== USART3)
    {
//        u3_tem = tof_rxbuff;
//        tof_Receive_Data(u3_tem);
//        uint32_t temp_flag = 0;
//        uint32_t temp;
//        temp_flag = __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE);
//        if((temp_flag!=RESET))
//        {
//            __HAL_UART_CLEAR_IDLEFLAG(&huart3);
//            temp = huart3.Instance->SR;
//            temp = huart3.Instance->DR;
//            HAL_UART_DMAStop(&huart3);
//            temp = hdma_usart3_rx.Instance->CNDTR;            //F1的板子
//            WT9011G4K_data.Rx_len = RXBUFFER_LEN-temp;
//            WIT_sensor_receive();					//按照自己需求改写这个函数
//            WT9011G4K_data.Rx_flag = 1;
//        }
        tem = uart3_rxbuff;
        Openmv2_Receive_Data(tem);
    }
    HAL_UART_Receive_IT(&huart3,(void *)&uart3_rxbuff,1);
}
void openmv_Init(void) {
    HAL_UART_Receive_IT(&huart2, (void *) &uart2_rxbuff, 1);
}

void Openmv_Receive_Data(int16_t com_data) {

    uint8_t i;
    static uint8_t RxCounter1 = 0;//计数
    static int16_t RxBuffer1[10] = {0};
    static uint8_t RxState = 0;
    static uint8_t RxFlag1 = 0;

    if (RxState == 0 && com_data == 0x2C)  //0x2c帧头
    {

        RxState = 1;
        RxBuffer1[RxCounter1++] = com_data;
        //HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    } else if (RxState == 1 && com_data == 0x12)  //0x12帧头
    {
        //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        RxState = 2;
        RxBuffer1[RxCounter1++] = com_data;
    } else if (RxState == 2) {

        RxBuffer1[RxCounter1++] = com_data;
        if (RxCounter1 >= 10 || com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
        {
            RxState = 3;
            RxFlag1 = 1;

            mv_Cx = RxBuffer1[RxCounter1 - 5];
            mv_Cy = RxBuffer1[RxCounter1 - 4];
            mv_Cw = RxBuffer1[RxCounter1 - 3];
            mv_Ch = RxBuffer1[RxCounter1 - 2];

            error_angle = mv_Cx;
            distance_target = mv_Cy;
            spin_flag=mv_Cw;
            stop_flag = mv_Ch;
//               if(RxState==1)
//          {
//            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//          }
//          else if(RxState!=1&&RxState!=0)
//          {
//            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
//          }
        }
    } else if (RxState == 3)        //检测是否接受到结束标志
    {
        if (RxBuffer1[RxCounter1 - 1] == 0x5B) {

            RxFlag1 = 0;
            RxCounter1 = 0;
            RxState = 0;

        } else   //接收错误
        {
            RxState = 0;
            RxCounter1 = 0;
            for (i = 0; i < 10; i++) {
                RxBuffer1[i] = 0x00;      //将存放数据数组清零
            }
        }
    } else   //接收异常
    {
        RxState = 0;
        RxCounter1 = 0;
        for (i = 0; i < 10; i++) {
            RxBuffer1[i] = 0x00;      //将存放数据数组清零
        }
    }
    HAL_UART_Receive_IT(&huart2,(void *)&uart2_rxbuff,1);
}

/***************MV2---usart3***********************/

void openmv2_Init(void) {
    HAL_UART_Receive_IT(&huart3, (void *) &uart3_rxbuff, 1);
}

void Openmv2_Receive_Data(int16_t com_data) {

    uint8_t i;
    static uint8_t RxCounter1 = 0;//计数
    static int16_t RxBuffer1[10] = {0};
    static uint8_t RxState = 0;
    static uint8_t RxFlag1 = 0;

    if (RxState == 0 && com_data == 0x2C)  //0x2c帧头
    {

        RxState = 1;
        RxBuffer1[RxCounter1++] = com_data;
        //HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    } else if (RxState == 1 && com_data == 0x12)  //0x12帧头
    {
        //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        RxState = 2;
        RxBuffer1[RxCounter1++] = com_data;
    } else if (RxState == 2) {

        RxBuffer1[RxCounter1++] = com_data;
        if (RxCounter1 >= 10 || com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
        {
            RxState = 3;
            RxFlag1 = 1;

            mv_Cx_2 = RxBuffer1[RxCounter1 - 5];
            mv_Cy_2 = RxBuffer1[RxCounter1 - 4];
            mv_Cw_2 = RxBuffer1[RxCounter1 - 3];
            mv_Ch_2 = RxBuffer1[RxCounter1 - 2];

            error_angle_mv2 = mv_Cx_2;
            distance_target_mv2 = mv_Cy_2;
            config_mv2=mv_Cw_2;
            stop_flag_mv2 = mv_Ch_2;
//               if(RxState==1)
//          {
//            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//          }
//          else if(RxState!=1&&RxState!=0)
//          {
//            HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
//          }
        }
    } else if (RxState == 3)        //检测是否接受到结束标志
    {
        if (RxBuffer1[RxCounter1 - 1] == 0x5B) {

            RxFlag1 = 0;
            RxCounter1 = 0;
            RxState = 0;

        } else   //接收错误
        {
            RxState = 0;
            RxCounter1 = 0;
            for (i = 0; i < 10; i++) {
                RxBuffer1[i] = 0x00;      //将存放数据数组清零
            }
        }
    } else   //接收异常
    {
        RxState = 0;
        RxCounter1 = 0;
        for (i = 0; i < 10; i++) {
            RxBuffer1[i] = 0x00;      //将存放数据数组清零
        }
    }
    HAL_UART_Receive_IT(&huart3,(void *)&uart3_rxbuff,1);
}