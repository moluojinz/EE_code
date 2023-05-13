//
// Created by ShiF on 2023/5/12.
//

#ifndef EE_CODE_OPENMV_H
#define EE_CODE_OPENMV_H
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

void  Openmv_Receive_Data(int16_t data);
void openmv_Init(void);


extern uint8_t  Cx, Cy, Cw, Ch;   //Cx:左右偏差 左- 右+   Cy:摄像头到物体distance



#endif //EE_CODE_OPENMV_H
