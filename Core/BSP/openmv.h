//
// Created by ShiF on 2023/5/12.
//

#ifndef EE_CODE_OPENMV_H
#define EE_CODE_OPENMV_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

void Openmv_Receive_Data(int16_t com_data);

void openmv_Init(void);


extern int8_t mv_Cx, mv_Cy, mv_Cw, mv_Ch;   //Cx:左右偏差 左- 右+   Cy:摄像头到物体distance



#endif //EE_CODE_OPENMV_H
