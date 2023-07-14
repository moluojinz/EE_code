//
// Created by ShiF on 2023/5/12.
//

#ifndef EE_CODE_OPENMV_H
#define EE_CODE_OPENMV_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
/***************MV1**********/
void Openmv_Receive_Data(int16_t com_data);
void openmv_Init(void);
extern int16_t mv_Cx, mv_Cy, mv_Cw, mv_Ch;   //Cx:左右偏差 左- 右+   Cy:摄像头到物体distance
extern int16_t error_angle,distance_target;
extern int16_t stop_flag;
/**********MV2*********/
void openmv2_Init(void);
void Openmv2_Receive_Data(int16_t com_data);
extern int16_t mv_Cx_2, mv_Cy_2, mv_Cw_2, mv_Ch_2;   //Cx:左右偏差 左- 右+   Cy:摄像头到物体distance
extern int16_t error_angle_mv2,distance_target_mv2,config_mv2;
extern int16_t stop_flag_mv2;
#endif //EE_CODE_OPENMV_H
