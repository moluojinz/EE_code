//
// Created by ShiF on 2023/5/9.
//

#ifndef EE_CODE_ENCODER_H
#define EE_CODE_ENCODER_H

#include "stm32f1xx_hal.h"
#include "pid.h"
int16_t TIM_GetEncorder(uint8_t Which);
float Motor_StateUpdate(float Speed_cnt);
void TIM_StartEncorder(void);
void Motor_SpeedC_BL(void);
void Motor_SpeedC_BR(void);
void Motor_SpeedC_F(void);
void PID_Init();
void encoder_Init(void);



extern float SetPos_F,SetPos_BL,SetPos_BR;
extern uint32_t Set_L,Set_R;
extern PID_t MotorPosPID_F, MotorPosPID_BL, MotorPosPID_BR;

#endif //EE_CODE_ENCODER_H
