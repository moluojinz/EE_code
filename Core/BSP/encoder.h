//
// Created by ShiF on 2023/5/9.
//

#ifndef EE_CODE_ENCODER_H
#define EE_CODE_ENCODER_H

#include "stm32f1xx_hal.h"
int16_t TIM_GetEncorder(uint8_t Which);
float Motor_StateUpdate(float Speed_cnt);
void TIM_StartEncorder(void);
void Motor_SpeedC_L(void);
void Motor_SpeedC_F(void);
void PID_Init();
extern uint32_t Set_L,Set_R;
void encoder_Init(void);

#endif //EE_CODE_ENCODER_H
