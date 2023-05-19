//
// Created by ShiF on 2023/5/16.
//
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "LED_BEEP.h"

void tb_LED_start(void) {
    HAL_GPIO_WritePin(tb_LED_GPIO_Port,tb_LED_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
}

void tb_stop(void) {
    HAL_GPIO_WritePin(tb_LED_GPIO_Port,tb_LED_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
}

void tb_BEEP_start(void){

}