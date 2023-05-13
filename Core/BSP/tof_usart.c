#include "stm32f1xx_hal.h"
#include "usart.h"
#include "tof_usart.h"

static uint16_t RxBuffer[7] = {0};
#define tof_rvsize

void TOF_UartIdleCallback(UART_HandleTypeDef *huart) {
    uint8_t com_data;
    uint8_t i;
    static uint8_t RxCounter = 0;

    static uint8_t RxState = 0;


    if (RxState == 0 && com_data == 0x64) {
        RxState = 1;
        RxBuffer[RxCounter++] = com_data;
    } else if (RxState == 1 && com_data == 0x3A) {
        RxState = 2;
        RxBuffer[RxCounter++] = com_data;
    } else if (RxState == 2) {
        RxBuffer[RxCounter++] = com_data;
        if (RxCounter == 7 && com_data == 0x20) {

        }
    }
}