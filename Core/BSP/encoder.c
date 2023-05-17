//
// Created by ShiF on 2023/5/9.
//

#include "encoder.h"
#include "tim.h"
#include "debugc.h"
#include "pid.h"
#include "math.h"
#include "bsp_headfile.h"
#include "openmv.h"
//#include "stm32f1xx_hal_gpio.h"
#include "main.h"

float SetSpd_F, SetSpd_BL, SetSpd_BR;     //设置转速
float SetPos_F, SetPos_BL, SetPos_BR;     //设置位置环
float Speed_BL = 0.0;
float Speed_BR = 0.0;
float Speed_F = 0.0;
float Speed = 0.0;
float Position_F = 0.0, Position_BL = 0.0, Position_BR = 0.0;
int16_t Diretion = 0;

int16_t test_CNT_tim1 = 0;
int16_t test_CNT_tim2 = 0;
int16_t test_CNT_tim3 = 0;
PID_t MotorSpeedPID_F, MotorSpeedPID_BL, MotorSpeedPID_BR;
PID_t MotorPosPID_F, MotorPosPID_BL, MotorPosPID_BR;
static int TIMER = 0;


void encoder_Init(void) {
    TIM_StartEncorder();
    PID_Init();

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim4)             //10ms中断
    {
        Motor_SpeedC_F();
        Motor_SpeedC_BL();
        Motor_SpeedC_BR();
//        usart_printf("%d,%d,%d,%d\r\n", Cx, Cy, Cw, Ch);
//        TIMER++;
//		usart_printf("%.3f,%.3f\r\n",Speed_BL,Speed_BR);
    }
}

void TIM_StartEncorder(void) {
    HAL_TIM_Base_Start_IT(&htim4);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_GPIO_WritePin(FIN_A_GPIO_Port, FIN_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FIN_B_GPIO_Port, FIN_B_Pin, GPIO_PIN_RESET);//默认正转
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    MotorSpeedPID_F.PID_Target = 0;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SetCounter (&htim1, 0);  //计数值重新置为0

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_GPIO_WritePin(BLIN_A_GPIO_Port, BLIN_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLIN_B_GPIO_Port, BLIN_B_Pin, GPIO_PIN_RESET);//默认正转
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    MotorSpeedPID_F.PID_Target = 0;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SetCounter (&htim2, 0);  //计数值重新置为0

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_GPIO_WritePin(BRIN_A_GPIO_Port, BRIN_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BRIN_B_GPIO_Port, BRIN_B_Pin, GPIO_PIN_RESET);//默认正转
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    MotorSpeedPID_F.PID_Target = 0;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
            __HAL_TIM_SetCounter (&htim3, 0);  //计数值重新置为0
}

int16_t TIM_GetEncorder(uint8_t Which) //5ms调用一次
{
    switch (Which) {
        case 1:
            Diretion = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);
            Speed = (short) (__HAL_TIM_GET_COUNTER(&htim1));//0—65535
            test_CNT_tim1 = Speed;
            Position_F += test_CNT_tim1;
            //usart_printf("%d\r\n",Speed);
                    __HAL_TIM_SetCounter (&htim1, 0);  //计数值重新清零
            break;
        case 2:
            Diretion = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
            Speed = (short) (__HAL_TIM_GET_COUNTER(&htim2));//0—65535
            test_CNT_tim2 = Speed;
            Position_BL += test_CNT_tim2;
            //usart_printf("%d\r\n",Speed);
                    __HAL_TIM_SetCounter (&htim2, 0);  //计数值重新清零
            break;
        case 3:
            Diretion = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
            Speed = (short) (__HAL_TIM_GET_COUNTER(&htim3));//0—65535
            test_CNT_tim3 = Speed;
            Position_BR += test_CNT_tim3;
            //usart_printf("%d\r\n",Speed);
                    __HAL_TIM_SetCounter (&htim3, 0);  //计数值重新清零
            break;
    }
    //usart_printf("%d\r\n", Speed);
    return Speed;
}

float Motor_StateUpdate(float Speed_cnt) //如何转换单位？
{
    Speed_cnt = Speed_cnt * 60 / (44 * 30 * 0.01); //M计数法 单位rpm
    return Speed_cnt;
}

void PID_Init() {
    PID_SpeedParamInit(&MotorSpeedPID_F);
    PID_SpeedParamInit(&MotorPosPID_F);
    PID_SpeedParamInit(&MotorSpeedPID_BL);
    PID_SpeedParamInit(&MotorPosPID_BL);
    PID_SpeedParamInit(&MotorSpeedPID_BR);
    PID_SpeedParamInit(&MotorPosPID_BR);
    MotorPosPID_F.PID_OutMax = 200;
    MotorPosPID_BL.PID_OutMax = 200;
    MotorPosPID_BL.PID_OutMax = 200;

}

static int8_t flag = 0;

/**************/
/*          电机配置           */
/*            后轮            */
/*     A:PB15                */
/*     B:PB14                */
/*    EA:PA9 ----> TIM1->CH2 */
/*    EB:PA8 ----> TIM1->CH1 */
/*   PWM:PB6 ----> TIM4->CH1 */
/**************/

void Motor_SpeedC_F(void) {

    /*//速度环PID参数
    MotorSpeedPID.Kp1 = 0.5;
    MotorSpeedPID.Ki1 = 0;
    MotorSpeedPID.Kd1 = 0;
    //MotorSpeedPID.PID_Target = Debug_Param().pos_targetAngle; //速度环调试用*/
    MotorSpeedPID_F.PID_OutMax = 450;

    //位置环PID参数
    MotorPosPID_F.Kp1 = 2 /*Debug_Param().vel_kp*/;
    MotorPosPID_F.Ki1 = 0/*Debug_Param().vel_ki*/;
    MotorPosPID_F.Kd1 = 0.3/*Debug_Param().vel_kd*/; //参数自己确定
//    MotorPosPID_F.PID_OutMax = 333;
    MotorPosPID_F.PID_Target = update_Des(SetPos_F)/*Debug_Param().vel_rampTargetValueDebug_Param().vel_ki*/;

    //利用VOFA+外部传输值进行实时调整参数
    MotorSpeedPID_F.Kp1 = 3/*Debug_Param().vel_kp*/;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Kp1);//调试外部输入功能
    MotorSpeedPID_F.Ki1 = 0.11/*Debug_Param().vel_ki*/;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Ki1);
    MotorSpeedPID_F.Kd1 = 0.05/*Debug_Param().vel_kd*/;
    // usart_printf("%.2f\r\n",MotorSpeedPID.Kd1);
//    MotorSpeedPID_F.PID_Target=20/*Debug_Param().vel_rampTargetValue*/;
    //usart_printf("%.2f\r\n",MotorSpeedPID.PID_Target);


    Speed_F = (float) TIM_GetEncorder(1);          //读取编码器数值

    Speed_F = Motor_StateUpdate(Speed_F);
    //位置环PID
    PID_Update(&MotorPosPID_F, Position_F);
    PID_GetPositionPID(&MotorPosPID_F);

    //速度环PID
    MotorSpeedPID_F.PID_Target = /*50*/MotorPosPID_F.PID_Out;
    PID_Update(&MotorSpeedPID_F, (float) Speed_F);
    PID_GetPositionPID(&MotorSpeedPID_F);

    if (MotorSpeedPID_F.PID_Out < 0) {
        HAL_GPIO_WritePin(FIN_A_GPIO_Port, FIN_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FIN_B_GPIO_Port, FIN_B_Pin, GPIO_PIN_SET);
        MotorSpeedPID_F.PID_Out = -MotorSpeedPID_F.PID_Out;
    } else {
        HAL_GPIO_WritePin(FIN_A_GPIO_Port, FIN_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(FIN_B_GPIO_Port, FIN_B_Pin, GPIO_PIN_RESET);
        MotorSpeedPID_F.PID_Out = MotorSpeedPID_F.PID_Out;
    }
    // MotorSpeedPID.PID_Out = 200;
//    usart_printf("%f,%f,%f,%d\r\n", Speed_F, Speed_BL, Speed_BR,test_CNT_tim1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MotorSpeedPID_F.PID_Out);


}


/**************/
/*          电机配置           */
/*            左后轮          */
/*     A:PB13                */
/*     B:PB12                */
/*    EA:PA1 ----> TIM2->CH2 */
/*    EB:PA0 ----> TIM2->CH1 */
/*   PWM:PB7 ----> TIM4->CH2 */
/**************/

void Motor_SpeedC_BL(void) {

    //速度环PID参数
    MotorSpeedPID_BL.Kp1 = 3.1;
    MotorSpeedPID_BL.Ki1 = 0.13;
    MotorSpeedPID_BL.Kd1 = 0.05;
    //MotorSpeedPID.PID_Target = Debug_Param().pos_targetAngle; //速度环调试用
    MotorSpeedPID_BL.PID_OutMax = 450;

    //位置环PID参数
    MotorPosPID_BL.Kp1 = 2/*Debug_Param().vel_kp*/;
    MotorPosPID_BL.Ki1 = 0/*Debug_Param().vel_ki*/;
    MotorPosPID_BL.Kd1 = 0.3/*Debug_Param().vel_kd*/; //参数自己确定
//    MotorPosPID_BL.PID_OutMax = 333;
    MotorPosPID_BL.PID_Target = update_Des(SetPos_BL)/*Debug_Param().vel_kdDebug_Param().vel_rampTargetValue*/;

    //利用VOFA+外部传输值进行实时调整参数
//    MotorSpeedPID_BL.Kp1 = /*1.1*/Debug_Param().vel_kp;
//    //usart_printf("%.2f\r\n",MotorSpeedPID.Kp1);//调试外部输入功能
//    MotorSpeedPID_BL.Ki1 = /*0.2*/Debug_Param().vel_ki;
//    //usart_printf("%.2f\r\n",MotorSpeedPID.Ki1);
//    MotorSpeedPID_BL.Kd1 = /*0.5*/Debug_Param().vel_kd;
//    // usart_printf("%.2f\r\n",MotorSpeedPID.Kd1);
//    MotorSpeedPID_BL.PID_Target=/*20*/Debug_Param().vel_rampTargetValue;
    //usart_printf("%.2f\r\n",MotorSpeedPID.PID_Target);

    Speed_BL = (float) TIM_GetEncorder(2);          //读取编码器数值

    Speed_BL = Motor_StateUpdate(Speed_BL);
    //位置环PID
    PID_Update(&MotorPosPID_BL, Position_BL);
    PID_GetPositionPID(&MotorPosPID_BL);

    //速度环PID
    MotorSpeedPID_BL.PID_Target = /*50*/MotorPosPID_BL.PID_Out;
    PID_Update(&MotorSpeedPID_BL, (float) Speed_BL);
    PID_GetPositionPID(&MotorSpeedPID_BL);

    if (MotorSpeedPID_BL.PID_Out < 0) {
        HAL_GPIO_WritePin(BLIN_A_GPIO_Port, BLIN_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BLIN_B_GPIO_Port, BLIN_B_Pin, GPIO_PIN_RESET);
        MotorSpeedPID_BL.PID_Out = -MotorSpeedPID_BL.PID_Out;
    } else {
        HAL_GPIO_WritePin(BLIN_A_GPIO_Port, BLIN_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLIN_B_GPIO_Port, BLIN_B_Pin, GPIO_PIN_SET);
        MotorSpeedPID_BL.PID_Out = MotorSpeedPID_BL.PID_Out;
    }
    // MotorSpeedPID.PID_Out = 200;
//    usart_printf("%f,%f,%f,%d\r\n", MotorSpeedPID_BL.PID_Out, Speed_BL, MotorSpeedPID_BL.PID_Target,test_CNT_tim2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MotorSpeedPID_BL.PID_Out);


}



/**************/
/*          电机配置           */
/*            右后轮  -      */
/*     A:PA11                */
/*     B:PA10                */
/*    EA:PA6 ----> TIM3->CH1 */
/*    EB:PA7 ----> TIM3->CH2 */
/*   PWM:PB8 ----> TIM4->CH3 */
/**************/

void Motor_SpeedC_BR(void) {

    //速度环PID参数
    MotorSpeedPID_BR.Kp1 = 3.1;
    MotorSpeedPID_BR.Ki1 = 0.13;
    MotorSpeedPID_BR.Kd1 = 0.05;
    //MotorSpeedPID.PID_Target = Debug_Param().pos_targetAngle; //速度环调试用
    MotorSpeedPID_BR.PID_OutMax = 450;

    //位置环PID参数
    MotorPosPID_BR.Kp1 = 2/*Debug_Param().vel_kp*/;
    MotorPosPID_BR.Ki1 = 0/*Debug_Param().vel_ki*/;
    MotorPosPID_BR.Kd1 = 0.3/*Debug_Param().vel_kd*/; //参数自己确定
//    MotorPosPID_BR.PID_OutMax = 333;
    MotorPosPID_BR.PID_Target = update_Des(SetPos_BR)/*Debug_Param().vel_rampTargetValue*/;

    //利用VOFA+外部传输值进行实时调整参数
//    MotorSpeedPID_BR.Kp1 = /*1.1*/Debug_Param().vel_kp;
//    //usart_printf("%.2f\r\n",MotorSpeedPID.Kp1);//调试外部输入功能
//    MotorSpeedPID_BR.Ki1 = /*0.2*/Debug_Param().vel_ki;
//    //usart_printf("%.2f\r\n",MotorSpeedPID.Ki1);
//    MotorSpeedPID_BR.Kd1 = /*0.5*/Debug_Param().vel_kd;
//    // usart_printf("%.2f\r\n",MotorSpeedPID.Kd1);
//    MotorSpeedPID_BR.PID_Target=/*20*/Debug_Param().vel_rampTargetValue;
//    //usart_printf("%.2f\r\n",MotorSpeedPID.PID_Target);


    Speed_BR = (float) TIM_GetEncorder(3);          //读取编码器数值

    Speed_BR = Motor_StateUpdate(Speed_BR);
    //位置环PID
    PID_Update(&MotorPosPID_BR, Position_BR);
    PID_GetPositionPID(&MotorPosPID_BR);

    //速度环PID
    MotorSpeedPID_BR.PID_Target = /*-50*/MotorPosPID_BR.PID_Out;
    PID_Update(&MotorSpeedPID_BR, (float) Speed_BR);
    PID_GetPositionPID(&MotorSpeedPID_BR);

    if (MotorSpeedPID_BR.PID_Out < 0) {
        HAL_GPIO_WritePin(BRIN_A_GPIO_Port, BRIN_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BRIN_B_GPIO_Port, BRIN_B_Pin, GPIO_PIN_RESET);
        MotorSpeedPID_BR.PID_Out = -MotorSpeedPID_BR.PID_Out;
    } else {
        HAL_GPIO_WritePin(BRIN_A_GPIO_Port, BRIN_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BRIN_B_GPIO_Port, BRIN_B_Pin, GPIO_PIN_SET);
        MotorSpeedPID_BR.PID_Out = MotorSpeedPID_BR.PID_Out;
    }
    // MotorSpeedPID.PID_Out = 200;
//    usart_printf("%f,%f,%f,%d\r\n", MotorSpeedPID_BR.PID_Out, Speed_BR, MotorSpeedPID_BR.PID_Target,test_CNT_tim3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MotorSpeedPID_BR.PID_Out);


}