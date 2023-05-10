//
// Created by ShiF on 2023/5/9.
//

#include "encoder.h"
#include "tim.h"
#include "debugc.h"
#include "pid.h"
#include "math.h"
//#include "stm32f1xx_hal_gpio.h"
#include "main.h"

float SetSpd_F,SetSpd_BL,SetPos_BR;     //设置转速
float SetPos_F,SetPos_BL,SetPos_BR;     //设置位置环

float Speed=0.0;
float Position_R=0.0,Position_L=0.0;
int16_t Position = 0;
int16_t Diretion = 0;
int16_t test_CNT_tim1 = 0;
int16_t test_CNT_tim2 = 0;
PID_t MotorSpeedPID_F,MotorSpeedPID_BL,MotorSpeedPID_BR;
//PID_t	MotorPosPID_R,MotorPosPID_L;
static int TIMER = 0;

int ship =  1 ; //ship(单片机):1 和 0


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim4)
    {
//        Motor_SpeedC_R();
//        TIMER++;
//		usart_printf("%d\r\n",TIMER);
    }
}

void TIM_StartEncorder(void)
{
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_GPIO_WritePin(FIN_A_GPIO_Port, FIN_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FIN_B_GPIO_Port, FIN_B_Pin, GPIO_PIN_RESET);//默认正转
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    MotorSpeedPID_F.PID_Target = 0;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCounter (&htim1, 0);  //计数值重新置为30000
}

int16_t TIM_GetEncorder(uint8_t Which) //5ms调用一次
{
    switch (Which)
    {
        case 1:
            Diretion = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
            Speed = (short)(__HAL_TIM_GET_COUNTER(&htim2));//0—65535
            test_CNT_tim1 = Speed;
            Position_R += test_CNT_tim1 ;
            //usart_printf("%d\r\n",Speed);
                    __HAL_TIM_SetCounter (&htim1, 0);  //计数值重新清零

            break;

    }
    //usart_printf("%d\r\n", Speed);
    return Speed;
}

float Motor_StateUpdate(float Speed_cnt) //如何转换单位？
{
    Speed_cnt = Speed_cnt  * 60 / (2000 * 30* 0.1); //M计数法 单位rpm
    return Speed_cnt;
}

void PID_Init()
{
    PID_SpeedParamInit(&MotorSpeedPID_F);

}
static int8_t flag = 0;

/**************/
/*          电机配置           */
/*            前轮            */
/*     A:PB15                */
/*     b:PB14                */
/*    EA:PA9 ----> TIM1->CH2 */
/*    EA:PA8 ----> TIM1->CH1 */
/*   PWM:PB6 ----> TIM4->CH1 */
/**************/

void Motor_SpeedC_F(void)
{
    float Speed_F=0.0;
    /*//速度环PID参数
    MotorSpeedPID.Kp1 = 0.5;
    MotorSpeedPID.Ki1 = 0;
    MotorSpeedPID.Kd1 = 0;
    //MotorSpeedPID.PID_Target = Debug_Param().pos_targetAngle; //速度环调试用*/
    MotorSpeedPID_F.PID_OutMax = 450;

    //位置环PID参数
    MotorSpeedPID_F.Kp1 = 10;
    MotorSpeedPID_F.Ki1 = 1;
    MotorSpeedPID_F.Kd1 = 0; //参数自己确定
    MotorSpeedPID_F.PID_OutMax = 333;
    MotorSpeedPID_F.PID_Target = /*Debug_Param().pos_targetAngle*/SetPos_F;

    //利用VOFA+外部传输值进行实时调整参数
    MotorSpeedPID_F.Kp1 = /*5*/Debug_Param().vel_kp;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Kp1);//调试外部输入功能
    MotorSpeedPID_F.Ki1 = /*0.2*/Debug_Param().vel_ki;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Ki1);
    MotorSpeedPID_F.Kd1 = /*0.5*/Debug_Param().vel_kd;
    // usart_printf("%.2f\r\n",MotorSpeedPID.Kd1);
    MotorSpeedPID_F.PID_Target=/*SetSpd_R*/Debug_Param().vel_rampTargetValue;
    //usart_printf("%.2f\r\n",MotorSpeedPID.PID_Target);


    Speed_F=(float)TIM_GetEncorder(4);          //读取编码器数值

    Speed_F=Motor_StateUpdate(Speed_F);
    //位置环PID
    PID_Update(&MotorSpeedPID_F, Position_R);
    PID_GetPositionPID(&MotorSpeedPID_F);

    //速度环PID
    //MotorSpeedPID.PID_Target = MotorPosPID.PID_Out;
    PID_Update(&MotorSpeedPID_F, (float)Speed_F);
    PID_GetPositionPID(&MotorSpeedPID_F);

    if (MotorSpeedPID_F.PID_Out < 0)
    {
        HAL_GPIO_WritePin(FIN_A_GPIO_Port, FIN_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(FIN_B_GPIO_Port, FIN_B_Pin, GPIO_PIN_RESET);
        MotorSpeedPID_F.PID_Out = -MotorSpeedPID_F.PID_Out;
    }
    else
    {
        HAL_GPIO_WritePin(FIN_A_GPIO_Port, FIN_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FIN_B_GPIO_Port, FIN_B_Pin, GPIO_PIN_SET);
        MotorSpeedPID_F.PID_Out = MotorSpeedPID_F.PID_Out;
    }
    // MotorSpeedPID.PID_Out = 200;
    usart_printf("%f,%d,%f,%d\r\n", MotorSpeedPID_F.PID_Out, Speed_F, MotorSpeedPID_F.PID_Target,test_CNT_tim1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MotorSpeedPID_F.PID_Out);
//	//以下是额外的，使电机转动一定时间，可以不用管

}



