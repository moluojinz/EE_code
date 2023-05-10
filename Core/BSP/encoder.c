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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

float SetSpd_L,SetSpd_R;       //设置左右转速
float SetPos_L,SetPos_R;     //设置左右位置环

int16_t test_CNT = 0;
float Speed=0.0;
float Position_R=0.0,Position_L=0.0;
int16_t Position = 0;
int16_t Diretion = 0;
int16_t test_CNT_tim4 = 0;
int16_t test_CNT_tim2 = 0;
PID_t MotorSpeedPID_R,MotorSpeedPID_L;
PID_t	MotorPosPID_R,MotorPosPID_L;
static int TIMER = 0;

int ship =  1 ; //ship(单片机):1 和 0


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim3)
    {
//        Motor_SpeedC_R();
//        TIMER++;
//		usart_printf("%d\r\n",TIMER);
    }
}

void TIM_StartEncorder(void)
{
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1 | TIM_CHANNEL_2);
//    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);//默认正转
//    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);//默认正转
    //HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    MotorSpeedPID_L.PID_Target = 0;
    MotorSpeedPID_R.PID_Target = 0;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            __HAL_TIM_SetCounter (&htim4, 0);  //计数值重新置为30000
            __HAL_TIM_SetCounter (&htim2, 0);  //计数值重新置为30000
}

int16_t TIM_GetEncorder(uint8_t Which) //5ms调用一次
{
    switch (Which)
    {
        case 2:
            Diretion = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
            Speed = (short)(__HAL_TIM_GET_COUNTER(&htim2));//0—65535
            test_CNT_tim2 = Speed;
            Position_R += test_CNT_tim2 ;
            //usart_printf("%d\r\n",Speed);
                    __HAL_TIM_SetCounter (&htim2, 0);  //计数值重新清零

            break;
        case 4:
            Diretion = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
            Speed = (short)(__HAL_TIM_GET_COUNTER(&htim4));//0—65535
            test_CNT_tim4=Speed;
            Position_L += test_CNT_tim4 ;
            //usart_printf("%d\r\n",Speed);
                    __HAL_TIM_SetCounter (&htim4, 0);  //计数值重新清零

            break;
    }
    //usart_printf("%d\r\n", Speed);
    return Speed;
}

//float Motor_StateUpdate(float Speed_f) //如何转换单位？
//{
//    Speed_f = Speed_f  * 60 / (2000 * 30* 0.1); //M计数法 单位rpm
//    return Speed_f;
//}

void PID_Init()
{
    PID_SpeedParamInit(&MotorSpeedPID_R);
    PID_SpeedParamInit(&MotorSpeedPID_L);
}
static int8_t flag = 0;
static int8_t maxtime = 60;
void Motor_SpeedC_R(void)
{
    float Speed_R=0.0;
    /*//速度环PID参数
    MotorSpeedPID.Kp1 = 0.5;
    MotorSpeedPID.Ki1 = 0;
    MotorSpeedPID.Kd1 = 0;
    //MotorSpeedPID.PID_Target = Debug_Param().pos_targetAngle; //速度环调试用*/
    MotorSpeedPID_R.PID_OutMax = 450;

    //位置环PID参数
    MotorPosPID_R.Kp1 = 10;
    MotorPosPID_R.Ki1 = 1;
    MotorPosPID_R.Kd1 = 0; //参数自己确定
    MotorPosPID_R.PID_OutMax = 333;
    MotorPosPID_R.PID_Target = /*Debug_Param().pos_targetAngle*/SetPos_R;

    //利用VOFA+外部传输值进行实时调整参数
    MotorSpeedPID_R.Kp1 = /*5*/Debug_Param().vel_kp;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Kp1);//调试外部输入功能
    MotorSpeedPID_R.Ki1 = /*0.2*/Debug_Param().vel_ki;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Ki1);
    MotorSpeedPID_R.Kd1 = /*0.5*/Debug_Param().vel_kd;
    // usart_printf("%.2f\r\n",MotorSpeedPID.Kd1);
    MotorSpeedPID_R.PID_Target=/*SetSpd_R*/Debug_Param().vel_rampTargetValue;
    //usart_printf("%.2f\r\n",MotorSpeedPID.PID_Target);


    Speed_R=(float)TIM_GetEncorder(4); //读取编码器数值

    Speed_R=Motor_StateUpdate(Speed_R);
    //usart_printf("%f\r\n",Speed);
    //位置环PID
    PID_Update(&MotorPosPID_R, Position_R);
    PID_GetPositionPID(&MotorPosPID_R);

    //速度环PID
    //MotorSpeedPID.PID_Target = MotorPosPID.PID_Out;
    PID_Update(&MotorSpeedPID_R, (float)Speed_R);
    PID_GetPositionPID(&MotorSpeedPID_R);
    //usart_printf("%f,%d,%f\r\n", MotorSpeedPID.PID_Out, Speed, MotorSpeedPID.PID_Target);
    if (MotorSpeedPID_R.PID_Out < 0)
    {
//        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        MotorSpeedPID_R.PID_Out = -MotorSpeedPID_R.PID_Out;
    }
    else
    {
//        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        MotorSpeedPID_R.PID_Out = MotorSpeedPID_R.PID_Out;
    }
    // MotorSpeedPID.PID_Out = 200;
    usart_printf("%f,%d,%f,%d\r\n", MotorSpeedPID_R.PID_Out, Speed, MotorSpeedPID_R.PID_Target,test_CNT);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MotorSpeedPID_R.PID_Out);
//	//以下是额外的，使电机转动一定时间，可以不用管

//	if (TIMER > maxtime)
//	{
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
//		if (!flag)
//		{
//			HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_SET);
//			HAL_Delay(1000);
//			HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_RESET);
//			//TIMER = 0 ;
//		}
//		flag = 1;
//	}
}

void Motor_SpeedC_L(void)
{
    float Speed_L=0.0;
    /*//速度环PID参数
    MotorSpeedPID.Kp1 = 0.5;
    MotorSpeedPID.Ki1 = 0;
    MotorSpeedPID.Kd1 = 0;
    //MotorSpeedPID.PID_Target = Debug_Param().pos_targetAngle; //速度环调试用*/
    MotorSpeedPID_L.PID_OutMax = 450;

    //位置环PID参数
    MotorPosPID_L.Kp1 = 10;
    MotorPosPID_L.Ki1 = 1;
    MotorPosPID_L.Kd1 = 0; //参数自己确定
    MotorPosPID_L.PID_OutMax = 333;
    MotorPosPID_L.PID_Target = SetSpd_R/*Debug_Param().pos_targetAngle*/;

    //利用VOFA+外部传输值进行实时调整参数
    MotorSpeedPID_L.Kp1 = /*5*/Debug_Param().vel_kp;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Kp1);//调试外部输入功能
    MotorSpeedPID_L.Ki1 = /*0.2*/Debug_Param().vel_ki;
    //usart_printf("%.2f\r\n",MotorSpeedPID.Ki1);
    MotorSpeedPID_L.Kd1 = /*0.5*/Debug_Param().vel_kd;
    // usart_printf("%.2f\r\n",MotorSpeedPID.Kd1);
    MotorSpeedPID_L.PID_Target=/*SetSpd_R*/Debug_Param().vel_rampTargetValue;
    //usart_printf("%.2f\r\n",MotorSpeedPID.PID_Target);


    TIM_GetEncorder(2); //读取编码器数值

    Speed_L=Motor_StateUpdate(Speed_L);
    //usart_printf("%f\r\n",Speed);
    //位置环PID
    PID_Update(&MotorPosPID_L, Position_L);
    PID_GetPositionPID(&MotorPosPID_L);

    //速度环PID
    //MotorSpeedPID.PID_Target = MotorPosPID.PID_Out;
    PID_Update(&MotorSpeedPID_L, Speed_L);
    PID_GetPositionPID(&MotorSpeedPID_L);
    //usart_printf("%f,%d,%f\r\n", MotorSpeedPID.PID_Out, Speed, MotorSpeedPID.PID_Target);
    if (MotorSpeedPID_L.PID_Out < 0)
    {
//        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        MotorSpeedPID_L.PID_Out = -MotorSpeedPID_L.PID_Out;
    }
    else
    {
//        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
//        MotorSpeedPID_L.PID_Out = MotorSpeedPID_L.PID_Out;
    }
    // MotorSpeedPID.PID_Out = 200;
    usart_printf("%f,%d,%f,%d\r\n", MotorSpeedPID_L.PID_Out, Speed, MotorSpeedPID_L.PID_Target,test_CNT);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorSpeedPID_L.PID_Out);
//	//以下是额外的，使电机转动一定时间，可以不用管

//	if (TIMER > maxtime)
//	{
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
//		if (!flag)
//		{
//			HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_SET);
//			HAL_Delay(1000);
//			HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_RESET);
//			//TIMER = 0 ;
//		}
//		flag = 1;
//	}
}
//外部中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //__HAL_TIM_SET_COUNTER(&htim5, 0);
    switch (GPIO_Pin)
    {
        case GPIO_PIN_0:
//		MotorSpeedPID.PID_Target = 40;
//		flag = 0;
//		TIMER = 0;
//		maxtime = 60;
            break;
        case GPIO_PIN_4:
//		MotorSpeedPID.PID_Target = 140;
//		flag = 0;
//		TIMER = 0;
//		maxtime = 30;
            break;
    }
    //HAL_Delay(200);
}

