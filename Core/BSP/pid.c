//
// Created by ShiF on 2023/5/9.
//

#include "pid.h"
//噪声放大效应严重
void PID_Update(PID_t* WhichPID, float NowInput)
{ //更新PID的数据，即更新PID的输入值
    WhichPID->PID_Input = NowInput;
    /*if (WhichPID->State_RampOrNormal == Ramp_e)
    {
        if (WhichPID->RampCountTime < WhichPID->RampTartgetTime)
        {
            ++WhichPID->RampCountTime;
        }
        else
        {
            WhichPID->RampCountTime = 0;
            if (WhichPID->PID_Target < WhichPID->RampTarget)
            { //斜坡函数还没计数完，
                WhichPID->PID_Target += WhichPID->RampTartgetStep;
                if (WhichPID->PID_Target >= WhichPID->RampTarget)
                {
                    WhichPID->PID_Target = WhichPID->RampTarget;
                    WhichPID->State_RampOrNormal = Normal_e;
                }
            }
            else if (WhichPID->PID_Target > WhichPID->RampTarget)
            {
                WhichPID->PID_Target -= WhichPID->RampTartgetStep;
                if (WhichPID->PID_Target <= WhichPID->RampTarget)
                {
                    WhichPID->PID_Target = WhichPID->RampTarget;
                    WhichPID->State_RampOrNormal = Normal_e;
                }
            }
            else
            { //斜坡函数计数已经完成，要退出斜坡函数模式
                WhichPID->State_RampOrNormal = Normal_e;
            }
        }
    } //判断是否为斜坡*///判定斜坡
    WhichPID->PID_Err_lastlast = WhichPID->PID_Err_last;
    WhichPID->PID_Err_last = WhichPID->PID_Err_now;
    WhichPID->PID_Err_now = WhichPID->PID_Target - NowInput;

    if (WhichPID->PID_Err_now < WhichPID->PID_Precision && WhichPID->PID_Err_now > -WhichPID->PID_Precision)
        WhichPID->PID_Err_now = 0; //使用精度

    WhichPID->PID_Err_all += WhichPID->PID_Err_now;

    if (WhichPID->PID_Err_all > WhichPID->PID_ErrAllMax)
    {
        WhichPID->PID_Err_all = WhichPID->PID_ErrAllMax;
    }
    else if (WhichPID->PID_Err_all < -WhichPID->PID_ErrAllMax)
    {
        WhichPID->PID_Err_all = -WhichPID->PID_ErrAllMax;
    }
}

float PID_GetPositionPID(PID_t* WhichPID)//位置式PID
{
    WhichPID->PID_Out =
            WhichPID->Kp1 * WhichPID->PID_Err_now + WhichPID->Kd1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last);
    WhichPID->PID_Out += (WhichPID->PID_Err_all * WhichPID->Ki1); //pid核心计算式

    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)
        WhichPID->PID_Out = WhichPID->PID_OutMax;
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax; //总输出限幅

    if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep)
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
    }
    if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep)
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;
    } //微分限幅

    WhichPID->PID_lastout = WhichPID->PID_Out;

    return WhichPID->PID_Out;
}

float PID_GetIncrementalPID(PID_t* WhichPID)//用不了
{
    WhichPID->PID_Out +=
            WhichPID->Ki1 * WhichPID->PID_Err_now + WhichPID->Kp1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last);
    WhichPID->PID_Out +=
            WhichPID->Kd1 * (WhichPID->PID_Err_now - 2 * WhichPID->PID_Err_last + WhichPID->PID_Err_lastlast);

    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)
        WhichPID->PID_Out = WhichPID->PID_OutMax;
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax;

    if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep)
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
    }
    if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep)
    {
        WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;
    }

    WhichPID->PID_lastout = WhichPID->PID_Out;
    return WhichPID->PID_Out;
}

void PID_SpeedParamInit(PID_t* WhichPID)
{ //初始化PID的默认参数
    WhichPID->Kp1 = 10.0;
    WhichPID->Ki1 = 0.1;
    WhichPID->Kd1 = 0;
    WhichPID->PID_Err_now = 0.0;
    WhichPID->PID_Err_last = 0.0;
    WhichPID->PID_Err_lastlast = 0.0;
    WhichPID->PID_Err_all = 0.0;
    WhichPID->PID_Out = 0.0;
    WhichPID->PID_lastout = 0.0;
    WhichPID->PID_Target = 0.0;
    WhichPID->PID_Input = 0.0;
    WhichPID->State_RampOrNormal = Ramp_e;
    WhichPID->RampTarget = 0.0;
    WhichPID->RampCountTime = 0.0;
    WhichPID->RampTartgetTime = RampDefault_Time;
    WhichPID->RampTartgetStep = RampDefault_Step;
    WhichPID->PID_WorkType = PositionPID_e;
    WhichPID->Connected = 0;
    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}

void PID_PosParamInit(PID_t* WhichPID)
{
    WhichPID->Kp1 = 0.3;
    WhichPID->Ki1 = 0;
    WhichPID->Kd1 = 0;
    WhichPID->PID_Err_now = 0.0;
    WhichPID->PID_Err_last = 0.0;
    WhichPID->PID_Err_lastlast = 0.0;
    WhichPID->PID_Err_all = 0.0;
    WhichPID->PID_Out = 0.0;
    WhichPID->PID_lastout = 0.0;
    WhichPID->PID_Target = 0.0;
    WhichPID->PID_Input = 0.0;
    WhichPID->State_RampOrNormal = Normal_e;
    WhichPID->RampTarget = 0.0;
    WhichPID->RampCountTime = 0.0;
    WhichPID->RampTartgetTime = RampDefault_Time;
    WhichPID->RampTartgetStep = RampDefault_Step;
    WhichPID->PID_WorkType = PositionPID_e;
    WhichPID->Connected = 0;
    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}

void PID_SetTargetWithRamp(PID_t* WhichPID, float Tartget)
{ //设置PID的目标值，使用斜坡函数
    if (WhichPID->RampTarget != Tartget)
    {
        WhichPID->RampTarget = Tartget;
        WhichPID->State_RampOrNormal = Ramp_e;
    }
}

void PID_Clear(PID_t* WhichPID)
{
    WhichPID->PID_Err_now = 0.0;
    WhichPID->PID_Err_last = 0.0;
    WhichPID->PID_Err_lastlast = 0.0;
    WhichPID->PID_Err_all = 0.0;
    WhichPID->PID_Out = 0.0;
    WhichPID->PID_lastout = 0.0;
    WhichPID->PID_Target = 0.0;
    WhichPID->PID_Input = 0.0;
    WhichPID->State_RampOrNormal = Normal_e;
    WhichPID->RampTarget = 0.0;
    WhichPID->RampCountTime = 0.0;
}
