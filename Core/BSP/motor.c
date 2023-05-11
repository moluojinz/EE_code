//
// Created by ShiF on 2023/5/9.
//
#include "bsp_headfile.h"
#include "motor.h"

float update_Des(float set)         //位置环参数转换
{
    return set;
}

/*******自转********/
/*  三个轮子同向转动 */
/******************/
void CAR_spin(float des)
{
    SetPos_F+=des;
    SetPos_BR+=des;
    SetPos_BL+=des;
}

/*******纵向********/
/*   前轮作从动轮   */
/*左后轮和右后轮作驱动轮*/
/******************/
void CAR_longitudinal(float des)
{
    SetPos_F+=0;
    SetPos_BL+=-des/((float)COS_30);
    SetPos_BR+=des/((float)COS_30);
}
/*******横向********/
/*三个轮子同时作为驱动轮*/
/*前轮为主导方向的轮子*/
/*后两轮转向相同，且与前轮转向相反*/
/*Des为正时向左平移*/
/******************/
void CAR_transverse(float des)
{
    SetPos_F+=-des;
    SetPos_BL+=des/((float)COS_60);
    SetPos_BR+=des/((float)COS_60);
}