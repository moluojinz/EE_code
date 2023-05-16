//
// Created by ShiF on 2023/5/9.
//
#include "bsp_headfile.h"
#include "motor.h"

PID_t mv_measure;
PID_t tof_measure;
uint8_t spin_flag;
uint8_t second_flag;
uint32_t count_spin;                    //记录第二阶段第一部分的旋转次数
float record_spin;                      //记录自转PID调整时改变位姿
uint32_t count_pid;                     //pid次数限制，进入靠近阶段
#define count_pidLIM    200             //pid调整次数
uint8_t close_flag;                     //靠近状态标志
#define distance_target_th  20          //靠近距离目标值
PID_t dis_target;
uint32_t count_close;                   //靠近距离的PID次数
#define count_closeLIM  200             //靠近PID次数限制
float close_PIDOUT;
uint8_t second_reset_flag;            //第二阶段复位信号
uint8_t third_flag;                     //第三阶段开始信号（TOF识别矫正过程）

void motor_Init(void) {
    spin_flag = 0;
    second_flag = 0;
    record_spin = 0;
    count_pid = 0;
    close_flag = 0;
    count_spin = 0;
    count_close = 0;
    close_PIDOUT = 0.0;
    second_reset_flag = 0;
    third_flag=0;


    mv_measure.Kp1 = 0.0;
    mv_measure.Ki1 = 0.0;
    mv_measure.Kd1 = 0.0;
    mv_measure.PID_Target = 0.0;
    mv_measure.PID_OutMax = 20;
    dis_target.Kp1 = 0.0;
    dis_target.Ki1 = 0.0;
    dis_target.Kd1 = 0.0;
    dis_target.PID_Target = 0;
    mv_measure.PID_OutMax = 10;
}

float update_Des(float set)         //位置环参数转换
{
    set = set * 1320 / 7 / PI;
    return set;
}

/*******自转********/
/*  三个轮子同向转动 */
/******************/
void CAR_spin(float des) {
    SetPos_F += des;
    SetPos_BR -= des;
    SetPos_BL += des;
    MotorPosPID_F.PID_OutMax = 20;
    MotorPosPID_BL.PID_OutMax = 20;
    MotorPosPID_BR.PID_OutMax = 20;
}

/*******纵向********/
/*   前轮作从动轮   */
/*左后轮和右后轮作驱动轮*/
/******************/
void CAR_longitudinal(float des) {
    SetPos_F += 0;
    SetPos_BL += des * ((float) COS_30);
    SetPos_BR += des * ((float) COS_30);
//    MotorPosPID_F.PID_OutMax = 250;
    MotorPosPID_BL.PID_OutMax = 150 * ((float) COS_30);
    MotorPosPID_BR.PID_OutMax = 150 * ((float) COS_30);
}
/*******横向********/
/*三个轮子同时作为驱动轮*/
/*前轮为主导方向的轮子*/
/*后两轮转向相同，且与前轮转向相反*/
/*Des为正时向左平移*/
/******************/
void CAR_transverse(float des) {
    SetPos_F += des;
    SetPos_BL += -des * ((float) COS_60);
    SetPos_BR += des * ((float) COS_60);
    MotorPosPID_F.PID_OutMax = 150;
    MotorPosPID_BL.PID_OutMax = 150 * ((float) COS_60);
    MotorPosPID_BR.PID_OutMax = 150 * ((float) COS_60);
}
/*******TOF********/
/*获取TOF数据*/
/*变量定义*/

/******************/

#define Dest_1  110
#define Dest_2  100
#define Dest_3  100
#define Dest_4  100
#define Dest_5  100

//第一阶段
void motor_firststage_control(void) {
    //stage1
    CAR_longitudinal(Dest_1);
    HAL_Delay(5000);
    //stage2
    CAR_transverse(Dest_2);
    HAL_Delay(6000);
    //stage3
    CAR_longitudinal(Dest_3);
    HAL_Delay(6000);
    //stage4
    CAR_transverse(Dest_4);
    HAL_Delay(6000);
    //stage5
    CAR_longitudinal(Dest_5);
    HAL_Delay(6000);
    second_flag = 1;


}

//第二阶段
void motor_secondstage_control(void) {

    if (second_flag) {
        if (!spin_flag) {
            //自转等待识别信号
            CAR_spin(oneTURN / 10);         //转动十分之一圈等待识别
            HAL_Delay(1000);
            count_spin++;
        } else {
            if (count_pid <= count_pidLIM && !close_flag) {
                //pid对焦
                mv_measure.Kp1 = 0.0;
                mv_measure.Ki1 = 0.0;
                mv_measure.Kd1 = 0.0;
                mv_measure.PID_Target = 0.0;

                PID_Update(&mv_measure, error_angle);
                PID_GetPositionPID(&mv_measure);

                /*************待更改标记****************/
                CAR_spin(mv_measure.PID_Out);               //回传数据还没做更改，PID_OUT输出为正是右转，输出为负是左转
                /*************************************/
                record_spin += mv_measure.PID_Out;
                count_pid++;
            }                            //在一定PID调整次数内调整位姿后跳出PID
            else if (count_pid > count_pidLIM && !close_flag) {
                close_flag = 1;                                 //置靠近状态标志
                HAL_Delay(1000);                           //等待对焦PID调整结束
            } else if (close_flag) {
                if (count_close <= count_closeLIM) {
                    dis_target.Kp1 = 0.0;
                    dis_target.Ki1 = 0.0;
                    dis_target.Kd1 = 0.0;
                    dis_target.PID_Target = distance_target_th;
                    PID_Update(&dis_target, distance_target);
                    PID_GetPositionPID(&dis_target);

                    /*************待更改标记****************/
                    CAR_longitudinal(dis_target.PID_Out);               //靠近参数没做好
                    /*************************************/
                    HAL_Delay(200);                                     //给时间调整
                    close_PIDOUT += dis_target.PID_Out;
                } else {
                    close_flag = 0;                                       //结束第二阶段靠近任务
                    second_flag = 0;
                    tb_LED_start();
                    tb_BEEP_start();
                    second_reset_flag = 1;
                    HAL_Delay(5000);
                    tb_stop();
                }
            }

        }
    }
    if (second_reset_flag) {                                                //第二阶段复位到识别处并完成180倒置准备TOF模块识别
        CAR_longitudinal(-close_PIDOUT);
        HAL_Delay(5000);
        CAR_spin(-record_spin - (count_spin * oneTURN / 10));
        HAL_Delay(5000);
        CAR_spin(oneTURN / 2);      //180旋转准备TOF识别
        third_flag=1;
    }
}

void motor_thirdstage_control(void)
{
    if(third_flag){

    }

}





