//
// Created by ShiF on 2023/5/9.
//
#include "bsp_headfile.h"
#include "motor.h"
#include "math.h"

uint8_t first_flag = 0;
PID_t mv_measure;
PID_t tof_measure;
uint8_t spin_flag;
uint8_t second_flag;
uint32_t count_spin;                    //记录第二阶段第一部分的旋转次数
float record_spin;                      //记录自转PID调整时改变位姿
uint32_t count_pid;                     //pid次数限制，进入靠近阶段
#define count_pidLIM    10             //pid调整次数
uint8_t close_flag;                     //靠近状态标志
#define distance_target_th  15          //靠近距离目标值
PID_t dis_target;
uint32_t count_close;                   //靠近距离的PID次数
#define count_closeLIM  10             //靠近PID次数限制
float close_PIDOUT;
uint8_t second_reset_flag;            //第二阶段复位信号
uint8_t third_flag;                     //第三阶段开始信号（TOF识别矫正过程）
//#define count_closeLIM  200             ***********>>>>>>>>>进入motor.h头文件更改
uint8_t tof_flag;
#define count_tofpidLIM     10        //TOF测距过程中的PID次数
uint32_t count_tof;
#define motor_stop_s    (Speed_F<=1&&Speed_F>=-1&&Speed_BR<=1&&Speed_BR>=-1&&Speed_BL<=1&&Speed_BL>=-1)

void motor_Init(void) {
/*****校赛参数初始化**************/
    first_flag = 1;
    spin_flag = 0;
    second_flag = 0;
    count_pid = 0;
    close_flag = 0;
    count_spin = 0;
    count_close = 0;
    close_PIDOUT = 0.0;
    second_reset_flag = 0;
    third_flag = 0;
    tof_flag = 0;
    count_tof = 0;
/*****校赛**************/

/*****国赛初始化**************/

/*****国赛**************/

    PID_SpeedParamInit(&mv_measure);
    PID_SpeedParamInit(&tof_measure);
    PID_SpeedParamInit(&dis_target);
    mv_measure.PID_OutMax = 20;
    dis_target.PID_OutMax = 10;
    tof_measure.PID_OutMax = 10;
}

/****************************SPD********************************/
/*******线性代数公式集成********/
/*   车身逆时针自转为正   */
/*  Vx       -2/3        1/3          1/3            Vf    */
/*[ Vy ] = [   0      -(3^(1/2))/3  (3^(1/2)/3 ] * [ Vbl ] */
/*  W       1/(3*a)    1/(3*a)       1/(3*a)         Vbr   */
/*Vx > 0 =====>  right*/
/*Vy > 0 =====>  forward*/
/*W  > 0 =====>  counterclockwise*/
/*a为轮轴到机器中心的距离*/
/*开启速度环控制*/
/******************/
#define a (oneTURN/(M_PI*2))

void CAR_liner(float Vx, float Vy, float W) {
    SetSpd_F = a * W - Vx;
    SetSpd_BL = a * W + Vx / 2 - Vy * sqrt(3) / 2;
    SetSpd_BR = a * W + Vx / 2 + Vy * sqrt(3) / 2;
}

void CAR_SPD_spin(void) {
    SetSpd_F = 50;
    SetSpd_BL = 50;
    SetSpd_BR = 50;
}
/****************************SPD********************************/
/****************************POS********************************/
float update_Des(float set) {
    set = set * 1320 / 7 / PI;
    return set;
}//位置环参数转换

/*******自转********/
/*  三个轮子同向转动 */
/******************/
void CAR_spin(float des) {
    SetPos_F += des;
    SetPos_BR += des;
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

/*******转弯********/
/*后两轮为驱动的轮子*/
/*F轮作为方向导向*/
/*Des为正时向右转弯*/
/******************/
void CAR_dir(float des) {
    SetPos_BL += 5;
    SetPos_BR += 5;
    SetPos_F += des;
    MotorPosPID_F.PID_OutMax = 50;
    MotorPosPID_BL.PID_OutMax = 20;
    MotorPosPID_BR.PID_OutMax = 20;
}
/****************************POS********************************/
#define Dest_1  220
#define Dest_2  11
#define Dest_3  26
#define Dest_4  54
#define Dest_5  17
#define Dest_6  11
#define Dest_7  147
#define Dest_8  152
#define Dest_9  148

uint8_t run_fstSTG_flag = 0;
uint8_t run_fstSTG_count = 1;

//第一阶段
void motor_firststage_control(void) {
    if (first_flag == 1) {
        //stage1
        if (run_fstSTG_count == 1 && !run_fstSTG_flag) {
            CAR_longitudinal(Dest_1);
        }
//        HAL_Delay(4500);
            //stage2
        else if (run_fstSTG_count == 2 && !run_fstSTG_flag) {
            CAR_transverse(-Dest_2);
        }
//        HAL_Delay(1000);
            //stage3
        else if (run_fstSTG_count == 3 && !run_fstSTG_flag) {
            CAR_longitudinal(Dest_3);
        }
//        HAL_Delay(1300);
            //stage4
        else if (run_fstSTG_count == 4 && !run_fstSTG_flag) { CAR_transverse(-Dest_4); }
//        HAL_Delay(1500);
            //stage5
        else if (run_fstSTG_count == 5 && !run_fstSTG_flag) {
            CAR_longitudinal(-Dest_5);
        }
//        HAL_Delay(1200);
            //stage6
        else if (run_fstSTG_count == 6 && !run_fstSTG_flag) {
            CAR_transverse(-Dest_6);
        }
//        HAL_Delay(1000);
            //stage7
        else if (run_fstSTG_count == 7 && !run_fstSTG_flag) {
            CAR_longitudinal(-Dest_7);
        }
//        HAL_Delay(3000);
            //stage8
        else if (run_fstSTG_count == 8 && !run_fstSTG_flag) {
            CAR_transverse(-Dest_8);
        }
//        HAL_Delay(3000);
            //stage9
        else if (run_fstSTG_count == 9 && !run_fstSTG_flag) {
            CAR_longitudinal(Dest_9);
        }
//        HAL_Delay(3000);
        else if (motor_stop_s && run_fstSTG_flag) {
            run_fstSTG_count++;
            if (run_fstSTG_count == 10) {
                second_flag = 1;
                first_flag = 0;
            }
        }
    }

}

//第二阶段
uint8_t run_closePID_flag = 0;
uint8_t run_disPID_flag = 0;
uint8_t run_secEND_flag = 0;
uint8_t run_secRST_count = 0;
uint8_t run_secRST_flag = 0;

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
                if (!run_closePID_flag) {
                    mv_measure.Kp1 = 0.005;
                    mv_measure.Ki1 = 0.0;
                    mv_measure.Kd1 = 0.0;
                    mv_measure.PID_Target = 0.0;

                    PID_Update(&mv_measure, error_angle);
                    PID_GetPositionPID(&mv_measure);
                    mv_measure.PID_OutMax = 5;
//                    HAL_Delay(50);
                    /*************待更改标记****************/
                    CAR_spin(mv_measure.PID_Out);               //回传数据还没做更改，PID_OUT输出为正是右转，输出为负是左转
                    /*************************************/
                    record_spin += mv_measure.PID_Out;
                    count_pid++;
                    run_closePID_flag = 1;
                } else {
                    if (motor_stop_s) {
                        run_closePID_flag = 0;
                    }
                }
            }                            //在一定PID调整次数内调整位姿后跳出PID
            else if (count_pid > count_pidLIM && !close_flag) {
                close_flag = 1;                                 //置靠近状态标志
                HAL_Delay(300);                           //等待对焦PID调整结束
            } else if (close_flag) {
                if (count_close <= count_closeLIM) {
                    if (!run_disPID_flag) {
                        dis_target.Kp1 = 0.1;
                        dis_target.Ki1 = 0.0;
                        dis_target.Kd1 = 0.0;
                        dis_target.PID_Target = distance_target_th;
                        PID_Update(&dis_target, distance_target);
                        PID_GetPositionPID(&dis_target);

                        /*************待更改标记****************/
                        CAR_longitudinal(-dis_target.PID_Out);               //靠近参数没做好
                        /*************************************/
//                        HAL_Delay(50);                                     //给时间调整
                        close_PIDOUT += dis_target.PID_Out;
                        run_disPID_flag = 1;
                    } else {
                        if (motor_stop_s) {
                            run_disPID_flag = 0;
                        }
                    }
                }
                if (count_close >= count_closeLIM || distance_target <= distance_target_th) {
                    if (!run_secEND_flag && motor_stop_s) {
                        CAR_longitudinal(20);
                        run_secEND_flag = 1;
                    } else {
                        if (motor_stop_s) {
                            tb_LED_start();
                            tb_BEEP_start();
                            second_reset_flag = 1;
                            HAL_Delay(5000);
                            tb_stop();
                            close_flag = 0;                                       //结束第二阶段靠近任务
                            second_flag = 0;
                        }
                    }
//                    HAL_Delay(3000);

                }

            }

        }
    }
    if (second_reset_flag == 1) {                                                //第二阶段复位到识别处并完成180倒置准备TOF模块识别
        if (!run_secRST_flag) {
            if (run_secRST_count == 0 && motor_stop_s) {
                CAR_longitudinal(-20);
                CAR_longitudinal(close_PIDOUT + 20);
                run_secRST_count++;
                run_secRST_flag = 1;
            } else if (run_secRST_count == 1 && motor_stop_s) {
                CAR_spin(-record_spin - (count_spin * oneTURN / 10));
                CAR_spin(oneTURN / 2);      //180旋转准备TOF识别
                run_secRST_flag = 1;
            }
        } else if (motor_stop_s && (run_secRST_flag == 1)) {
            run_secRST_flag = 0;
        }
//        HAL_Delay(2000);
//
//        HAL_Delay(3000);
//
//        HAL_Delay(5000);
        if (run_secRST_count == 2 && motor_stop_s) {
            third_flag = 1;
            tof_flag = 1;
            second_reset_flag = 0;
        }
    }
//    usart_printf("%d,%d,%d,%d,%.2f,%.2f\r\n", error_angle, distance_target, spin_flag, stop_flag, record_spin,
//                 close_PIDOUT);
}

#define BACK_Dest_1 158
#define BACK_Dest_2 60
#define BACK_Dest_3 100
#define BACK_Dest_4 80
#define BACK_FOR_TOF    148
uint8_t tof_runstop_flag = 1;
uint8_t run_backtof_flag = 0;
uint8_t run_tofPID_flag = 0;

void motor_thirdstage_control(void) {
    if (third_flag) {
        if (tof_flag) {                             //第三阶段开始获取tof_data比较
            if (tof_runstop_flag) {
                if (motor_stop_s && !run_backtof_flag) {
                    CAR_longitudinal(BACK_FOR_TOF);
                    run_backtof_flag = 1;
                } else if (motor_stop_s && run_backtof_flag) {
//                HAL_Delay(3000);
                    tof_runstop_flag = 0;
                }
            } else if (count_tof <= count_tofpidLIM && !tof_runstop_flag) {
                if (motor_stop_s && !run_tofPID_flag) {
                    tof_measure.Kp1 = 0.01;
                    tof_measure.Ki1 = 0.0;
                    tof_measure.Kd1 = 0.0;
                    tof_measure.PID_Target = tof_distance_th;

                    PID_Update(&tof_measure, (float) tof_data);
                    PID_GetPositionPID(&tof_measure);
//                if (count_tof <= 25) {
//                    HAL_Delay(200);
//                } else if (count_tof >= 25) {
//                    HAL_Delay(50);
//                }
                    /*****************tof-pid靠近输出为负*******************/
                    CAR_transverse(tof_measure.PID_Out);
                    count_tof++;
                    run_tofPID_flag = 1;
                } else if (motor_stop_s && run_tofPID_flag)
                    run_tofPID_flag = 0;
            } else tof_flag = 0;
        } else {
            HAL_Delay(100);
            CAR_longitudinal(-BACK_Dest_1);
//            HAL_Delay(5000);
//            CAR_longitudinal(-BACK_Dest_2);
            HAL_Delay(5000);
//            CAR_transverse(-BACK_Dest_3);
//            HAL_Delay(5000);
//            CAR_longitudinal(BACK_Dest_4);
            third_flag = 0;
        }
    }
}

void motor_spin_test(void) {

    //pid对焦
    mv_measure.Kp1 = Debug_Param().vel_kp;
    mv_measure.Ki1 = Debug_Param().vel_ki;
    mv_measure.Kd1 = Debug_Param().vel_kd;
    mv_measure.PID_Target = 0.0;

    PID_Update(&mv_measure, error_angle);
    PID_GetPositionPID(&mv_measure);

    /*************待更改标记****************/
    CAR_spin(mv_measure.PID_Out);               //回传数据还没做更改，PID_OUT输出为正是右转，输出为负是左转
    /*************************************/
    record_spin += mv_measure.PID_Out;
    HAL_Delay(500);
//    usart_printf("%.2f,%.2f,%.2f,%d\r\n", record_spin, mv_measure.PID_Out, mv_measure.PID_Target, error_angle);

}

void motor_close_test(void) {
    dis_target.Kp1 = Debug_Param().vel_kp;
    dis_target.Ki1 = Debug_Param().vel_ki;
    dis_target.Kd1 = Debug_Param().vel_kd;
    dis_target.PID_Target = distance_target_th;
    PID_Update(&dis_target, distance_target);
    PID_GetPositionPID(&dis_target);

    /*************待更改标记****************/
    CAR_longitudinal(-dis_target.PID_Out);               //靠近参数没做好
    /*************************************/
    close_PIDOUT += dis_target.PID_Out;
    HAL_Delay(200);                                     //给时间调整
//    usart_printf("%.2f,%.2f,%.2f,%d\r\n", close_PIDOUT, dis_target.PID_Out, dis_target.PID_Target, distance_target);
}

void motor_tof_test(void) {

    tof_measure.Kp1 = 0.01;
    tof_measure.Ki1 = 0.0;
    tof_measure.Kd1 = 0.0;
    tof_measure.PID_Target = tof_distance_th;
    PID_Update(&tof_measure, (float) tof_data);
    PID_GetPositionPID(&tof_measure);

    /*****************tof-pid靠近输出为负*******************/
    CAR_transverse(tof_measure.PID_Out);
    count_tof++;
    HAL_Delay(50);

//    usart_printf("%d,%.2f,%.2f,%d,%.2f\r\n", tof_data, tof_measure.PID_Out, tof_measure.PID_Target, count_tof,
//                 tof_measure.PID_OutMax);

}


void motor_detectflag(void) {
    uint8_t flag = stop_flag;
    if (!flag) {
        SetSpd_BL = 20;
        SetSpd_BR = 20;
    } else {
        SetSpd_BL = 0;
        SetSpd_BR = 0;
    }

}
