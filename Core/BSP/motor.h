//
// Created by ShiF on 2023/5/9.
//

#ifndef EE_CODE_MOTOR_H
#define EE_CODE_MOTOR_H

#define COS_30  0.866
#define COS_60  0.500
#define PI      3.1415
#define oneTURN 62.44

void    motor_Init(void);
float   update_Des(float set);
void    CAR_spin(float des);
void    CAR_longitudinal(float des);
void    CAR_transverse(float des);
void    CAR_dir(float des);
void    motor_firststage_control(void);
void    motor_secondstage_control(void);
void    motor_thirdstage_control(void);
void    motor_spin_test(void);
void    motor_close_test(void);
void    motor_tof_test(void);
void    motor_detectflag(void);
void    motor_test_control(void);

void    CAR_liner(float Vx, float Vy, float W);//速度环，线性集成
void    CAR_SPD_spin(void);

extern PID_t mv_measure;
extern PID_t tof_measure;
extern uint8_t spin_flag;
extern uint8_t second_flag;
extern PID_t dis_target;

#define tof_distance_th     (10*35)     //(10*X)    X为距离，单位为cm



#endif //EE_CODE_MOTOR_H
