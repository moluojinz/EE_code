//
// Created by ShiF on 2023/5/9.
//

#ifndef EE_CODE_MOTOR_H
#define EE_CODE_MOTOR_H

#define COS_30  0.866
#define COS_60  0.500

float   update_Des(float set);
void    CAR_spin(float des);
void    CAR_longitudinal(float des);
void    CAR_transverse(float des);



#endif //EE_CODE_MOTOR_H
