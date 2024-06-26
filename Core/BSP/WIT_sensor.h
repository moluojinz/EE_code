//
// Created by ShiF on 2023/7/13.
//

#ifndef EE_CODE_WIT_SENSOR_H
#define EE_CODE_WIT_SENSOR_H

#include "stm32f1xx_hal.h"

#define RXBUFFER_LEN 99		//接收9类数据，一共99位

#define SAVE 			0x00
#define CALSW 		0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 			0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD		0x1c

#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
#define q0          0x51
#define q1          0x52
#define q2          0x53
#define q3          0x54

#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5

struct STime
{
    unsigned char ucYear;
    unsigned char ucMonth;
    unsigned char ucDay;
    unsigned char ucHour;
    unsigned char ucMinute;
    unsigned char ucSecond;
    unsigned short usMiliSecond;
};
struct SAcc//加速度
{
    short a[3];
    short T;
};
struct SGyro//角速度
{
    short w[3];
    short T;
};
struct SAngle//角度
{
    short Angle[3];
    short T;
};
struct SMag//磁场输出
{
    short h[3];
    short T;
};

struct SDStatus//端口状态数据输出
{
    short sDStatus[4];
};

struct SPress//气压高度
{
    long lPressure;
    long lAltitude;
};

struct SLonLat//经纬度
{
    long lLon;
    long lLat;
};

struct SGPSV
{
    short sGPSHeight;
    short sGPSYaw;
    long lGPSVelocity;
};
struct SQ //四元数
{ short q[4];
};
/*****************************************/
typedef struct
{
    float angle[3];
}Angle;

typedef struct
{
    float a[3];
}Acc;

typedef struct
{
    float w[3];
}SGyro;


typedef struct//四元数
{ float q[4];
}SQ;

typedef struct//磁场输出
{
    float h[3];
}SMag;

typedef struct//气压高度
{
    float lPressure;
    float lAltitude;
}SPress;

typedef struct//经纬度
{
    float lLon;
    float lLat;
}SLonLat;

typedef struct WIT_sensor_USART
{
    uint8_t Rx_flag;
    uint8_t Rx_len;
    uint8_t frame_head;					//帧头
    uint8_t RxBuffer[RXBUFFER_LEN];		//接收缓冲
    Angle angle;						//角度
    Acc acc;								//加速度
    SGyro w;								//角速度
    SMag h;									//磁场
    SPress lPressure;   	  //气压
    SPress lAltitude;     	//高度
    SLonLat lLon;						//经度
    SLonLat lLat;						//维度
    SQ q; 									//四元数
}WIT_sensor_USART;

extern WIT_sensor_USART WT9011G4K_data;
extern struct SAcc 		stcAcc;
extern struct SGyro 		stcGyro;
extern struct SAngle 	stcAngle;
extern struct SMag 	stcMag;
extern struct SPress 	stcPress;
extern struct SLonLat 	stcLonLat;
extern struct SQ stcQ;

void WIT_sensor_USART_Init(WIT_sensor_USART *Data);
void WIT_sensor_receive(void);
#endif //EE_CODE_WIT_SENSOR_H
