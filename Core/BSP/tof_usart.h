//
// Created by ShiF on 2023/5/15.
//

#ifndef EE_CODE_TOF_USART_H
#define EE_CODE_TOF_USART_H

/* ATK-MS53L1M模块功能码 */
enum
{
    ATK_MS53L1M_FUNCODE_SYS             = 0x00, /* 系统设置 */
    ATK_MS53L1M_FUNCODE_BACKRATE        = 0x01, /* 回传速率设置 */
    ATK_MS53L1M_FUNCODE_BAUDRATE        = 0x02, /* 波特率设置 */
    ATK_MS53L1M_FUNCODE_IDSET           = 0x03, /* 设备地址设置 */
    ATK_MS53L1M_FUNCODE_MEAUDATA        = 0x05, /* 测量数据获取 */
    ATK_MS53L1M_FUNCODE_OUTPUTSTATUS    = 0x07, /* 测量状态 */
    ATK_MS53L1M_FUNCODE_MEAUMODE        = 0x08, /* 测量模式设置 */
    ATK_MS53L1M_FUNCODE_CALIMODE        = 0x09, /* 校准模式 */
    ATK_MS53L1M_FUNCODE_WORKMODE        = 0x0A, /* 工作模式 */
    ATK_MS53L1M_FUNCODE_TIMEBUDGET      = 0x0B, /* 定时预设 */
    ATK_MS53L1M_FUNCODE_TIMRPERIOD      = 0x0D, /* 测量间隔 */
    ATK_MS53L1M_FUNCODE_ERRORFRAM       = 0x0F, /* 错误帧信息 */
    ATK_MS53L1M_FUNCODE_VERSION         = 0x10, /* 版本信息 */
};

/* 系统设置参数 */
enum
{
    ATK_MS53L1M_SYS_PARAM_RESET         = 0x01, /* 系统恢复设置 */
    ATK_MS53L1M_SYS_RESET               = 0x02, /* 系统复位 */
};

/* 回传速率设置参数 */
enum
{
    ATK_MS53L1M_BACKRATE_01HZ           = 0x00, /* 0.1Hz */
    ATK_MS53L1M_BACKRATE_02HZ           = 0x01, /* 0.2Hz */
    ATK_MS53L1M_BACKRATE_05HZ           = 0x02, /* 0.5Hz */
    ATK_MS53L1M_BACKRATE_1HZ            = 0x03, /* 1Hz */
    ATK_MS53L1M_BACKRATE_2HZ            = 0x04, /* 2Hz */
    ATK_MS53L1M_BACKRATE_5HZ            = 0x05, /* 5Hz */
    ATK_MS53L1M_BACKRATE_10HZ           = 0x06, /* 10Hz */
    ATK_MS53L1M_BACKRATE_20HZ           = 0x07, /* 20Hz */
    ATK_MS53L1M_BACKRATE_50HZ           = 0x08, /* 50Hz */
    ATK_MS53L1M_BACKRATE_100HZ          = 0x09, /* 100Hz */
};

/* 串口波特率设置参数 */
enum
{
    ATK_MS53L1M_BAUDRATE_2400           = 0x00, /* 2400bps */
    ATK_MS53L1M_BAUDRATE_4800           = 0x01, /* 4800bps */
    ATK_MS53L1M_BAUDRATE_9600           = 0x02, /* 9600bps */
    ATK_MS53L1M_BAUDRATE_19200          = 0x03, /* 19200bps */
    ATK_MS53L1M_BAUDRATE_38400          = 0x04, /* 38400bps */
    ATK_MS53L1M_BAUDRATE_57600          = 0x05, /* 57600bps */
    ATK_MS53L1M_BAUDRATE_115200         = 0x06, /* 115200bps */
    ATK_MS53L1M_BAUDRATE_230400         = 0x07, /* 230400bps */
    ATK_MS53L1M_BAUDRATE_460800         = 0x08, /* 460800bps */
    ATK_MS53L1M_BAUDRATE_921600         = 0x09, /* 921600bps */
};

/* 测量模式设置参数 */
enum
{
    ATK_MS53L1M_MEAUMODE_SHORT          = 0x00, /* 短距离 */
    ATK_MS53L1M_MEAUMODE_MIDDLE         = 0x01, /* 中距离 */
    ATK_MS53L1M_MEAUMODE_LONG           = 0x02, /* 长距离 */
};

/* 工作模式设置参数 */
enum
{
    ATK_MS53L1M_WORKMODE_NORMAL         = 0x00, /* Normal模式 */
    ATK_MS53L1M_WORKMODE_MODBUS         = 0x01, /* Modbus模式 */
    ATK_MS53L1M_WORKMODE_IIC            = 0x02, /* IIC模式 */
};

/* 错误帧输出设置参数 */
enum
{
    ATK_MS53L1M_ERRORFRAM_OFF           = 0x00, /* 关闭 */
    ATK_MS53L1M_ERRORFRAM_ON            = 0x01, /* 开启 */
};

/* 错误代码 */
#define ATK_MS53L1M_EOK         0   /* 没有错误 */
#define ATK_MS53L1M_ERROR       1   /* 错误 */
#define ATK_MS53L1M_ETIMEOUT    2   /* 超时错误 */
#define ATK_MS53L1M_EFRAME      3   /* 帧错误 */
#define ATK_MS53L1M_ECRC        4   /* CRC校验错误 */
#define ATK_MS53L1M_EOPT        5   /* 操作错误 */

#define ATK_MS53L1M_MASTER_FRAME_HEAD   0x51    /* 主机请求帧头 */
#define ATK_MS53L1M_SLAVE_FRAME_HEAD    0x55    /* 从机应答帧头 */
#define ATK_MS53L1M_SENSOR_TYPE         0x0B    /* ATK-MS53L1M传感器类型 */

#define ATK_MS53L1M_FRAME_LEN_MAX       270     /* 接收帧最大长度 */
#define ATK_MS53L1M_FRAME_LEN_MIN       8       /* 接收帧最小长度 */

#define ATK_MS53L1M_OPT_READ            0x00    /* 读操作 */
#define ATK_MS53L1M_OPT_WRITE           0x01    /* 写操作 */


#endif //EE_CODE_TOF_USART_H
