#ifndef _PIG_INCLUDE_H_
#define _PIG_INCLUDE_H_
    #include "fsl_include.h"
//龙邱二次封装驱动库
    #include "LQ_IIC.h"
    #include "LQ_GPIOmacro.h" //GPIO口51操作格式 
    #include "LQ_SGP18T.h"    //TFT1.8吋彩屏模块
    #include "LQ_LED.h"       //LED指示
    #include "LQ_KEY.h"       //独立按键
    #include "LQ_PWM.h"       //电机，舵机PWM控制 XBARA
    #include "LQ_UART.h"       //UART串口
    #include "LQ_ADC.h"       //ADC转换采集
    #include "LQ_PIT.h"       //PIT定时
    #include "LQ_TRNG.h"      //随机数发生器
    #include "LQ_Encoder.h"   //编码器正交解码数据采集
    #include "m_systime.h"    //systick 定时器，用于计时
    #include "status.h"       //标志位
    #include "LQ_QTMR.h"      //计时器可用于生成PWM  和 正交解码
    #include "LQ_SPI.h"
    #include "LQ_MPU6050.h"

//Pig二次封装库及驱动库
    #include "Pig_SPI.h"
    #include "TFTDriver.h"
//Pig函数库
    # include "DebugFunction.h"

#define delayms(x) _systime.delay_ms(x)
#define delayus(x) _systime.delay_us(x)
#define uint16 uint16_t
#define int16 int16_t
#define uint8 uint8_t
#define int8 int8_t

#endif