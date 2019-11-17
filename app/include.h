#ifndef _PIG_INCLUDE_H_
#define _PIG_INCLUDE_H_
    #include "fsl_include.h"
//������η�װ������
    #include "LQ_IIC.h"
    #include "LQ_GPIOmacro.h" //GPIO��51������ʽ 
    #include "LQ_SGP18T.h"    //TFT1.8������ģ��
    #include "LQ_LED.h"       //LEDָʾ
    #include "LQ_KEY.h"       //��������
    #include "LQ_PWM.h"       //��������PWM���� XBARA
    #include "LQ_UART.h"       //UART����
    #include "LQ_ADC.h"       //ADCת���ɼ�
    #include "LQ_PIT.h"       //PIT��ʱ
    #include "LQ_TRNG.h"      //�����������
    #include "LQ_Encoder.h"   //�����������������ݲɼ�
    #include "m_systime.h"    //systick ��ʱ�������ڼ�ʱ
    #include "status.h"       //��־λ
    #include "LQ_QTMR.h"      //��ʱ������������PWM  �� ��������
    #include "LQ_SPI.h"
    #include "LQ_MPU6050.h"

//Pig���η�װ�⼰������
    #include "Pig_SPI.h"
    #include "TFTDriver.h"
//Pig������
    # include "DebugFunction.h"

#define delayms(x) _systime.delay_ms(x)
#define delayus(x) _systime.delay_us(x)
#define uint16 uint16_t
#define int16 int16_t
#define uint8 uint8_t
#define int8 int8_t

#endif