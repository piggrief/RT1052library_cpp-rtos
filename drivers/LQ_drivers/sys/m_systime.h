
// Header:
// File Name: 
// Author: Z
// Date:2018/08/03

#ifndef __M_SYSTIME_H
#define __M_SYSTIME_H

#include "include.h"

typedef struct
{
		
	void (* init) (void);  
	uint64_t (* get_time_us) (void);
	uint32_t (* get_time_ms) (void);
	void (* delay_us)(uint64_t);//��ʱ1us ʵ��Ϊ1.23us   ��ʱ10us ʵ��Ϊ10.23us   ��ʱ100us ʵ��Ϊ100.23us
	void (* delay_ms)(uint32_t);

}systime_t;

extern systime_t  _systime;
/*!λ�ã�m_systime.h
* @brief ������������ʱ�䣬us����
*
* @param *RunFuction ����������ʱ��ĺ���ָ�룬Ҫ����һ��void���룬void����ĺ���
* @example uint64_t time = MeasureRunTime_us(DelayTest);
*/
uint64_t MeasureRunTime_us(void(*RunFuction)(void));
/*!λ�ã�m_systime.h
* @brief ������������ʱ�䣬ms����
*
* @param *RunFuction ����������ʱ��ĺ���ָ�룬Ҫ����һ��void���룬void����ĺ���
* @example uint64_t time = MeasureRunTime_ms(DelayTest);
*/
uint32_t MeasureRunTime_ms(void(*RunFuction)(void));

void systime_init();
#endif //__M_SYSTIME_H




