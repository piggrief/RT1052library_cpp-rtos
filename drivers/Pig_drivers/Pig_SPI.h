/*!
* @file       Pig_SPI.h
* @brief      ���ݹٷ���fsl_lpspi���һЩ����Ľӿ�
* @details    ����SPI�����š�ʱ����Ҫ�ùٷ����ù���������
* @author     pig's grief
* @version    v1.0
* @date       2019-4-10
* @to do      
*/
# ifndef _PIG_SPI_H_
# define _PIG_SPI_H_

# include "include.h"

extern const lpspi_master_config_t DefaultSPI_config;
extern const lpspi_master_config_t TFTSPI_config;
extern lpspi_master_config_t SPI1_config;
extern lpspi_master_config_t SPI2_config;
extern lpspi_master_config_t SPI3_config;
extern lpspi_master_config_t SPI4_config;


/*!
* @brief SPI��ʼ��
*
* @param SPIn SPIͨ����LPSPI1��LPSPI2��
* @param srcClock_Hz SPIʱ��Ƶ�ʣ�һ����peripheral.h�ж���
* @param IfDefault ѡ���Ƿ�ʹ��Ĭ��SPI���ã�Ϊ0���ֱ��ʹ��������ã���Ϊ0����Ƚ�������ñ��Ĭ������
*/
void SPI_Init(LPSPI_Type * SPIn, uint32_t srcClock_Hz, int IfDefault);

/*!
* @brief SPI��������
*
* @param SPIn SPIͨ����LPSPI1��LPSPI2��
* @param *cmdbuff ���������д�뻺������
* @param *databuff ���ݻ���������ȡ��������
* @param dSize �������ݸ�����1����2���ȣ�
*/
void spi_mosi(LPSPI_Type * SPIn, uint8 * cmdbuff, uint8 * databuff, int dSize);


# endif