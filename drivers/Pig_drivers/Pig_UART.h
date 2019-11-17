/*!
* @file       Pig_UART.c
* @brief      ����fsl�ٷ������C++�Ķ��η�װ�⣨UART���裩
* @details
* @author     pig's grief
* @version    v1.0
* @date       2019-11-03
* @to do
*/
# pragma once
# include "fsl_include.h"
# include "PORT_CFG.h"

typedef enum
{
    None,
    Interrupt,
    DMA
}UARTUsedMode_t;

typedef enum
{
    DMASendFinish,
    DMASending

}UARTDMAStatus_t;

class Pig_UART
{
public:

    //Pig_UART();
    Pig_UART(LPUART_Type *base, uint32_t Bound, PORT_t RX, PORT_t TX, 
        UARTUsedMode_t ModeSet = Interrupt, int DMAChannel_RX = 0, int DMAChannel_TX = 1);
    ~Pig_UART();

    /*!
    * @brief ͨ�����ڷ����ַ�
    * @param data   �ַ�����
    */
    void PutChar(uint8_t data);
    /*!
    * @brief ͨ�����ڷ����ַ���
    * @param buff    �ַ���
    * @param length  �ַ�������
    * @param IfUseDMA �Ƿ�ʹ��DMA����
    */
    void PutBuff(uint8_t * buff, uint32_t length, bool IfUseDMA);

private:
    /*��UART��ʹ�õ���������*/
    LPUART_Type * UART;
    /*��UART��ʹ�õ�����������һ��0����LPUART1*/
    int LPUARTIndex;
    /*��UART����ʹ�õ�����ģʽ���жϻ�DMA������*/
    UARTUsedMode_t NowMode;
    /*�����ж����ȼ������ֵ�*/
    static int InterruptPriorityList[8];
    /* ���UARTϵͳʱ��Ƶ�� */
    uint32_t GetUartSrcFreq(void);
    /*!
    * @brief ��ʼ��UART����ܽ�
    * @param base    UART�������ƣ���LPUART1~LPUART8
    * @param RX      RX�ܽ�
    * @param TX      TX�ܽ�
    */
    void PortInit(PORT_t RX, PORT_t TX);
    /*!
    * @brief ��ʼ��UART����ܽ�
    * @param base    UART�������ƣ���LPUART1~LPUART8
    * @param Bound   ������
    */
    void FunctionInit(uint32_t Bound);
    /*!
    * @brief ��ʼ��UART DMA�շ�����
    * @param base    UART�������ƣ���LPUART1~LPUART8
    * @param DMAChannel_RX   RXӳ���DMAͨ��
    * @param DMAChannel_TX   TXӳ���DMAͨ��
    */
    void DMAFunctionInit(int DMAChannel_RX, int DMAChannel_TX);
};


void UART_Put_Buff_DMA(uint8_t *dataToSend, uint8_t length);