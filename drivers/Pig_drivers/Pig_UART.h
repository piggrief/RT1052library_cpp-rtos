/*!
* @file       Pig_UART.c
* @brief      根据fsl官方库进行C++的二次封装库（UART外设）
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
    * @brief 通过串口发送字符
    * @param data   字符数据
    */
    void PutChar(uint8_t data);
    /*!
    * @brief 通过串口发送字符串
    * @param buff    字符串
    * @param length  字符串长度
    * @param IfUseDMA 是否使用DMA传输
    */
    void PutBuff(uint8_t * buff, uint32_t length, bool IfUseDMA);

private:
    /*该UART所使用的外设名称*/
    LPUART_Type * UART;
    /*该UART所使用的外设索引，一般0代表LPUART1*/
    int LPUARTIndex;
    /*该UART外设使用的运行模式，中断或DMA或不启动*/
    UARTUsedMode_t NowMode;
    /*串口中断优先级配置字典*/
    static int InterruptPriorityList[8];
    /* 获得UART系统时钟频率 */
    uint32_t GetUartSrcFreq(void);
    /*!
    * @brief 初始化UART外设管脚
    * @param base    UART外设名称，如LPUART1~LPUART8
    * @param RX      RX管脚
    * @param TX      TX管脚
    */
    void PortInit(PORT_t RX, PORT_t TX);
    /*!
    * @brief 初始化UART外设管脚
    * @param base    UART外设名称，如LPUART1~LPUART8
    * @param Bound   波特率
    */
    void FunctionInit(uint32_t Bound);
    /*!
    * @brief 初始化UART DMA收发功能
    * @param base    UART外设名称，如LPUART1~LPUART8
    * @param DMAChannel_RX   RX映射的DMA通道
    * @param DMAChannel_TX   TX映射的DMA通道
    */
    void DMAFunctionInit(int DMAChannel_RX, int DMAChannel_TX);
};


void UART_Put_Buff_DMA(uint8_t *dataToSend, uint8_t length);