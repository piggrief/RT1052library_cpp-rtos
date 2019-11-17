/*!
* @file       Pig_UART.c
* @brief      根据fsl官方库进行C++的二次封装库（UART外设）
* @details
* @author     pig's grief
* @version    v1.0
* @date       2019-11-03
* @to do
*/
# include "Pig_UART.h"

using namespace std;
extern lpuart_transfer_t xfer;
extern lpuart_transfer_t sendXfer;
extern lpuart_edma_handle_t g_lpuartEdmaHandle;
extern edma_handle_t g_lpuartTxEdmaHandle;
extern edma_handle_t g_lpuartRxEdmaHandle;

UARTDMAStatus_t UARTDMAStatusMap[8] = 
{ DMASendFinish, DMASendFinish, DMASendFinish, DMASendFinish,
  DMASendFinish, DMASendFinish, DMASendFinish, DMASendFinish };

//void UART_Put_Buff_DMA(uint8_t *dataToSend, uint8_t length)
//{
//    /* If TX is idle and g_txBuffer is full, start to send data. */
//    /*使用DMA + 串口，无需占用CPU时间 */
//    sendXfer.data = dataToSend;
//    sendXfer.dataSize = length;
//    if (UARTDMAStatusMap[0] == DMASendFinish)
//    {
//        UARTDMAStatusMap[0] = DMASending;
//        LPUART_SendEDMA(LPUART1, &g_lpuartEdmaHandle, &sendXfer);
//    }
//
//}

void LPUART1_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (UARTDMAStatusMap[0] == DMASending)
    {
        UARTDMAStatusMap[0] = DMASendFinish;
    }
}

/* 获得UART系统时钟频率 */
uint32_t Pig_UART::GetUartSrcFreq(void)
{
    uint32_t freq;

    /* To make it simple, we assume default PLL and divider settings, and the only variable
    from application is use PLL3 source or OSC source */
    if (CLOCK_GetMux(kCLOCK_UartMux) == 0) /* PLL3 div6 80M */
    {
        freq = (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }
    else
    {
        freq = CLOCK_GetOscFreq() / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }

    return freq;
}

Pig_UART::Pig_UART(LPUART_Type *base, uint32_t Bound, PORT_t RX, PORT_t TX, 
    UARTUsedMode_t ModeSet, int DMAChannel_RX, int DMAChannel_TX)
{
    NowMode = ModeSet;
    UART = base;
    PortInit(RX, TX);
    FunctionInit(Bound);

    if (ModeSet == DMA)
        DMAFunctionInit(DMAChannel_RX, DMAChannel_TX);

    if (base == LPUART1)
        LPUARTIndex = 0;
    else if (base == LPUART2)
        LPUARTIndex = 1;
    else if (base == LPUART3)
        LPUARTIndex = 2;
    else if (base == LPUART4)
        LPUARTIndex = 3;
    else if (base == LPUART5)
        LPUARTIndex = 4;
    else if (base == LPUART6)
        LPUARTIndex = 5;
    else if (base == LPUART7)
        LPUARTIndex = 6;
    else if (base == LPUART8)
        LPUARTIndex = 7;
}

void Pig_UART::PortInit(PORT_t RX, PORT_t TX)
{
    if (UART == LPUART1)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart1);	//使能LPUART1时钟

        assert(RX == PTL14 && TX == PTK14);

        //LPUART1所使用的IO功能配置，即：从ALT0~ALT7选择合适的功能。
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0U);	//LPUART1_TX   K14
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0U);	//LPUART1_RX   L14

        //配置IO引脚GPIO_AD_B0_12和GPIO_AD_B0_13的功能
        //低转换速度,驱动能力为R0/6,速度为100Mhz，关闭开路功能，使能pull/keepr
        //选择keeper功能，下拉100K Ohm，关闭Hyst
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0x10B0u);
            
    }
    else if (UART == LPUART2)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart2);	//使能LPUART1时钟

        assert(RX == PTM12 && TX == PTL11);

        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_LPUART2_TX, 0U);	//LPUART2_TX    L11
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_LPUART2_RX, 0U);	//LPUART2_RX    M12

        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_02_LPUART2_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_03_LPUART2_RX, 0x10B0u);
    }
    else if (UART == LPUART3)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart3);	//使能LPUART1时钟
        
        assert((RX == PTC9 || RX == PTK10) && (TX == PTB9 || TX == PTJ12));

        if (TX == PTJ12)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_LPUART3_TX, 0U);	//LPUART3_TX     J12
            IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_06_LPUART3_TX, 0x10B0u);
        }
        else if (TX == PTB9)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_B0_08_LPUART3_TX, 0U);	//LPUART3_TX     J12
            IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_08_LPUART3_TX, 0x10B0u);
        }
        if (RX == PTC9)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_B0_09_LPUART3_RX, 0U);	//LPUART3_RX     C9
            IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_09_LPUART3_RX, 0x10B0u);
        }
        else if (RX == PTK10)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_LPUART3_RX, 0U);	//LPUART3_TX     K10
            IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_07_LPUART3_RX, 0x10B0u);
        }
    }
    else if (UART == LPUART4)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart4);	//使能LPUART1时钟

        assert((RX == PTM5 || RX == PTB11) && (TX == PTA11 || TX == PTL5));

        if (TX == PTA11)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_B1_00_LPUART4_TX, 0U);	//LPUART3_TX     J12
            IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_00_LPUART4_TX, 0x10B0u);
        }
        else if (TX == PTL5)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_00_LPUART4_TX, 0U);	//LPUART3_TX     J12
            IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_00_LPUART4_TX, 0x10B0u);
        }
        if (RX == PTM5)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_01_LPUART4_RX, 0U);	//LPUART3_RX     C9
            IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_01_LPUART4_RX, 0x10B0u);
        }
        else if (RX == PTB11)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_B1_01_LPUART4_RX, 0U);	//LPUART3_TX     K10
            IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_01_LPUART4_RX, 0x10B0u);
        }
    }
    else if (UART == LPUART5)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart5);	//使能LPUART1时钟

        assert(RX == PTD14 && TX == PTD13);

        IOMUXC_SetPinMux(IOMUXC_GPIO_B1_12_LPUART5_TX, 0U);	//LPUART5_TX       D13
        IOMUXC_SetPinMux(IOMUXC_GPIO_B1_13_LPUART5_RX, 0U);	//LPUART5_RX       D14

        IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_12_LPUART5_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_13_LPUART5_RX, 0x10B0u);
    }
    else if (UART == LPUART6)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart6);	//使能LPUART1时钟

        assert(RX == PTG11 && TX == PTM11);

        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_LPUART6_TX, 0U);	//LPUART6_TX   M11
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_LPUART6_RX, 0U);	//LPUART6_RX   J11

        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_LPUART6_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_LPUART6_RX, 0x10B0u);
    }
    else if (UART == LPUART7)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart7);	//使能LPUART1时钟

        assert(RX == PTN4 && TX == PTP3);

        IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_08_LPUART7_TX, 0U);	//LPUART7_TX   P3
        IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_09_LPUART7_RX, 0U);	//LPUART7_RX   N4

        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_08_LPUART7_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_09_LPUART7_RX, 0x10B0u);
    }
    else if (UART == LPUART8)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart8);	//使能LPUART1时钟

        assert((RX == PTJ2 || RX == PTB7 || RX == PTJ13) &&
            (TX == PTH2 || TX == PTL13));

        if (TX == PTH2)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_04_LPUART8_TX, 0U);	//LPUART3_TX     J12
            IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_LPUART8_TX, 0x10B0u);
        }
        else if (TX == PTL13)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_LPUART8_TX, 0U);	//LPUART3_TX     J12
            IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_10_LPUART8_TX, 0x10B0u);
        }
        if (RX == PTJ2)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_05_LPUART8_RX, 0U);	//LPUART3_RX     C9
            IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_LPUART8_RX, 0x10B0u);
        }
        else if (RX == PTB7)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_39_LPUART8_RX, 0U);	//LPUART3_TX     K10
            IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_39_LPUART8_RX, 0x10B0u);
        }
        else if (RX == PTJ13)
        {
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_LPUART8_RX, 0U);	//LPUART3_TX     K10
            IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_11_LPUART8_RX, 0x10B0u);
        }
    }
    else
        assert(0);//UART外设名称错误！！！
}

void Pig_UART::FunctionInit(uint32_t Bound)
{
    CLOCK_SetMux(kCLOCK_UartMux, 0); 	//设置UART时钟源为PLL3 80Mhz，PLL3/6=480/6=80MHz
    CLOCK_SetDiv(kCLOCK_UartDiv, 0); 	//设置UART时钟1分频，即UART时钟为80Mhz

    lpuart_config_t lpuart_config;              //串口配置结构体
    LPUART_GetDefaultConfig(&lpuart_config);    //得到默认配置，后面在根据实际情况配置

    lpuart_config.baudRate_Bps = Bound;						//波特率
    lpuart_config.dataBitsCount = kLPUART_EightDataBits;		//8位
    lpuart_config.stopBitCount = kLPUART_OneStopBit;			//1位停止位
    lpuart_config.parityMode = kLPUART_ParityDisabled;		//无奇偶校验
    lpuart_config.enableRx = true;							//使能接收
    lpuart_config.enableTx = true;							//使能发送

    uint32_t  freq = 0;							//串口的时钟源频率
    freq = GetUartSrcFreq();	                //得到串口时钟频率

    LPUART_Init(UART, &lpuart_config, freq);				//初始化LPUART1 

    //LPUART中断设置
    LPUART_EnableInterrupts(UART, kLPUART_RxDataRegFullInterruptEnable); //kLPUART_RxDataRegFullInterruptEnable：使能接收中断  使用其他中断请自行修改

    if (UART == LPUART1)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[0]));
        EnableIRQ(LPUART1_IRQn);	                            //使能LPUART1中断      
    }
    else if (UART == LPUART2)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[1]));
        EnableIRQ(LPUART2_IRQn);	                            //使能LPUART1中断      
    }
    else if (UART == LPUART3)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[2]));
        EnableIRQ(LPUART3_IRQn);	                            //使能LPUART1中断    
    }
    else if (UART == LPUART4)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[3]));
        EnableIRQ(LPUART4_IRQn);	                            //使能LPUART1中断     
    }
    else if (UART == LPUART5)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[4]));
        EnableIRQ(LPUART5_IRQn);	                            //使能LPUART1中断     
    }
    else if (UART == LPUART6)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[5]));
        EnableIRQ(LPUART6_IRQn);	                            //使能LPUART1中断    
    }
    else if (UART == LPUART7)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[6]));
        EnableIRQ(LPUART7_IRQn);	                            //使能LPUART1中断       
    }
    else if (UART == LPUART8)
    {
        //优先级配置 抢占优先级1  子优先级2   越小优先级越高  抢占优先级可打断别的中断
        NVIC_SetPriority(LPUART8_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[7]));
        EnableIRQ(LPUART8_IRQn);	                            //使能LPUART1中断       
    }
}

void Pig_UART::DMAFunctionInit(int DMAChannel_RX, int DMAChannel_TX)
{
    /* Init DMAMUX */
    DMAMUX_Init(DMAMUX);              //初始化DMA多路复用器
    /* Set channel for LPUART */
    DMAMUX_SetSource(DMAMUX, DMAChannel_RX, kDmaRequestMuxLPUART1Tx);   //设置DMA通道0 映射到 LPUART1Tx
    DMAMUX_SetSource(DMAMUX, DMAChannel_TX, kDmaRequestMuxLPUART1Rx);   //设置DMA通道1 映射到 LPUART1Rx
    DMAMUX_EnableChannel(DMAMUX, DMAChannel_RX);                        //使能通道0
    DMAMUX_EnableChannel(DMAMUX, DMAChannel_TX);                        //使能通道1

    edma_config_t config;     //DMA配置结构体
    /* Init the EDMA module */
    EDMA_GetDefaultConfig(&config); //得到默认DMA配置
    EDMA_Init(DMA0, &config);       //初始化配置DMA


    EDMA_CreateHandle(&g_lpuartRxEdmaHandle, DMA0, DMAChannel_RX);  
    // 创建DMA句柄，将DMA0， 通道DMAChannel_RX， 等信息存放在g_lpuartTxEdmaHandle结构体中
    EDMA_CreateHandle(&g_lpuartTxEdmaHandle, DMA0, DMAChannel_TX);  
    // 创建DMA句柄，将DMA0， 通道DMAChannel_TX， 等信息存放在g_lpuartRxEdmaHandle结构体中

    if (UART == LPUART1)
        /* Create LPUART DMA handle. */
        LPUART_TransferCreateHandleEDMA(UART, &g_lpuartEdmaHandle, LPUART1_UserCallback, NULL, &g_lpuartTxEdmaHandle,
            &g_lpuartRxEdmaHandle);    
        // 创建串口DMA句柄  并将base，LPUART_UserCallback, NULL, &g_lpuartTxEdmaHandle, &g_lpuartRxEdmaHandle信息存放在g_lpuartEdmaHandle中
}

void Pig_UART::PutChar(uint8_t data)
{
    while (!(UART->STAT & LPUART_STAT_TDRE_MASK));    //等待base->STAT为空
    UART->DATA = data;
}
void Pig_UART::PutBuff(uint8_t * buff, uint32_t length, bool IfUseDMA)
{
    if (!IfUseDMA)
    {
        while (length--)
        {
            while (!(UART->STAT & LPUART_STAT_TDRE_MASK));    //等待base->STAT为空
            UART->DATA = *(buff++);
        }
    }
    else
    {
        /* If TX is idle and g_txBuffer is full, start to send data. */
        /*使用DMA + 串口，无需占用CPU时间 */
        sendXfer.data = buff;
        sendXfer.dataSize = length;

        if (UARTDMAStatusMap[LPUARTIndex] == DMASendFinish)
        {
            UARTDMAStatusMap[LPUARTIndex] = DMASending;
            LPUART_SendEDMA(UART, &g_lpuartEdmaHandle, &sendXfer);
        }
    }
}


int Pig_UART::InterruptPriorityList[8] =
{ 5, 4, 2, 6, 1, 3, 7, 8};//数字越小优先级越高