/*!
* @file       Pig_UART.c
* @brief      ����fsl�ٷ������C++�Ķ��η�װ�⣨UART���裩
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
//    /*ʹ��DMA + ���ڣ�����ռ��CPUʱ�� */
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

/* ���UARTϵͳʱ��Ƶ�� */
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
        CLOCK_EnableClock(kCLOCK_Lpuart1);	//ʹ��LPUART1ʱ��

        assert(RX == PTL14 && TX == PTK14);

        //LPUART1��ʹ�õ�IO�������ã�������ALT0~ALT7ѡ����ʵĹ��ܡ�
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0U);	//LPUART1_TX   K14
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0U);	//LPUART1_RX   L14

        //����IO����GPIO_AD_B0_12��GPIO_AD_B0_13�Ĺ���
        //��ת���ٶ�,��������ΪR0/6,�ٶ�Ϊ100Mhz���رտ�·���ܣ�ʹ��pull/keepr
        //ѡ��keeper���ܣ�����100K Ohm���ر�Hyst
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0x10B0u);
            
    }
    else if (UART == LPUART2)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart2);	//ʹ��LPUART1ʱ��

        assert(RX == PTM12 && TX == PTL11);

        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_LPUART2_TX, 0U);	//LPUART2_TX    L11
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_LPUART2_RX, 0U);	//LPUART2_RX    M12

        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_02_LPUART2_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_03_LPUART2_RX, 0x10B0u);
    }
    else if (UART == LPUART3)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart3);	//ʹ��LPUART1ʱ��
        
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
        CLOCK_EnableClock(kCLOCK_Lpuart4);	//ʹ��LPUART1ʱ��

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
        CLOCK_EnableClock(kCLOCK_Lpuart5);	//ʹ��LPUART1ʱ��

        assert(RX == PTD14 && TX == PTD13);

        IOMUXC_SetPinMux(IOMUXC_GPIO_B1_12_LPUART5_TX, 0U);	//LPUART5_TX       D13
        IOMUXC_SetPinMux(IOMUXC_GPIO_B1_13_LPUART5_RX, 0U);	//LPUART5_RX       D14

        IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_12_LPUART5_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_13_LPUART5_RX, 0x10B0u);
    }
    else if (UART == LPUART6)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart6);	//ʹ��LPUART1ʱ��

        assert(RX == PTG11 && TX == PTM11);

        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_LPUART6_TX, 0U);	//LPUART6_TX   M11
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_LPUART6_RX, 0U);	//LPUART6_RX   J11

        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_LPUART6_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_LPUART6_RX, 0x10B0u);
    }
    else if (UART == LPUART7)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart7);	//ʹ��LPUART1ʱ��

        assert(RX == PTN4 && TX == PTP3);

        IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_08_LPUART7_TX, 0U);	//LPUART7_TX   P3
        IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_09_LPUART7_RX, 0U);	//LPUART7_RX   N4

        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_08_LPUART7_TX, 0x10B0u);
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_09_LPUART7_RX, 0x10B0u);
    }
    else if (UART == LPUART8)
    {
        CLOCK_EnableClock(kCLOCK_Lpuart8);	//ʹ��LPUART1ʱ��

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
        assert(0);//UART�������ƴ��󣡣���
}

void Pig_UART::FunctionInit(uint32_t Bound)
{
    CLOCK_SetMux(kCLOCK_UartMux, 0); 	//����UARTʱ��ԴΪPLL3 80Mhz��PLL3/6=480/6=80MHz
    CLOCK_SetDiv(kCLOCK_UartDiv, 0); 	//����UARTʱ��1��Ƶ����UARTʱ��Ϊ80Mhz

    lpuart_config_t lpuart_config;              //�������ýṹ��
    LPUART_GetDefaultConfig(&lpuart_config);    //�õ�Ĭ�����ã������ڸ���ʵ���������

    lpuart_config.baudRate_Bps = Bound;						//������
    lpuart_config.dataBitsCount = kLPUART_EightDataBits;		//8λ
    lpuart_config.stopBitCount = kLPUART_OneStopBit;			//1λֹͣλ
    lpuart_config.parityMode = kLPUART_ParityDisabled;		//����żУ��
    lpuart_config.enableRx = true;							//ʹ�ܽ���
    lpuart_config.enableTx = true;							//ʹ�ܷ���

    uint32_t  freq = 0;							//���ڵ�ʱ��ԴƵ��
    freq = GetUartSrcFreq();	                //�õ�����ʱ��Ƶ��

    LPUART_Init(UART, &lpuart_config, freq);				//��ʼ��LPUART1 

    //LPUART�ж�����
    LPUART_EnableInterrupts(UART, kLPUART_RxDataRegFullInterruptEnable); //kLPUART_RxDataRegFullInterruptEnable��ʹ�ܽ����ж�  ʹ�������ж��������޸�

    if (UART == LPUART1)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[0]));
        EnableIRQ(LPUART1_IRQn);	                            //ʹ��LPUART1�ж�      
    }
    else if (UART == LPUART2)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[1]));
        EnableIRQ(LPUART2_IRQn);	                            //ʹ��LPUART1�ж�      
    }
    else if (UART == LPUART3)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[2]));
        EnableIRQ(LPUART3_IRQn);	                            //ʹ��LPUART1�ж�    
    }
    else if (UART == LPUART4)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[3]));
        EnableIRQ(LPUART4_IRQn);	                            //ʹ��LPUART1�ж�     
    }
    else if (UART == LPUART5)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[4]));
        EnableIRQ(LPUART5_IRQn);	                            //ʹ��LPUART1�ж�     
    }
    else if (UART == LPUART6)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[5]));
        EnableIRQ(LPUART6_IRQn);	                            //ʹ��LPUART1�ж�    
    }
    else if (UART == LPUART7)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[6]));
        EnableIRQ(LPUART7_IRQn);	                            //ʹ��LPUART1�ж�       
    }
    else if (UART == LPUART8)
    {
        //���ȼ����� ��ռ���ȼ�1  �����ȼ�2   ԽС���ȼ�Խ��  ��ռ���ȼ��ɴ�ϱ���ж�
        NVIC_SetPriority(LPUART8_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, InterruptPriorityList[7]));
        EnableIRQ(LPUART8_IRQn);	                            //ʹ��LPUART1�ж�       
    }
}

void Pig_UART::DMAFunctionInit(int DMAChannel_RX, int DMAChannel_TX)
{
    /* Init DMAMUX */
    DMAMUX_Init(DMAMUX);              //��ʼ��DMA��·������
    /* Set channel for LPUART */
    DMAMUX_SetSource(DMAMUX, DMAChannel_RX, kDmaRequestMuxLPUART1Tx);   //����DMAͨ��0 ӳ�䵽 LPUART1Tx
    DMAMUX_SetSource(DMAMUX, DMAChannel_TX, kDmaRequestMuxLPUART1Rx);   //����DMAͨ��1 ӳ�䵽 LPUART1Rx
    DMAMUX_EnableChannel(DMAMUX, DMAChannel_RX);                        //ʹ��ͨ��0
    DMAMUX_EnableChannel(DMAMUX, DMAChannel_TX);                        //ʹ��ͨ��1

    edma_config_t config;     //DMA���ýṹ��
    /* Init the EDMA module */
    EDMA_GetDefaultConfig(&config); //�õ�Ĭ��DMA����
    EDMA_Init(DMA0, &config);       //��ʼ������DMA


    EDMA_CreateHandle(&g_lpuartRxEdmaHandle, DMA0, DMAChannel_RX);  
    // ����DMA�������DMA0�� ͨ��DMAChannel_RX�� ����Ϣ�����g_lpuartTxEdmaHandle�ṹ����
    EDMA_CreateHandle(&g_lpuartTxEdmaHandle, DMA0, DMAChannel_TX);  
    // ����DMA�������DMA0�� ͨ��DMAChannel_TX�� ����Ϣ�����g_lpuartRxEdmaHandle�ṹ����

    if (UART == LPUART1)
        /* Create LPUART DMA handle. */
        LPUART_TransferCreateHandleEDMA(UART, &g_lpuartEdmaHandle, LPUART1_UserCallback, NULL, &g_lpuartTxEdmaHandle,
            &g_lpuartRxEdmaHandle);    
        // ��������DMA���  ����base��LPUART_UserCallback, NULL, &g_lpuartTxEdmaHandle, &g_lpuartRxEdmaHandle��Ϣ�����g_lpuartEdmaHandle��
}

void Pig_UART::PutChar(uint8_t data)
{
    while (!(UART->STAT & LPUART_STAT_TDRE_MASK));    //�ȴ�base->STATΪ��
    UART->DATA = data;
}
void Pig_UART::PutBuff(uint8_t * buff, uint32_t length, bool IfUseDMA)
{
    if (!IfUseDMA)
    {
        while (length--)
        {
            while (!(UART->STAT & LPUART_STAT_TDRE_MASK));    //�ȴ�base->STATΪ��
            UART->DATA = *(buff++);
        }
    }
    else
    {
        /* If TX is idle and g_txBuffer is full, start to send data. */
        /*ʹ��DMA + ���ڣ�����ռ��CPUʱ�� */
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
{ 5, 4, 2, 6, 1, 3, 7, 8};//����ԽС���ȼ�Խ��