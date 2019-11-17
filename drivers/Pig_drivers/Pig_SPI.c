/*!
* @file       Pig_SPI.c
* @brief      ���ݹٷ���fsl_lpspi���һЩ����Ľӿ�
* @details    
* @author     pig's grief
* @version    v1.0
* @date       2019-4-10
* @to do
*/
# include "include.h"
# include "Pig_SPI.h"

//�������Ĭ�ϵ�SPI�������ò���
const lpspi_master_config_t DefaultSPI_config = {
    .baudRate = 30000000,                       //�����ʣ��������趨
    .bitsPerFrame = 8,                          //ÿ֡����ı�������һ��Ϊ8
    .cpol = kLPSPI_ClockPolarityActiveHigh,     //����CLK�Ǹߵ�ƽ��Ч���ǵ͵�ƽ��Ч��һ��Ϊ�ߵ�ƽ��Ч
    .cpha = kLPSPI_ClockPhaseFirstEdge,         //�����������ڵ�һ�����ǵڶ��������ز�����һ��Ϊ��һ��������
    .direction = kLPSPI_MsbFirst,               //����������MSB��ǰ����LSB��ǰ��һ��ΪLSB
    .pcsToSckDelayInNanoSec = 100,              //Ƭѡ�ӳ٣�Ҳ����Ƭѡ����ʱ��ÿ�ʼ���䣬��ʹ��SPI��Ӳ��������
    .lastSckToPcsDelayInNanoSec = 100,          //���һ���źŵ�Ƭѡ���ӳ٣������������
    .betweenTransferDelayInNanoSec = 100,       //���δ���֮���ʱ�䣬�����������
    .whichPcs = kLPSPI_Pcs0,                    //ҪƬѡ�Ķ˿�
    .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,  //Ƭѡ�źŵ͵�ƽ��Ч���Ǹߵ�ƽ��Ч��һ��Ϊ�͵�ƽ��Ч
    .pinCfg = kLPSPI_SdiInSdoOut,               //����SDI��SDO������������ķ���һ��ΪSPI�룬SPO��
    .dataOutConfig = kLpspiDataOutRetained      //ѡ���Ƿ���������ݣ�һ��ѡ�����������
};

//�������ר��ΪTFT��SPIͨѶ���õ�SPI�������ò���
const lpspi_master_config_t TFTSPI_config = {
    .baudRate = 30000000,                       //�����ʣ��������趨
    .bitsPerFrame = 8,                          //ÿ֡����ı�������һ��Ϊ8
    .cpol = kLPSPI_ClockPolarityActiveHigh,     //����CLK�Ǹߵ�ƽ��Ч���ǵ͵�ƽ��Ч��һ��Ϊ�ߵ�ƽ��Ч
    .cpha = kLPSPI_ClockPhaseFirstEdge,         //�����������ڵ�һ�����ǵڶ��������ز�����һ��Ϊ��һ��������
    .direction = kLPSPI_MsbFirst,               //����������MSB��ǰ����LSB��ǰ��һ��ΪLSB
    .pcsToSckDelayInNanoSec = 100,              //Ƭѡ�ӳ٣�Ҳ����Ƭѡ����ʱ��ÿ�ʼ���䣬��ʹ��SPI��Ӳ��������
    .lastSckToPcsDelayInNanoSec = 100,          //���һ���źŵ�Ƭѡ���ӳ٣������������
    .betweenTransferDelayInNanoSec = 100,       //���δ���֮���ʱ�䣬�����������
    .whichPcs = kLPSPI_Pcs0,                    //ҪƬѡ�Ķ˿�
    .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,  //Ƭѡ�źŵ͵�ƽ��Ч���Ǹߵ�ƽ��Ч��һ��Ϊ�͵�ƽ��Ч
    .pinCfg = kLPSPI_SdiInSdoOut,               //����SDI��SDO������������ķ���һ��ΪSPI�룬SPO��
    .dataOutConfig = kLpspiDataOutRetained      //ѡ���Ƿ���������ݣ�һ��ѡ�����������
};

//SPI1���������ò���
lpspi_master_config_t SPI1_config = {
    .baudRate = 30000000,                       //�����ʣ��������趨
    .bitsPerFrame = 8,                          //ÿ֡����ı�������һ��Ϊ8
    .cpol = kLPSPI_ClockPolarityActiveHigh,     //����CLK�Ǹߵ�ƽ��Ч���ǵ͵�ƽ��Ч��һ��Ϊ�ߵ�ƽ��Ч
    .cpha = kLPSPI_ClockPhaseFirstEdge,         //�����������ڵ�һ�����ǵڶ��������ز�����һ��Ϊ��һ��������
    .direction = kLPSPI_MsbFirst,               //����������MSB��ǰ����LSB��ǰ��һ��ΪLSB
    .pcsToSckDelayInNanoSec = 100,              //Ƭѡ�ӳ٣�Ҳ����Ƭѡ����ʱ��ÿ�ʼ���䣬��ʹ��SPI��Ӳ��������
    .lastSckToPcsDelayInNanoSec = 100,          //���һ���źŵ�Ƭѡ���ӳ٣������������
    .betweenTransferDelayInNanoSec = 100,       //���δ���֮���ʱ�䣬�����������
    .whichPcs = kLPSPI_Pcs0,                    //ҪƬѡ�Ķ˿�
    .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,  //Ƭѡ�źŵ͵�ƽ��Ч���Ǹߵ�ƽ��Ч��һ��Ϊ�͵�ƽ��Ч
    .pinCfg = kLPSPI_SdiInSdoOut,               //����SDI��SDO������������ķ���һ��ΪSPI�룬SPO��
    .dataOutConfig = kLpspiDataOutRetained      //ѡ���Ƿ���������ݣ�һ��ѡ�����������
};

//SPI2���������ò���
lpspi_master_config_t SPI2_config = {
    .baudRate = 30000000,                       //�����ʣ��������趨
    .bitsPerFrame = 8,                          //ÿ֡����ı�������һ��Ϊ8
    .cpol = kLPSPI_ClockPolarityActiveHigh,     //����CLK�Ǹߵ�ƽ��Ч���ǵ͵�ƽ��Ч��һ��Ϊ�ߵ�ƽ��Ч
    .cpha = kLPSPI_ClockPhaseFirstEdge,         //�����������ڵ�һ�����ǵڶ��������ز�����һ��Ϊ��һ��������
    .direction = kLPSPI_MsbFirst,               //����������MSB��ǰ����LSB��ǰ��һ��ΪLSB
    .pcsToSckDelayInNanoSec = 100,              //Ƭѡ�ӳ٣�Ҳ����Ƭѡ����ʱ��ÿ�ʼ���䣬��ʹ��SPI��Ӳ��������
    .lastSckToPcsDelayInNanoSec = 100,          //���һ���źŵ�Ƭѡ���ӳ٣������������
    .betweenTransferDelayInNanoSec = 100,       //���δ���֮���ʱ�䣬�����������
    .whichPcs = kLPSPI_Pcs0,                    //ҪƬѡ�Ķ˿�
    .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,  //Ƭѡ�źŵ͵�ƽ��Ч���Ǹߵ�ƽ��Ч��һ��Ϊ�͵�ƽ��Ч
    .pinCfg = kLPSPI_SdiInSdoOut,               //����SDI��SDO������������ķ���һ��ΪSPI�룬SPO��
    .dataOutConfig = kLpspiDataOutRetained      //ѡ���Ƿ���������ݣ�һ��ѡ�����������
};

//SPI3���������ò���
lpspi_master_config_t SPI3_config = {
    .baudRate = 30000000,                       //�����ʣ��������趨
    .bitsPerFrame = 8,                          //ÿ֡����ı�������һ��Ϊ8
    .cpol = kLPSPI_ClockPolarityActiveHigh,     //����CLK�Ǹߵ�ƽ��Ч���ǵ͵�ƽ��Ч��һ��Ϊ�ߵ�ƽ��Ч
    .cpha = kLPSPI_ClockPhaseFirstEdge,         //�����������ڵ�һ�����ǵڶ��������ز�����һ��Ϊ��һ��������
    .direction = kLPSPI_MsbFirst,               //����������MSB��ǰ����LSB��ǰ��һ��ΪLSB
    .pcsToSckDelayInNanoSec = 100,              //Ƭѡ�ӳ٣�Ҳ����Ƭѡ����ʱ��ÿ�ʼ���䣬��ʹ��SPI��Ӳ��������
    .lastSckToPcsDelayInNanoSec = 100,          //���һ���źŵ�Ƭѡ���ӳ٣������������
    .betweenTransferDelayInNanoSec = 100,       //���δ���֮���ʱ�䣬�����������
    .whichPcs = kLPSPI_Pcs0,                    //ҪƬѡ�Ķ˿�
    .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,  //Ƭѡ�źŵ͵�ƽ��Ч���Ǹߵ�ƽ��Ч��һ��Ϊ�͵�ƽ��Ч
    .pinCfg = kLPSPI_SdiInSdoOut,               //����SDI��SDO������������ķ���һ��ΪSPI�룬SPO��
    .dataOutConfig = kLpspiDataOutRetained      //ѡ���Ƿ���������ݣ�һ��ѡ�����������
};

//SPI4���������ò���
lpspi_master_config_t SPI4_config = {
    .baudRate = 30000000,                       //�����ʣ��������趨
    .bitsPerFrame = 8,                          //ÿ֡����ı�������һ��Ϊ8
    .cpol = kLPSPI_ClockPolarityActiveHigh,     //����CLK�Ǹߵ�ƽ��Ч���ǵ͵�ƽ��Ч��һ��Ϊ�ߵ�ƽ��Ч
    .cpha = kLPSPI_ClockPhaseFirstEdge,         //�����������ڵ�һ�����ǵڶ��������ز�����һ��Ϊ��һ��������
    .direction = kLPSPI_MsbFirst,               //����������MSB��ǰ����LSB��ǰ��һ��ΪLSB
    .pcsToSckDelayInNanoSec = 100,              //Ƭѡ�ӳ٣�Ҳ����Ƭѡ����ʱ��ÿ�ʼ���䣬��ʹ��SPI��Ӳ��������
    .lastSckToPcsDelayInNanoSec = 100,          //���һ���źŵ�Ƭѡ���ӳ٣������������
    .betweenTransferDelayInNanoSec = 100,       //���δ���֮���ʱ�䣬�����������
    .whichPcs = kLPSPI_Pcs0,                    //ҪƬѡ�Ķ˿�
    .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,  //Ƭѡ�źŵ͵�ƽ��Ч���Ǹߵ�ƽ��Ч��һ��Ϊ�͵�ƽ��Ч
    .pinCfg = kLPSPI_SdiInSdoOut,               //����SDI��SDO������������ķ���һ��ΪSPI�룬SPO��
    .dataOutConfig = kLpspiDataOutRetained      //ѡ���Ƿ���������ݣ�һ��ѡ�����������
};

void SPI_Init(LPSPI_Type * SPIn, uint32_t srcClock_Hz, int IfDefault)
{
    if (SPIn == LPSPI1)
    {
        if (IfDefault)
        {
            SPI1_config = DefaultSPI_config;
        }
        LPSPI_MasterInit(SPIn, &SPI1_config, srcClock_Hz);
    }
    else if (SPIn == LPSPI2)
    {
        if (IfDefault)
        {
            SPI2_config = DefaultSPI_config;
        }
        LPSPI_MasterInit(SPIn, &SPI2_config, srcClock_Hz);
    }
    else if (SPIn == LPSPI3)
    {
        if (IfDefault)
        {
            SPI3_config = DefaultSPI_config;
        }
        LPSPI_MasterInit(SPIn, &SPI3_config, srcClock_Hz);
    }
    else if (SPIn == LPSPI4)
    {
        if (IfDefault)
        {
            SPI4_config = DefaultSPI_config;
        }
        LPSPI_MasterInit(SPIn, &SPI4_config, srcClock_Hz);
    }
}

lpspi_transfer_t NowTrans;
void spi_mosi(LPSPI_Type * SPIn, uint8 * cmdbuff, uint8 * databuff, int dSize)
{
    NowTrans.dataSize = dSize;
    NowTrans.txData = cmdbuff;
    NowTrans.rxData = databuff;

    lpspi_which_pcs_t WhichPCS = kLPSPI_Pcs0;
    if (SPIn == LPSPI1)
        WhichPCS = SPI1_config.whichPcs;
    else if (SPIn == LPSPI2)
        WhichPCS = SPI2_config.whichPcs;
    else if (SPIn == LPSPI3)
        WhichPCS = SPI3_config.whichPcs;
    else if (SPIn == LPSPI4)
        WhichPCS = SPI4_config.whichPcs;

    if (WhichPCS == kLPSPI_Pcs0)
        NowTrans.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    else if (WhichPCS == kLPSPI_Pcs1)
        NowTrans.configFlags = kLPSPI_MasterPcs1 | kLPSPI_MasterPcsContinuous;
    else if (WhichPCS == kLPSPI_Pcs2)
        NowTrans.configFlags = kLPSPI_MasterPcs2 | kLPSPI_MasterPcsContinuous;
    else if (WhichPCS == kLPSPI_Pcs3)
        NowTrans.configFlags = kLPSPI_MasterPcs3 | kLPSPI_MasterPcsContinuous;

    LPSPI_MasterTransferBlocking(SPIn, &NowTrans);
}