#include "LQ_MPU6050.h"

//IIC1   SCLk    J11
//IIC1   SDA     K11

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU6050_Init(void)
{
    uint8_t res;
    LPI2C_Init(LPI2C2, 400000);    ////MPU6050 ֧��400K I2C
    res=MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);           //��ȡMPU6050��ID
    if(res!=MPU6050_ID) //����ID��ȷ
    {
        printf("ID=%#X\r\n",res);
        printf("MPU6050 is fail!\n");
    }
    else  printf("MPU6050 is OK!\n");

    res = 0;
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU6050
    delayms(100);  //��ʱ100ms
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU6050
    res += MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps   
    res += MPU_Set_Accel_Fsr(1);					       	 	//���ٶȴ�����,��4g
    res += MPU_Set_Rate(1000);						       	 	//���ò�����1000Hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);      //�������ֵ�ͨ�˲���   98hz
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
//    MPU_Write_Byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
//    MPU_Write_Byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X80);//INT���ŵ͵�ƽ��Ч
    res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//0x01  	//����CLKSEL,PLL X��Ϊ�ο�
    //res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
    
    Get_Offset(&Accel_OriginData, &GYRO_OriginData);
    
    if(res == 0)  //����Ĵ�����д��ɹ�
    {
        printf("MPU set is OK!\n");
    }
    else return 1;
    
    return 0;
}



void Test_MPU6050(void)
{
    MPU6050_Init();
    TFTSPI_Init();               //TFT1.8��ʼ��  
    TFTSPI_CLS(u16BLUE);           //����
    uint8_t  txt[30];
	short aacx,aacy,aacz;	        //���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;        //������ԭʼ���� 
    while(1)
    {
        MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//�õ����ٶȴ���������
        sprintf((char*)txt,"ax:%06d",aacx);
        TFTSPI_P8X16Str(0,1,(uint8_t*)txt,u16RED,u16BLUE);
        sprintf((char*)txt,"ay:%06d",aacy);
        TFTSPI_P8X16Str(0,2,(uint8_t*)txt,u16RED,u16BLUE);
        sprintf((char*)txt,"az:%06d",aacz);
        TFTSPI_P8X16Str(0,3,(uint8_t*)txt,u16RED,u16BLUE);
        sprintf((char*)txt,"gx:%06d",gyrox);
        TFTSPI_P8X16Str(0,4,(uint8_t*)txt,u16RED,u16BLUE);
        sprintf((char*)txt,"gy:%06d",gyroy);
        TFTSPI_P8X16Str(0,5,(uint8_t*)txt,u16RED,u16BLUE);
        sprintf((char*)txt,"gz:%06d",gyroz);
        TFTSPI_P8X16Str(0,6,(uint8_t*)txt,u16RED,u16BLUE);
        printf("\r\nAX: %d  ",aacx); 
        printf("\r\nAY: %d  ",aacy);
        printf("\r\nAZ: %d  ",aacz); 
        printf("\r\nGX: %d  ",gyrox);
        printf("\r\nGY: %d  ",gyroy); 
        printf("\r\nGZ: %d  ",gyroz);
        delayms(100);
        printf("*********************");
    }
    
    

}


//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return (short)temp*100;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res; 
	res=MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}

//�õ��Ӽ�ֵ���¶�ֵ�����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
{
    uint8_t buf[14],res;  
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
                *gx=(((uint16_t)buf[8]<<8)|buf[9] )* 0.0609756097560976;  
		*gy=(((uint16_t)buf[10]<<8)|buf[11])* 0.0609756097560976;  
		*gz=(((uint16_t)buf[12]<<8)|buf[13])* 0.0609756097560976;
//        *ax=((uint16_t)buf[0]);  
//        *ay=((uint16_t)buf[2]);  
//        *az=((uint16_t)buf[4]);
//        *gx=((uint16_t)buf[8]);  
//        *gy=((uint16_t)buf[10]);  
//        *gz=((uint16_t)buf[12]);
	} 	
    return res;
}


//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    return SCCB_WriteMultiRegs(LPI2C2, addr, kSCCB_RegAddr8Bit, reg, buf, len);
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    return SCCB_ReadMultiRegs(LPI2C2, addr, kSCCB_RegAddr8Bit, reg, buf, len);     
}


//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t value)
{
    g_fxosHandle.xfer.slaveAddress = addr;   //��ַ
    return IIC_WriteReg(&g_fxosHandle, reg, value);    //д�Ĵ�������
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t value;
    g_fxosHandle.xfer.slaveAddress = addr;   //��ַ
    IIC_ReadReg(&g_fxosHandle, reg, &value, 1);  //���Ĵ�������
    return value;
}

struct AccelData Accel_OriginData;
void Get_AccData(struct AccelData * Accel_Data)
{
    MPU_Get_Accelerometer((short *)&Accel_Data->X,(short *)&Accel_Data->Y,(short *)&Accel_Data->Z);
  
    Accel_Data->X = (Accel_Data->X - Accel_Data->Offset_X) * 1.225189904435187e-4;
    Accel_Data->Y = (Accel_Data->Y - Accel_Data->Offset_Y) * 1.225189904435187e-4;
    Accel_Data->Z = (Accel_Data->Z - Accel_Data->Offset_Z) * 1.225189904435187e-4;
}

struct GYROData GYRO_OriginData;
void Get_Gyro(struct GYROData * GYRO_Data)
{
    short gyrox,gyroy,gyroz;        //������ԭʼ���� 
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
    GYRO_Data->X = (gyrox - GYRO_Data->Offset_X);//* 0.0609756097560976;
    GYRO_Data->Y = (gyroy - GYRO_Data->Offset_Y);//* 0.0609756097560976;
    GYRO_Data->Z = (gyroz - GYRO_Data->Offset_Z);//* 0.0609756097560976;
}

//��ʼ�������Ǻͼ��ٶȼƵ���ƫֵ
void Get_Offset(struct AccelData * Accel_Data, struct GYROData * GYRO_Data)
{
    int Count_Collect = 0;
    float temp_X = 0, temp_Y = 0, temp_Z = 0;
    while (Count_Collect<200)
    {
        Count_Collect++;
        Get_AccData(Accel_Data);
        temp_X += Accel_Data->X;
        temp_Y += Accel_Data->Y;
        temp_Z += Accel_Data->Z;
    }
    Accel_Data->Offset_X = temp_X / Count_Collect;
    Accel_Data->Offset_Y = temp_Y / Count_Collect;
    Accel_Data->Offset_Z = temp_Z / Count_Collect;

    Count_Collect = 0;
    temp_X = 0;
    temp_Y = 0;
    temp_Z = 0;
    while (Count_Collect<200)
    {
        Count_Collect++;
        Get_Gyro(GYRO_Data);
        temp_X += GYRO_Data->X;
        temp_Y += GYRO_Data->Y;
        temp_Z += GYRO_Data->Z;
    }
    GYRO_Data->Offset_X = temp_X / Count_Collect;
    GYRO_Data->Offset_Y = temp_Y / Count_Collect;
    GYRO_Data->Offset_Z = temp_Z / Count_Collect;
}







