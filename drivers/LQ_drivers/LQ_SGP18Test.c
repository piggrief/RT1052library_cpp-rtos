
#include "include.h"
/*******************************************************************************
*  SDK�ṩ��������Noncacheable�����建�����ͱ����ķ�����
*  AT_NONCACHEABLE_SECTION_ALIGN(var, alignbytes)
*  AT_NONCACHEABLE_SECTION(var)
******************************************************************************/
AT_NONCACHEABLE_SECTION_ALIGN(uint16_t lcdFrameBuf[2][LCD_HEIGHT][LCD_WIDTH], FRAME_BUFFER_ALIGN);               //LCD���ݻ�����

uint8_t counter;       //LCD��������������һ����ǰ��ʾ�ã�һ��������

int OFFSET0=0;      //��Զ������������ֵ�ۺ�ƫ����
int OFFSET1=0;      //�ڶ���
int OFFSET2=0;      //�����������


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�Z
������˵����oled + camera test
������汾��V1.0
�������¡�2018��11��7�� 
����������
������ֵ����
������ֵ����
��ʵ���� TFT1.8��ʾOV7725 RGBͼ�����  7725�ֱ�������Ϊ320*240
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_SGP18_OV7725(void)
{
    TFTSPI_Init();               //TFT1.8��ʼ��     
    uint32_t fullCameraBufferAddr;     
    LQ_Camera_Init();
//    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR)) {   
//        SCB_DisableICache();
//    }
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR)) {//ע�⣬ʹ��csiFrameBuf����ʱ����ùر�Cache ��Ȼ�ϴ����ݿ��ܻ�����cache���棬������ݴ���
        SCB_DisableDCache();
    }
    delayms(200);        //��ʱ200����     
    while (1)
    {     
        // Wait to get the full frame buffer to show. 
        while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &fullCameraBufferAddr))  //����ͷCSI����������
        {
//          for(int i = 0; i < APP_CAMERA_HEIGHT; i+=2)  //��һ��ȡһ��
//          {
//              for(int j = 0; j < APP_CAMERA_WIDTH ; j+=2)//��һ��ȡһ��
//              {
//                  TFTSPI_Write_Word (*((uint16_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH + j)); //��ʾ����
//              }
//          }
        }   
        CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, fullCameraBufferAddr);//��������������ύ���������        
        TFTSPI_Set_Pos(0,0,APP_CAMERA_WIDTH/2,APP_CAMERA_HEIGHT/2);
        for(int i = 0; i < APP_CAMERA_HEIGHT; i+=2)  //��һ��ȡһ��
        {
            for(int j = 0; j < APP_CAMERA_WIDTH ; j+=2)//��һ��ȡһ��
            {
                TFTSPI_Write_Word (*((uint16_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH + j)); //��ʾ����
            }
        }
        
        LED_Color_Reverse(red); //EVK LED��˸    
    }
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����  �ߡ�Z
������˵����oled + camera test
������汾��V1.0
�������¡�2018��11��7�� 
����������
������ֵ����
������ֵ����
��ʵ���� TFT1.8��ʾ���۶�ֵ�� ͼ�����  ���۷ֱ�������Ϊ752*480
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_SGP18_Camera(void)
{
    TFTSPI_Init();               //TFT1.8��ʼ��  
    TFTSPI_CLS(u16WHITE);           //����
    uint32_t fullCameraBufferAddr;   
#ifdef LQOV7725
    cameraConfig.pixelFormat = kVIDEO_PixelFormatYUYV;
#endif
    LQ_Camera_Init();
    delayms(200);        //��ʱ200����  
    uint16_t color = 0;
//    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR)) {   
//        SCB_DisableICache();
//    }
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR)) {//ע�⣬ʹ��csiFrameBuf����ʱ����ùر�D-Cache ��Ȼ�ϴ����ݿ��ܻ�����cache���棬������ݴ���
        SCB_DisableDCache();
    }
    while (1)
    {     
        // Wait to get the full frame buffer to show. 
        while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &fullCameraBufferAddr))  //����ͷCSI����������
        {
        }   
        
#ifdef LQMT9V034   
        TFTSPI_Set_Pos(0,0,(uint8_t)(APP_CAMERA_WIDTH/2-1) ,APP_CAMERA_HEIGHT);//ע�� ������ʾ��СҪ�������ʵ����ʾ��С��ȣ���Ȼ����ʾ���������߻���
        for(int i = 0; i < APP_CAMERA_HEIGHT; i+=2)  //  480/4/2/2 = 30
        {
            for(int j = 0; j < APP_CAMERA_WIDTH*2; j+=2)//��2��ȡһ��  752*2/4/4 = 188   //�������� һ��94����
            {
                //�Ҷ���ʾ
                color = 0;
                color=(((*((uint8_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH * 2 + j))>>3))<<11;
                color=color|((((*((uint8_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH * 2 + j))>>2))<<5);
                color=color|(((*((uint8_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH * 2 + j))>>3));
                TFTSPI_Write_Word(color);
                //��ֵ����ʾ
//                if(*((uint8_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH * 2 + j) > 0x60)  //��ֵ0x60 ��ֵ����ʾ
//                  TFTSPI_Write_Word (0xffff); //��ʾ����
//                else
//                  TFTSPI_Write_Word (0x0000); //��ʾ����
            }
        }
#else  // 7725 �ĻҶ�ͼ��  ע�⣬���Ҷ�ͼ��ʱ��7725ʹ��YUYV��ʽ cameraConfig = { .pixelFormat = kVIDEO_PixelFormatYUYV }
//        TFTSPI_Set_Pos(0,0,APP_CAMERA_WIDTH/2,APP_CAMERA_HEIGHT/2);
//        for(int i = 0; i < APP_CAMERA_HEIGHT; i+=2)  //��2��ȡһ�� 240 / 2 = 120
//        {
//            for(int j = 1; j < APP_CAMERA_WIDTH *2 ; j+=4)//��4��ȡһ�� ����ͷUYVY��ʽ  Y�ǻҶȣ�320/2=160
//            {
//                if(*((uint8_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH * 2 + j) > 0x60)  //��ֵ0x60 ��ֵ����ʾ
//                  TFTSPI_Write_Word (0xffff); //��ʾ����
//                else
//                  TFTSPI_Write_Word (0x0000); //��ʾ����
//            }
//        }
        TFTSPI_Set_Pos(0,0,APP_CAMERA_WIDTH/4-1,APP_CAMERA_HEIGHT/4);
        for(int i = 0; i < APP_CAMERA_HEIGHT; i+=4)  //��4��ȡһ�� 240 / 4 = 60
        {
            for(int j = 1; j < APP_CAMERA_WIDTH *2 ; j+=8)//��8��ȡһ�� ����ͷUYVY��ʽ  Y�ǻҶȣ�320/4=80
            {
                if(*((uint8_t *)fullCameraBufferAddr +  i * APP_CAMERA_WIDTH * 2 + j) > 0x60)  //��ֵ0x60 ��ֵ����ʾ
                  TFTSPI_Write_Word (0xffff); //��ʾ����
                else
                  TFTSPI_Write_Word (0x0000); //��ʾ����
            }
        }
#endif
        CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, fullCameraBufferAddr);//��������������ύ���������  
        LED_Color_Reverse(red); //EVK LED��˸  
    }
}



