/*********************************************************************************************************************
* @file       TFTDriver.h
* @brief      1.44��TFT������������
* @details
* @author
* @version    v1.0
* @date       2019-2-16
* @to do      ��д����ͼ��ʾ����

* @����
			  TFT���߶��壺
					------------------------------------
						ģ��ܽ�            ��Ƭ���ܽ�
						SCL                 E2
						SDA                 E1
						RES                 E3
						DC				    E5
						CS                  E4
						BL                  ����
					------------------------------------
********************************************************************************************************************/

#ifndef _TFTDRIVER_H
#define _TFTDRIVER_H
#include "include.h"
extern unsigned char Flag_TFTShow;

# define LPSPI_CLOCK_DIVIDER 7U //LPSPI��Ƶ����,7U����˷�Ƶ

/*�ܽ�����*/
# define TFT_DC_Port GPIO2
# define TFT_DC_Pin 22U       //M4
# define TFT_RES_Port GPIO2 
# define TFT_RES_Pin 21U      //P2
# define IOMUX_LPSPI_SCL IOMUXC_GPIO_AD_B0_00_LPSPI3_SCK
# define IOMUX_LPSPI_PCS IOMUXC_GPIO_AD_B0_04_LPSPI3_PCS1
# define IOMUX_LPSPI_SDO IOMUXC_GPIO_AD_B0_01_LPSPI3_SDO


#define	TFT_X_MAX	128	//Һ��X�����
#define TFT_Y_MAX	160 //Һ��Y�����


//#define DC_PIN		PTB18	//Һ������λӲ������
//#define REST_PIN	PTB19 //Һ����λ���Ŷ���

#define DC(x)   	GPIO_PinWrite(TFT_DC_Port, TFT_DC_Pin, x);
#define REST(x) 	GPIO_PinWrite(TFT_RES_Port, TFT_RES_Pin, x);
#define DC_Init     IOMUXC_SetPinMux(IOMUXC_GPIO_B1_06_GPIO2_IO22 ,0U);
#define RES_Init    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_05_GPIO2_IO21 ,0U);

//-------������ɫ----------
#define RED     	0XF800    //��ɫ
#define GREEN   	0X07E0    //��ɫ
#define BLUE    	0X001F    //��ɫ
#define BRED    	0XF81F
#define GRED    	0XFFE0    //��ɫ
#define GBLUE   	0X07FF    //
#define BLACK   	0X0000    //��ɫ
#define WHITE   	0XFFFF    //��ɫ
#define YELLOW  	0xFFE0    //��ɫ



//����д�ֱʵ���ɫ
#define PENCOLOR RED

//���屳����ɫ
#define BGCOLOR	 WHITE

void TFT_init();
void dsp_single_colour(int color);
void TFT_showchar(uint16 x, uint16 y, uint8 dat, int PenColor, int BackColor);
void TFT_showstr(uint16 x, uint16 y, uint8 const dat[], int PenColor, int BackColor);
int TFT_showfloat(uint16 x, uint16 y, float dat, int ZhengshuWeishu, int XiaoshuWeishu, int PenColor, int BackColor);
int TFT_showUfloat(uint16 x, uint16 y, float dat, int ZhengshuWeishu, int XiaoshuWeishu, int PenColor, int BackColor);
void TFT_showint8(uint16 x, uint16 y, int8 dat, int PenColor, int BackColor);
void TFT_showuint8(uint16 x, uint16 y, uint8 dat, int PenColor, int BackColor);
void TFT_showint16(uint16 x, uint16 y, int16 dat, int PenColor, int BackColor);
void TFT_showuint16(uint16 x, uint16 y, uint16 dat, int PenColor, int BackColor);
void TFT_showimage(const unsigned char *p);
void TFT_showimage_all(const unsigned char *p, int Size_x, int Size_y);
void TFT_showimage_gray(const unsigned char p[40][200], int SizeX_Image, int SizeY_Image, int SizeX_Show, int SizeY_Show);
void displayimage032(uint8 *p, int Gate_To2);

extern const uint8 tft_ascii[95][16];
extern const uint8  asc2_1608[1520];

#endif