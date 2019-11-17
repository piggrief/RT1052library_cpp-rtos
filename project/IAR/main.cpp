# include "include.h"
# include "Pig_UART.h"

using namespace std;

class RT1052System
{
public:
  RT1052System()
  {
//    BOARD_ConfigMPU();                   /* ��ʼ���ڴ汣����Ԫ */
//    BOARD_InitSDRAMPins();               /* SDRAM��ʼ�� */
//    BOARD_BootClockRUN();                /* ��ʼ��������ʱ�� */
//    BOARD_InitDEBUG_UARTPins();          //UART���ԿڹܽŸ��ó�ʼ�� 
//    BOARD_InitDebugConsole();            //UART���Կڳ�ʼ�� ����ʹ�� PRINTF����          
//    //LED_Init();                          //��ʼ�����İ�Ϳ������ϵ�LED�ӿ�
//    //UART_DMA_Init();       //����1��ʼ�� ����ʹ�� printf����
//    //_systime.init();                     //����systick��ʱ��
//    NVIC_SetPriorityGrouping(2);/*�����ж����ȼ���  0: 0����ռ���ȼ�16λ�������ȼ�
//                                *1: 2����ռ���ȼ� 8�������ȼ� 2: 4����ռ���ȼ� 4�������ȼ�
//                                *3: 8����ռ���ȼ� 2�������ȼ� 4: 16����ռ���ȼ� 0�������ȼ�
//                                */
  }
  ~RT1052System(){}
};
unsigned long counttttt = 0;
void vTask1( void *pvParameters ) 
{
  while(1)
  {
    counttttt++;
    for(unsigned long long i = 0; i < 100000; i++)
      for(unsigned long long j = 0; j < 500; j++);
  }
}
void vTask2( void *pvParameters ) 
{
  LED_Ctrl(LEDALL, OFF);
  while(1)
  {
    LED_Ctrl(LED_G, RVS);
    for(unsigned long long i = 0; i < 100000; i++)
      for(unsigned long long j = 0; j < 500; j++);
  }
}
int state[2] = {0};
int main()
{
  RT1052System MainSys();
  BOARD_ConfigMPU();                   /* ��ʼ���ڴ汣����Ԫ */
  BOARD_InitSDRAMPins();               /* SDRAM��ʼ�� */
  BOARD_BootClockRUN();                /* ��ʼ��������ʱ�� */
  BOARD_InitDEBUG_UARTPins();          //UART���ԿڹܽŸ��ó�ʼ�� 
  BOARD_InitDebugConsole();            //UART���Կڳ�ʼ�� ����ʹ�� PRINTF����          
  LED_Init();                          //��ʼ�����İ�Ϳ������ϵ�LED�ӿ�
  //UART_DMA_Init();       //����1��ʼ�� ����ʹ�� printf����
  //_systime.init();                     //����systick��ʱ��
  NVIC_SetPriorityGrouping(2);/*�����ж����ȼ���  0: 0����ռ���ȼ�16λ�������ȼ�
  *1: 2����ռ���ȼ� 8�������ȼ� 2: 4����ռ���ȼ� 4�������ȼ�
                                *3: 8����ռ���ȼ� 2�������ȼ� 4: 16����ռ���ȼ� 0�������ȼ�
                                */

  
  state[0] = xTaskCreate( vTask1, "Task 1", 200, NULL, 1, NULL ); 
  state[1] = xTaskCreate( vTask2, "Task 2", 200, NULL, 2, NULL ); 
  vTaskStartScheduler(); 
  while(1);
  //Pig_Test Test1 = Pig_Test();
  
  //LED_RGB.Test();
  
  Pig_UART UART_1(LPUART1, 115200, PTL14, PTK14, DMA, 1, 2);
  while(1)
  {
      //UART_1.PutBuff((uint8 *)("Hello!\r\n"), sizeof("Hello!\r\n"), true);
      UART_Put_Buff_DMA((uint8 *)("Hello!\r\n"), sizeof("Hello!\r\n"));
      for(unsigned long long i = 0; i < 100000; i++)
        for(unsigned long long j = 0; j < 500; j++);
  }
  
  return 0;
}
