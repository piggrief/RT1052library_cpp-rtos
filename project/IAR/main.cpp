# include "include.h"
# include "Pig_UART.h"

using namespace std;

class RT1052System
{
public:
  RT1052System()
  {
//    BOARD_ConfigMPU();                   /* 初始化内存保护单元 */
//    BOARD_InitSDRAMPins();               /* SDRAM初始化 */
//    BOARD_BootClockRUN();                /* 初始化开发板时钟 */
//    BOARD_InitDEBUG_UARTPins();          //UART调试口管脚复用初始化 
//    BOARD_InitDebugConsole();            //UART调试口初始化 可以使用 PRINTF函数          
//    //LED_Init();                          //初始化核心板和开发板上的LED接口
//    //UART_DMA_Init();       //串口1初始化 可以使用 printf函数
//    //_systime.init();                     //开启systick定时器
//    NVIC_SetPriorityGrouping(2);/*设置中断优先级组  0: 0个抢占优先级16位个子优先级
//                                *1: 2个抢占优先级 8个子优先级 2: 4个抢占优先级 4个子优先级
//                                *3: 8个抢占优先级 2个子优先级 4: 16个抢占优先级 0个子优先级
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
  BOARD_ConfigMPU();                   /* 初始化内存保护单元 */
  BOARD_InitSDRAMPins();               /* SDRAM初始化 */
  BOARD_BootClockRUN();                /* 初始化开发板时钟 */
  BOARD_InitDEBUG_UARTPins();          //UART调试口管脚复用初始化 
  BOARD_InitDebugConsole();            //UART调试口初始化 可以使用 PRINTF函数          
  LED_Init();                          //初始化核心板和开发板上的LED接口
  //UART_DMA_Init();       //串口1初始化 可以使用 printf函数
  //_systime.init();                     //开启systick定时器
  NVIC_SetPriorityGrouping(2);/*设置中断优先级组  0: 0个抢占优先级16位个子优先级
  *1: 2个抢占优先级 8个子优先级 2: 4个抢占优先级 4个子优先级
                                *3: 8个抢占优先级 2个子优先级 4: 16个抢占优先级 0个子优先级
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
