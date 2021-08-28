#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "car_control.h"

//定义全局变量，小车速度档位值
u16 GEAR=2;

//综合测试程序
int main(void)  
 {    
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
		delay_init(168);	     //延时初始化
		uart_init(115200);  //串口1初始化 	 
		PS2_Init();			 //驱动端口初始化
		PS2_SetInit();		 //配配置初始化,配置“红绿灯模式”，并选择是否可以修改
	                     //开启震动模式
		Bluetooth_Ctr_Init();  
		TIM3_PWM_Init(500-1,84-1);	//84M/84=1Mhz的计数频率,重装载值500，所以PWM频率为 1M/500=2Khz. 
		TIM4_PWM_Init(200-1,7199-1);
		CAR_Init();   
		LED_Init();
		LCD_Init();           //初始化LCD FSMC接口
		TIM_SetCompare1(TIM4, 186);//原90度，此处设置为超声波测距模块的零位	
		stop();
    while(1)
		{
			Mode_Scan();
			delay_ms(50);
		}       
 }
 
