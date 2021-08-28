#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "car_control.h"

//����ȫ�ֱ�����С���ٶȵ�λֵ
u16 GEAR=2;

//�ۺϲ��Գ���
int main(void)  
 {    
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
		delay_init(168);	     //��ʱ��ʼ��
		uart_init(115200);  //����1��ʼ�� 	 
		PS2_Init();			 //�����˿ڳ�ʼ��
		PS2_SetInit();		 //�����ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	                     //������ģʽ
		Bluetooth_Ctr_Init();  
		TIM3_PWM_Init(500-1,84-1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz. 
		TIM4_PWM_Init(200-1,7199-1);
		CAR_Init();   
		LED_Init();
		LCD_Init();           //��ʼ��LCD FSMC�ӿ�
		TIM_SetCompare1(TIM4, 186);//ԭ90�ȣ��˴�����Ϊ���������ģ�����λ	
		stop();
    while(1)
		{
			Mode_Scan();
			delay_ms(50);
		}       
 }
 
