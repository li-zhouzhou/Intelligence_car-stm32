/****************************************************************************
 * @file: car_control.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: ���ļ�����������С���ĸ��������ļ�������PWM���á�С����ʼ����ǰ����
					 ���ˡ�ת��ֹͣ��ģʽѡ��ȹ���
 * @History: // �޸���ʷ��¼�б�ÿ���޸ļ�¼�����޸����ڡ��޸��߼����ݼ���
		1.  date:
			  author:
		    modification:
		2. ...
****************************************************************************/

#include "car_control.h"
#include "led.h"
#include "usart.h"

/****************************************************************************
	 Ӳ�����ӣ�
	 PA6 ��1������ģ��ENA	ʹ�ܶˣ�����PWM�źŵ����ٶ�
	 PA7 ��1������ģ��ENB	ʹ�ܶˣ�����PWM�źŵ����ٶ�
	 PB0 ��2������ģ��ENA	ʹ�ܶˣ�����PWM�źŵ����ٶ�
	 PB1 ��2������ģ��ENB	ʹ�ܶˣ�����PWM�źŵ����ٶ�
	 
	 PF1��PF3   ��1������ģ��IN1��IN2    
	 PF5��PF7   ��1������ģ��IN3  IN4          
	 PF13��PF14 ��2������ģ��IN1  IN2	   
	 PF15��PG1  ��2������ģ��IN3  IN4	

	 12V�ӿڽ���12V��Դ����GND�����Դ��ͬʱ��MCU���ء�5V�������
****************************************************************************/

//����ģʽ�����õ�ȫ�ֱ�������ʾ������������	
extern float UltrasonicWave_Distance; 

/*Car config-----------------------------------------------------------------*/
/**
 * @brief: ��ʼ�����Ƶ������ת��ɲ����IO��
 * @param: NONE
 * @retval:	NONE
 * @Others: ѡ��IOʱע�ⲻҪ��STM32��Ӳ���ӿڳ�ͻ��������һ��ʼѡ���IOռ����
						LCD����һ���ӿڣ�����LCD��ʼ��������
**/
void CAR_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOGʱ��
  //IO��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_14;//LED0��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_SetBits(GPIOF,GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_14);
	GPIO_SetBits(GPIOG,GPIO_Pin_1);
}

/**
 * @brief: ��ʼ��PWM������ã�����TIM3ͬʱ���4·PWM��������4�����ת��
 * @param: arr-�Զ���װ��ֵ		psc-ʱ��Ԥ��Ƶϵ��
 * @retval	NONE
 * @Others: GPIO_PinAFConfig��������ֲ����и��ã�������һ�����ú�������һ�𣬲�Ȼֻ��һ·���
**/
void TIM3_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��GPIOAʱ��	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��GPIOBʱ��	
	
	//GPIO_PinAFConfig��������ֲ����и��ã�������һ�����ú�������һ�𣬲�Ȼֻ��һ·���
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOA6����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //GPIOA7����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); //GPIOB0����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3); //GPIOB1����Ϊ��ʱ��3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //��ʼ��PF9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);  	  

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
	
	//��ʼ��TIM3 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3OC2
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3OC3
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3OC4

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);
 
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
										  
}  


/*Car driver-----------------------------------------------------------------*/
/**
 * @brief: ��λ�ٶ�ת������������1-4��������λ���Ե�ת��Ϊ��Ӧ��PWM�����
 * @param: ��λֵ
 * @retval��pulse
 * @Others: NONE
**/
u16 gear_trans(u16 gear)
{
	u16 Pulse=0;
	switch(gear)
	{
		case 0x00:
			Pulse=500;
			break;
		case 0x01:
			Pulse=350;
			break;
		case 0x02:
			Pulse=300;
			break;
		case 0x03:
			Pulse=200;
			break;
		case 0x04:
			Pulse=100;
			break;
		default:
			break;
	}
	return Pulse;
}

/**
 * @brief: ǰ������������С��ǰ��
 * @param: ��λֵ
 * @retval��NONE
 * @Others: NONE
**/
void drive(u16 gear)
{
		u16 Pulse;
		Pulse=gear_trans(gear);
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_1); 
		GPIO_SetBits(GPIOF,GPIO_Pin_3);   
	
		GPIO_SetBits(GPIOF,GPIO_Pin_5);	  
		GPIO_ResetBits(GPIOF,GPIO_Pin_7); 
	
		GPIO_SetBits(GPIOF,GPIO_Pin_15);	   
		GPIO_ResetBits(GPIOG,GPIO_Pin_1); 
	
		GPIO_SetBits(GPIOF,GPIO_Pin_14);	  
		GPIO_ResetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�

}
/**
 * @brief: ���˺���
 * @param: ��λֵ
 * @retval��NONE
 * @Others: NONE
**/
void reverse(u16 gear)	
{
		u16 Pulse;
		Pulse=gear_trans(gear);
	
		GPIO_SetBits(GPIOF,GPIO_Pin_1); 
		GPIO_ResetBits(GPIOF,GPIO_Pin_3); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_5);	  
		GPIO_SetBits(GPIOF,GPIO_Pin_7); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_15);	 
		GPIO_SetBits(GPIOG,GPIO_Pin_1); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_14);	
		GPIO_SetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�

}
/**
 * @brief: ͣ������
 * @param: NONE
 * @retval��NONE
 * @Others: NONE
**/
void stop(void)
{
		GPIO_ResetBits(GPIOF,GPIO_Pin_1); 
		GPIO_ResetBits(GPIOF,GPIO_Pin_3);   
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_5);	 
		GPIO_ResetBits(GPIOF,GPIO_Pin_7); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_15);	  
		GPIO_ResetBits(GPIOG,GPIO_Pin_1); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_14);	 
		GPIO_ResetBits(GPIOF,GPIO_Pin_13); 
}
/**
 * @brief: ��ת����������һ�೵�ֱ���ת��
 * @param: ת�䵵λ
 * @retval��NONE
 * @Others: NONE
**/
void left_move(u16 gear_change)
{
		u16 Pulse;
		Pulse=gear_trans(gear_change);
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_1); 
		GPIO_SetBits(GPIOF,GPIO_Pin_3);   
	
		GPIO_SetBits(GPIOF,GPIO_Pin_5);	  
		GPIO_ResetBits(GPIOF,GPIO_Pin_7); 
	
		GPIO_SetBits(GPIOF,GPIO_Pin_15);	   
		GPIO_ResetBits(GPIOG,GPIO_Pin_1);
	
		GPIO_SetBits(GPIOF,GPIO_Pin_14);	 
		GPIO_ResetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,500);	//�޸ıȽ�ֵ����೵�ֱ���
		TIM_SetCompare2(TIM3,500);	//�޸ıȽ�ֵ����೵�ֱ���
		TIM_SetCompare3(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}
/**
 * @brief: ��ת����������һ�೵�ֱ���ת��
 * @param: ת�䵵λ
 * @retval��NONE
 * @Others: NONE
**/
void right_move(u16 gear_change)
{
		u16 Pulse;
		Pulse=gear_trans(gear_change);
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_1); 
		GPIO_SetBits(GPIOF,GPIO_Pin_3);  
	
		GPIO_SetBits(GPIOF,GPIO_Pin_5);	  
		GPIO_ResetBits(GPIOF,GPIO_Pin_7); 
	
		GPIO_SetBits(GPIOF,GPIO_Pin_15);	
		GPIO_ResetBits(GPIOG,GPIO_Pin_1);
	
		GPIO_SetBits(GPIOF,GPIO_Pin_14);	 
		GPIO_ResetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,500);	//�޸ıȽ�ֵ���Ҳ೵�ֱ���
		TIM_SetCompare4(TIM3,500);	//�޸ıȽ�ֵ���Ҳ೵�ֱ���
}
/**
 * @brief: ��ת����������һ��ǰ��һ�����ת��
 * @param: ת�䵵λ
 * @retval��NONE
 * @Others: NONE
**/
void left_move_2(u16 gear_change)
{
		u16 Pulse;
		Pulse=gear_trans(gear_change);
	
		GPIO_SetBits(GPIOF,GPIO_Pin_1);  
		GPIO_ResetBits(GPIOF,GPIO_Pin_3);  
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_5);	  
		GPIO_SetBits(GPIOF,GPIO_Pin_7); 
	
		GPIO_SetBits(GPIOF,GPIO_Pin_15);	  
		GPIO_ResetBits(GPIOG,GPIO_Pin_1); 
	
		GPIO_SetBits(GPIOF,GPIO_Pin_14);	 
		GPIO_ResetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}
/**
 * @brief: ��ת����������һ��ǰ��һ�����ת��
 * @param: ת�䵵λ
 * @retval��NONE
 * @Others: NONE
**/
void right_move_2(u16 gear_change)
{
		u16 Pulse;
		Pulse=gear_trans(gear_change);
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_1);  
		GPIO_SetBits(GPIOF,GPIO_Pin_3);  
	
		GPIO_SetBits(GPIOF,GPIO_Pin_5);	  
		GPIO_ResetBits(GPIOF,GPIO_Pin_7); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_15);	 
		GPIO_SetBits(GPIOG,GPIO_Pin_1); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_14);	   
		GPIO_SetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,Pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}
/**
 * @brief: ǰ����������Ҫ����ң��ģʽ
 * @param: pulseֵ
 * @retval��NONE
 * @Others: NONE
**/
void drive_pulse(int pulse)
{	
		GPIO_ResetBits(GPIOF,GPIO_Pin_1); 
		GPIO_SetBits(GPIOF,GPIO_Pin_3);  
	
		GPIO_SetBits(GPIOF,GPIO_Pin_5);	   
		GPIO_ResetBits(GPIOF,GPIO_Pin_7);
	
		GPIO_SetBits(GPIOF,GPIO_Pin_15);	  
		GPIO_ResetBits(GPIOG,GPIO_Pin_1);
	
		GPIO_SetBits(GPIOF,GPIO_Pin_14);	  
		GPIO_ResetBits(GPIOF,GPIO_Pin_13);
	
		TIM_SetCompare1(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}

/**
 * @brief: ���˺�������Ҫ����ң��ģʽ
 * @param: pulseֵ
 * @retval��NONE
 * @Others: NONE
**/
void reverse_pulse(int pulse)
{
		GPIO_SetBits(GPIOF,GPIO_Pin_1); 
		GPIO_ResetBits(GPIOF,GPIO_Pin_3);  
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_5);	  
		GPIO_SetBits(GPIOF,GPIO_Pin_7);
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_15);	
		GPIO_SetBits(GPIOG,GPIO_Pin_1); 
	
		GPIO_ResetBits(GPIOF,GPIO_Pin_14);	 
		GPIO_SetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,pulse);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}

/**
 * @brief: ת�亯������Ҫ����ң��ģʽ
 * @param: pulse1-��࣬pulse2-�Ҳ�
 * @retval��NONE
 * @Others: NONE
**/
void turn_pulse(int pulse1,int pulse2)
{
	 GPIO_ResetBits(GPIOF,GPIO_Pin_1); 
		GPIO_SetBits(GPIOF,GPIO_Pin_3);   
	
		GPIO_SetBits(GPIOF,GPIO_Pin_5);	   
		GPIO_ResetBits(GPIOF,GPIO_Pin_7);
	
		GPIO_SetBits(GPIOF,GPIO_Pin_15);	
		GPIO_ResetBits(GPIOG,GPIO_Pin_1); 
	
		GPIO_SetBits(GPIOF,GPIO_Pin_14);	  
		GPIO_ResetBits(GPIOF,GPIO_Pin_13); 
	
		TIM_SetCompare1(TIM3,pulse1);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare2(TIM3,pulse1);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM3,pulse2);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM3,pulse2);	//�޸ıȽ�ֵ���޸�ռ�ձ�
}


/*Mode config-----------------------------------------------------------------*/
/**
 * @brief: ģʽѡ���ʼ������,���ڸ���ģʽ�ĳ�ʼ��
 * @param: ����ֵ��ң�غ�����ģʽͨ��
 * @retval��NONE 
 * @Others: ��������������Ϊֱ����ң�غ�����ѡ��ģʽ��
						����ң������ģʽ�ĳ�ʼ�������������У��˴���������
**/
void Mode_select(u8 key)
{
	switch(key)
	{
		case 13://REMOTE_CTR_MODE

			break;
		case 15://BLUETOOTH_MODE

			break;
		case 16://TRACE_MODE
				Trace_Init();	
			break;
		case 14://OBSTACLE_AVOID_MODE
				HCSR04_Init();
			break;
		
		case 66://REMOTE_CTR_MODE

			break;
		case 65://BLUETOOTH_MODE

			break;
		case 67://TRACE_MODE
				Trace_Init();	
			break;
		case 68://OBSTACLE_AVOID_MODE
				HCSR04_Init();
			break;
		default:
			break;
		
	}
}

/**
 * @brief: ģʽ���й��ܺ���
 * @param: ģʽѡ�񰴼�ֵ��ң�غ�����ģʽͨ��
 * @retval��NONE
 * @Others: NONE
**/
void Mode_run(u8 mode)
{
	switch(mode)
	{
		case 13://REMOTE_CTR_MODE
				Remote_Ctr();
			break;
		case 15://BLUETOOTH_MODE
				Bluetooth_Ctr();
			break;
		case 16://TRACE_MODE
				TRACE_Implement();
			break;
		case 14://OBSTACLE_AVOID_MODE
				Obstacle_avoid();
			break;
		
		case 66://REMOTE_CTR_MODE
				Remote_Ctr();
			break;
		case 65://BLUETOOTH_MODE
				Bluetooth_Ctr();
			break;
		case 67://TRACE_MODE
				TRACE_Implement();
			break;
		case 68://OBSTACLE_AVOID_MODE
				Obstacle_avoid();
		
			break;
		default:
			break;
	}
}


/**
 * @brief: ģʽѡ��ɨ�躯��������ɨ�谴�µļ�ֵ�Ӷ�������Ӧ�Ĳ���
 * @param: NONE
 * @retval��NONE
 * @Others: NONE
**/
void Mode_Scan(void)
{
		static int Start_flag=0;//��ʼ���б�־λ����0ʱС����ʼ����
		static u8 key1;//ң��ģʽ�µ�ѡ���ֵ  
		static u8 mode1;//ң��ģʽ�µ����м�ֵ  
		static u8 key2;//����ģʽ�µ�ѡ���ֵ    
		static u8 mode2;//����ģʽ�µ����м�ֵ  
		
		//ң��ģʽɨ��
		key1=PS2_DataKey();
		if(key1==13|key1==14|key1==15|key1==16)//ģʽѡ�����а�������
		{
		printf("  \r\n   %d  is  light  \r\n",Data[1]);//ID
		printf("  \r\n   %d  is  pressed  \r\n",key1);
		Mode_select(key1);
		mode1=key1;
		Start_flag=0;
		if(key1 == 11)
		{
			PS2_Vibration(0xFF,0x00); //�����𶯺��������ʱ  delay_ms(1000);
			delay_ms(500);
		}
		else if(key1 == 12)
		{
			PS2_Vibration(0x00,0xFF); //�����𶯺��������ʱ  delay_ms(1000);
			delay_ms(500);
		}
		else
			 PS2_Vibration(0x00,0x00); 
		}
		//����start������־λ��1��׼������ִ��״̬
		if(key1==4)   
		{
			Start_flag=1;
		}
		//start�����£���־λ��1������ִ��״̬
		if(Start_flag==1)
		{
			Mode_run(mode1);
		}
		printf(" %5d %5d %5d %5d\r\n",PS2_AnologData(PSS_LX),PS2_AnologData(PSS_LY),
																	PS2_AnologData(PSS_RX),PS2_AnologData(PSS_RY) );
		
		//����ģʽɨ��
		if(USART_RX_STA&0x8000)
		{					   
			printf("\r\n�����͵���ϢΪ:\r\n");		
			USART_SendData(USART1, USART_RX_BUF[0]);//����ֵ���������Դ��ڵ�������
			USART_SendData(USART2, USART_RX_BUF[0]); //����ֵ�������ֻ��������ڵ�������
			//ģʽѡ�����а�������
			if(USART_RX_BUF[0]==0x41|USART_RX_BUF[0]==0x42|USART_RX_BUF[0]==0x43|USART_RX_BUF[0]==0x44)
			{
				key2=USART_RX_BUF[0];
				Mode_select(key2);
				Start_flag=0;
			}
			mode2=USART_RX_BUF[0];
			//����start������־λ��2��׼������ִ��״̬
			if(mode2==0x46)   
			{
				Start_flag=2;
			}
			
			printf("\r\n\r\n");//���뻻��
			USART_RX_STA=0;

		}
		//start�����£���־λ��2������ִ��״̬
		if(Start_flag==2)
		{
			Mode_run(key2);
		}
		
		
		//��ͣ
		if(key1==1||mode2==83)
		{
			stop();
			Start_flag=0;
		}

}

