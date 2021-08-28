/****************************************************************************
 * @file: obstacle_avoid.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: ���ļ�����������С������ģʽ�ĸ��ֹ����ļ����������������PWM����
					 ��������������ʼ�������������������ȡ�����뺯��������ִ�к���
 * @History: // �޸���ʷ��¼�б�ÿ���޸ļ�¼�����޸����ڡ��޸��߼����ݼ���
		1.  date:
			author:
		    modification:
		2. ...
****************************************************************************/

/* Includes-------------------------------------------------------------------*/
#include "delay.h"
#include "usart.h"
#include "car_control.h"
#include "lcd.h"

/*Obs_avoid init-----------------------------------------------------------------*/
/**
 * @brief: ��������������̨�����PWM���
 * @param: arr-�Զ���װ��ֵ		psc-ʱ��Ԥ��Ƶϵ��
 * @retval��NONE
 * @others: NONE
**/
void TIM4_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* ����ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //GPIOB6����Ϊ��ʱ��4
	
	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;// PB6
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//TIM4��ʱ����ʼ��
	TIM_TimeBaseInitStructure.TIM_Period = arr; //PWM Ƶ��=72000/(199+1)=36Khz//�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;//����������ΪTIM4ʱ��Ƶ��Ԥ��Ƶֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	//PWM��ʼ��	����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIM4
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//PWM���ʹ��
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;

	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	//ע��˴���ʼ��ʱTIM_OC1Init������TIM_OCInit������������Ϊ�̼���İ汾��һ��
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM4,ENABLE);//ʹ��TIM4����
}

#define SONAR_PORT  GPIOG
#define TRIG_PIN    GPIO_Pin_2
#define ECHO_PIN    GPIO_Pin_3

//����ȫ�ֱ���--������������
float UltrasonicWave_Distance;

/*******************************************************************************
//���ȼ����⣺��ʱ��5->HCSR04_StartMeasure  ���ȼ����(2,2)
							��ʱ��2->����									���ȼ��м�(1,1)		
							�ⲿ�ж�3->��ʼ��ʱ						���ȼ����(0,0)
							����Ĳ�಻׼��ԭ�������Ϊ���ȼ���������Ū����
							�����������ȼ�ֻ�ǲ��԰���ȷ�����ȼ����÷�������ʱû���
//Ԥ��Ƶϵ������װ��ֵ���⣺��ʱ��2�Ͷ�ʱ��5����������Ͳ��Զ�ȡֵ���ܱ�֤��Ч��
*******************************************************************************/	

/**
 * @brief: ��������������ʼ����
 * @param: NONE
 * @retval��NONE
 * @others: NONE
**/

void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    
    EXTI_InitTypeDef EXTI_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(SONAR_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;                   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    GPIO_Init(SONAR_PORT, &GPIO_InitStructure); 
	
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, GPIO_PinSource3);  // �ж����Լ��жϳ�ʼ������

    EXTI_ClearITPendingBit(EXTI_Line3);
	
	  /* �ⲿ�жϳ�ʼ�������ڽ��ռ���ź� */
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);     
		
		/* �ⲿ�ж����ȼ����� */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//�ⲿ�ж�3
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		
		/* TIM2�жϳ�ʼ�������ڼ����Ӷ���ʱ */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 83;//��Ƶϵ��83��Ƶ��Ϊ1MHz�����۲�������0.34mm
		TIM_TimeBaseInitStructure.TIM_Period = 50000;//��������50000,�൱��0.05s,��������Χ17m
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
		
		TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM2, //TIM2
		TIM_IT_Update  |  //TIM �ж�Դ
		TIM_IT_Trigger,   //TIM �����ж�Դ 
		ENABLE  //ʹ��
		);
		
		TIM_Cmd(TIM2, DISABLE); 

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		
		/* TIM2�ж����ȼ����� */
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);  

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
		
		/* TIM5�жϳ�ʼ��������������ຯ�� */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 83;
    TIM_TimeBaseInitStructure.TIM_Period = 100;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
		
		TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM5, //TIM5
		TIM_IT_Update  |  //TIM �ж�Դ
		TIM_IT_Trigger,   //TIM �����ж�Դ 
		ENABLE  //ʹ��
		);
		
		TIM_Cmd(TIM5, ENABLE); 

    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		
		/* TIM5�ж����ȼ����� */
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM5�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
		NVIC_Init(&NVIC_InitStructure); 
}

/*Distance measure--------------------------------------------------*/
/**
 * @brief: ������๦�ܺ���������������������
 * @param: NONE
 * @retval��NONE
 * @others: �˺����ڲ�����ʱ���������ǿ�����ģ���Ϊ������TIM5���жϺ����ڲ�
						STM32�����������ʱ�����������룬so �˴��Զ����˿��������ʱ����
**/
void HCSR04_StartMeasure(void)
{
    GPIO_SetBits(SONAR_PORT, TRIG_PIN);
    delay_us_unpre(40);   //  The width of trig signal must be greater than 10us.
													//�����ڶ�ʱ��5���жϺ����У����Բ�����ϵͳ��ʱ
	                        //  �Զ�����ͨ��ʱ�������ѽ����ʱʧЧ����
    GPIO_ResetBits(SONAR_PORT, TRIG_PIN);
		//printf("Test start!");//printf����Ҳ�ǲ������뺯�������������ж�ʧЧ
}
/**
 * @brief: ��ຯ��
 * @param: TIM2����ֵ
 * @retval�������룬units��cm
 * @others: NONE
**/
float HCSR04_GetDistance(u32 count)
{
		float distance;
    // distance = measurement/2/1000*340 = measurement/59 (cm)  measurement-units:us
    distance = (float)count / 58.8 ; 
    return distance;
}


/*ISR-------------------------------------------------------------------*/
/**
 * @brief: ��ʱ��2�жϷ�����
 * @param: NONE
 * @retval��NONE
 * @others: NONE
**/
void TIM2_IRQHandler(void)   //TIM2�ж�
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		}
}
/**
 * @brief: ��ʱ��5�жϷ�����
 * @param: NONE
 * @retval��NONE
 * @others: �����涨ʱ�䷢����������ź�
**/
void TIM5_IRQHandler(void)   //TIM5�ж�
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
			HCSR04_StartMeasure();
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		}
}

/**
 * @brief: �ⲿ�ж�3�жϷ�����
 * @param: NONE
 * @retval��NONE
 * @others: �ڲ�����������ʾ��LCD��ʾ����
**/
void EXTI3_IRQHandler(void)
{
		u8 distance[12];				//���distance�ַ���
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)//��⵽�����ź�
			{
            TIM_SetCounter(TIM2,0);//������ֵ��0
						TIM_Cmd(TIM2, ENABLE);//����ʱ��                                             
						while(GPIO_ReadInputDataBit(SONAR_PORT,ECHO_PIN));//�ȴ��͵�ƽ	                 
						TIM_Cmd(TIM2, DISABLE);//�ر�ʱ�� 
						//�˴�ѡ��ֱ�����жϺ���������룬���������жϺ�����ȡ������ֵ���жϺ����ⲿ���� 				
            UltrasonicWave_Distance = HCSR04_GetDistance(TIM_GetCounter(TIM2));
						if(UltrasonicWave_Distance>0)
						{
								//printf("distance:%f cm\r\n",UltrasonicWave_Distance);//printf����Ҳ�ǲ������뺯�������������ж�ʧЧ
								sprintf((char*)distance,"Dis:%4d cm",(u8)UltrasonicWave_Distance);//���������ӡ��distance���顣	
								LCD_ShowString(30,110,200,16,16,distance);	
						}
						
			}
    EXTI_ClearITPendingBit(EXTI_Line3);
}
/*Obs_avoid implement--------------------------------------------------------------*/
/**
 * @brief: ���õõ��Ĳ����������Ӧ������ɱ�������
 * @param: NONE
 * @retval��NONE
 * @others: �����ʵʱ�Եò����������Ϊ�ú����õ��˴�����ʱ�����Ѹ�Ϊ���ݲ�����
						������Ӧ������ʵ�ַ�ʽ��ʵʱ��������Խ��

**/
void Obstacle_avoid(void)
{
	extern u16 GEAR;
	if(UltrasonicWave_Distance<30)
	{
		reverse(GEAR);
		while(UltrasonicWave_Distance<=40);
		stop();
		TIM_SetCompare1(TIM4, 181);//̽������ת45��
		delay_ms(500);
		if(UltrasonicWave_Distance<30)
		{
				TIM_SetCompare1(TIM4, 191);//̽������ת45��
				delay_ms(500);
				if(UltrasonicWave_Distance<30)
				{
						reverse(GEAR);
						while(UltrasonicWave_Distance<=40);
						right_move_2(GEAR+1);
						delay_ms(500);
						TIM_SetCompare1(TIM4, 186);//̽���ǻ���λ
						drive(GEAR);
				}
				else
				{		
						right_move_2(GEAR+1);
						delay_ms(500);
						TIM_SetCompare1(TIM4, 186);//̽���ǻ���λ
						drive(GEAR);
				}
		}
		else
		{
				left_move_2(GEAR+1);
				delay_ms(500);
				drive(GEAR);			
				TIM_SetCompare1(TIM4, 186);//̽���ǻ���λ
				
				
		}
		
		
	}
}
