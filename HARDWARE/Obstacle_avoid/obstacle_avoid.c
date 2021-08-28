/****************************************************************************
 * @file: obstacle_avoid.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: 本文件包含了智能小车避障模式的各种功能文件，包括舵机驱动的PWM配置
					 超声波传感器初始化、测距启动函数、获取测距距离函数、避障执行函数
 * @History: // 修改历史记录列表，每条修改记录包括修改日期、修改者及内容简述
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
 * @brief: 配置驱动避障云台舵机的PWM输出
 * @param: arr-自动重装载值		psc-时钟预分频系数
 * @retval：NONE
 * @others: NONE
**/
void TIM4_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* 开启时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //GPIOB6复用为定时器4
	
	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;// PB6
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//TIM4定时器初始化
	TIM_TimeBaseInitStructure.TIM_Period = arr; //PWM 频率=72000/(199+1)=36Khz//设置自动重装载寄存器周期的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;//设置用来作为TIM4时钟频率预分频值
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	//PWM初始化	根据TIM_OCInitStruct中指定的参数初始化外设TIM4
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//PWM输出使能
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;

	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	//注意此处初始化时TIM_OC1Init而不是TIM_OCInit，否则会出错。因为固件库的版本不一样
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能TIM4在CCR1上的预装载寄存器
	
	TIM_Cmd(TIM4,ENABLE);//使能TIM4外设
}

#define SONAR_PORT  GPIOG
#define TRIG_PIN    GPIO_Pin_2
#define ECHO_PIN    GPIO_Pin_3

//定义全局变量--超声波测距距离
float UltrasonicWave_Distance;

/*******************************************************************************
//优先级问题：定时器5->HCSR04_StartMeasure  优先级最低(2,2)
							定时器2->计数									优先级中间(1,1)		
							外部中断3->开始计时						优先级最高(0,0)
							最初的测距不准的原因就是因为优先级设置有误，弄反啦
							现在所用优先级只是测试版正确的优先级设置方法，暂时没有深究
//预分频系数和重装载值问题：定时器2和定时器5均经过计算和测试而取值，能保证有效性
*******************************************************************************/	

/**
 * @brief: 超声波传感器初始化，
 * @param: NONE
 * @retval：NONE
 * @others: NONE
**/

void HCSR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    
    EXTI_InitTypeDef EXTI_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(SONAR_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;                   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    GPIO_Init(SONAR_PORT, &GPIO_InitStructure); 
	
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, GPIO_PinSource3);  // 中断线以及中断初始化配置

    EXTI_ClearITPendingBit(EXTI_Line3);
	
	  /* 外部中断初始化，用于接收检测信号 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);     
		
		/* 外部中断优先级设置 */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断3
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		
		/* TIM2中断初始化，用于计数从而计时 */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 83;//分频系数83，频率为1MHz，理论测量精度0.34mm
		TIM_TimeBaseInitStructure.TIM_Period = 50000;//计数周期50000,相当于0.05s,最大测量范围17m
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
		
		TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM2, //TIM2
		TIM_IT_Update  |  //TIM 中断源
		TIM_IT_Trigger,   //TIM 触发中断源 
		ENABLE  //使能
		);
		
		TIM_Cmd(TIM2, DISABLE); 

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		
		/* TIM2中断优先级设置 */
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);  

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
		
		/* TIM5中断初始化，用于启动测距函数 */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 83;
    TIM_TimeBaseInitStructure.TIM_Period = 100;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
		
		TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM5, //TIM5
		TIM_IT_Update  |  //TIM 中断源
		TIM_IT_Trigger,   //TIM 触发中断源 
		ENABLE  //使能
		);
		
		TIM_Cmd(TIM5, ENABLE); 

    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		
		/* TIM5中断优先级设置 */
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM5中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
		NVIC_Init(&NVIC_InitStructure); 
}

/*Distance measure--------------------------------------------------*/
/**
 * @brief: 启动测距功能函数，触发超声波传感器
 * @param: NONE
 * @retval：NONE
 * @others: 此函数内部的延时函数必须是可重入的，因为放在了TIM5的中断函数内部
						STM32例程里面的延时函数不可重入，so 此处自定义了可重入的延时函数
**/
void HCSR04_StartMeasure(void)
{
    GPIO_SetBits(SONAR_PORT, TRIG_PIN);
    delay_us_unpre(40);   //  The width of trig signal must be greater than 10us.
													//出现在定时器5的中断函数中，所以不能用系统延时
	                        //  自定义普通延时函数，已解决延时失效问题
    GPIO_ResetBits(SONAR_PORT, TRIG_PIN);
		//printf("Test start!");//printf函数也是不可重入函数，开启后导致中断失效
}
/**
 * @brief: 测距函数
 * @param: TIM2计数值
 * @retval：测距距离，units：cm
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
 * @brief: 定时器2中断服务函数
 * @param: NONE
 * @retval：NONE
 * @others: NONE
**/
void TIM2_IRQHandler(void)   //TIM2中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		}
}
/**
 * @brief: 定时器5中断服务函数
 * @param: NONE
 * @retval：NONE
 * @others: 用作规定时间发送启动测距信号
**/
void TIM5_IRQHandler(void)   //TIM5中断
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			HCSR04_StartMeasure();
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		}
}

/**
 * @brief: 外部中断3中断服务函数
 * @param: NONE
 * @retval：NONE
 * @others: 内部将测距距离显示在LCD显示屏上
**/
void EXTI3_IRQHandler(void)
{
		u8 distance[12];				//存放distance字符串
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)//检测到触发信号
			{
            TIM_SetCounter(TIM2,0);//将计数值置0
						TIM_Cmd(TIM2, ENABLE);//开启时钟                                             
						while(GPIO_ReadInputDataBit(SONAR_PORT,ECHO_PIN));//等待低电平	                 
						TIM_Cmd(TIM2, DISABLE);//关闭时钟 
						//此处选择直接在中断函数中求距离，而不是在中断函数中取出计数值在中断函数外部计算 				
            UltrasonicWave_Distance = HCSR04_GetDistance(TIM_GetCounter(TIM2));
						if(UltrasonicWave_Distance>0)
						{
								//printf("distance:%f cm\r\n",UltrasonicWave_Distance);//printf函数也是不可重入函数，开启后导致中断失效
								sprintf((char*)distance,"Dis:%4d cm",(u8)UltrasonicWave_Distance);//将测距距离打印到distance数组。	
								LCD_ShowString(30,110,200,16,16,distance);	
						}
						
			}
    EXTI_ClearITPendingBit(EXTI_Line3);
}
/*Obs_avoid implement--------------------------------------------------------------*/
/**
 * @brief: 利用得到的测距距离进行相应操作完成避障任务
 * @param: NONE
 * @retval：NONE
 * @others: 最初的实时性得不到解决是因为该函数用到了大量延时，现已改为根据测距距离
						进行相应操作的实现方式，实时性问题得以解决

**/
void Obstacle_avoid(void)
{
	extern u16 GEAR;
	if(UltrasonicWave_Distance<30)
	{
		reverse(GEAR);
		while(UltrasonicWave_Distance<=40);
		stop();
		TIM_SetCompare1(TIM4, 181);//探测仪左转45度
		delay_ms(500);
		if(UltrasonicWave_Distance<30)
		{
				TIM_SetCompare1(TIM4, 191);//探测仪右转45度
				delay_ms(500);
				if(UltrasonicWave_Distance<30)
				{
						reverse(GEAR);
						while(UltrasonicWave_Distance<=40);
						right_move_2(GEAR+1);
						delay_ms(500);
						TIM_SetCompare1(TIM4, 186);//探测仪回零位
						drive(GEAR);
				}
				else
				{		
						right_move_2(GEAR+1);
						delay_ms(500);
						TIM_SetCompare1(TIM4, 186);//探测仪回零位
						drive(GEAR);
				}
		}
		else
		{
				left_move_2(GEAR+1);
				delay_ms(500);
				drive(GEAR);			
				TIM_SetCompare1(TIM4, 186);//探测仪回零位
				
				
		}
		
		
	}
}
