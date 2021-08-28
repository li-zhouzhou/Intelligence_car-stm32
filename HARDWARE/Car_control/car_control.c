/****************************************************************************
 * @file: car_control.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: 本文件包含了智能小车的各种驱动文件，包括PWM配置、小车初始化、前进、
					 后退、转向、停止、模式选择等功能
 * @History: // 修改历史记录列表，每条修改记录包括修改日期、修改者及内容简述
		1.  date:
			  author:
		    modification:
		2. ...
****************************************************************************/

#include "car_control.h"
#include "led.h"
#include "usart.h"

/****************************************************************************
	 硬件连接：
	 PA6 接1号驱动模块ENA	使能端，输入PWM信号调节速度
	 PA7 接1号驱动模块ENB	使能端，输入PWM信号调节速度
	 PB0 接2号驱动模块ENA	使能端，输入PWM信号调节速度
	 PB1 接2号驱动模块ENB	使能端，输入PWM信号调节速度
	 
	 PF1、PF3   接1号驱动模块IN1、IN2    
	 PF5、PF7   接1号驱动模块IN3  IN4          
	 PF13、PF14 接2号驱动模块IN1  IN2	   
	 PF15、PG1  接2号驱动模块IN3  IN4	

	 12V接口接入12V电源正、GND接入电源负同时与MCU共地、5V输出不接
****************************************************************************/

//避障模式下设置的全局变量，表示超声波测距距离	
extern float UltrasonicWave_Distance; 

/*Car config-----------------------------------------------------------------*/
/**
 * @brief: 初始化控制电机正反转及刹车的IO口
 * @param: NONE
 * @retval:	NONE
 * @Others: 选择IO时注意不要和STM32的硬件接口冲突，比如我一开始选择的IO占用了
						LCD屏的一个接口，导致LCD初始化不正常
**/
void CAR_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOG时钟
  //IO初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_14;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_SetBits(GPIOF,GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_14);
	GPIO_SetBits(GPIOG,GPIO_Pin_1);
}

/**
 * @brief: 初始化PWM输出配置，利用TIM3同时输出4路PWM波，控制4个电机转速
 * @param: arr-自动重装载值		psc-时钟预分频系数
 * @retval	NONE
 * @Others: GPIO_PinAFConfig函数必须分步进行复用，不能用一个复用函数并在一起，不然只有一路输出
**/
void TIM3_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能GPIOA时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能GPIOB时钟	
	
	//GPIO_PinAFConfig函数必须分步进行复用，不能用一个复用函数并在一起，不然只有一路输出
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOA6复用为定时器3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //GPIOA7复用为定时器3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); //GPIOB0复用为定时器3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3); //GPIOB1复用为定时器3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //初始化PF9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);  	  

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM3 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3OC2
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3OC3
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3OC4

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);
 
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
										  
}  


/*Car driver-----------------------------------------------------------------*/
/**
 * @brief: 档位速度转换函数，设置1-4档，将档位线性的转换为相应的PWM波输出
 * @param: 档位值
 * @retval：pulse
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
 * @brief: 前进函数，控制小车前进
 * @param: 档位值
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,Pulse);	//修改比较值，修改占空比

}
/**
 * @brief: 后退函数
 * @param: 档位值
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,Pulse);	//修改比较值，修改占空比

}
/**
 * @brief: 停车函数
 * @param: NONE
 * @retval：NONE
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
 * @brief: 左转函数，利用一侧车轮抱死转弯
 * @param: 转弯档位
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,500);	//修改比较值，左侧车轮抱死
		TIM_SetCompare2(TIM3,500);	//修改比较值，左侧车轮抱死
		TIM_SetCompare3(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,Pulse);	//修改比较值，修改占空比
}
/**
 * @brief: 右转函数，利用一侧车轮抱死转弯
 * @param: 转弯档位
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,500);	//修改比较值，右侧车轮抱死
		TIM_SetCompare4(TIM3,500);	//修改比较值，右侧车轮抱死
}
/**
 * @brief: 左转函数，利用一侧前进一侧后退转弯
 * @param: 转弯档位
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,Pulse);	//修改比较值，修改占空比
}
/**
 * @brief: 右转函数，利用一侧前进一侧后退转弯
 * @param: 转弯档位
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,Pulse);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,Pulse);	//修改比较值，修改占空比
}
/**
 * @brief: 前进函数，主要用于遥控模式
 * @param: pulse值
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,pulse);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,pulse);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,pulse);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,pulse);	//修改比较值，修改占空比
}

/**
 * @brief: 后退函数，主要用于遥控模式
 * @param: pulse值
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,pulse);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,pulse);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,pulse);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,pulse);	//修改比较值，修改占空比
}

/**
 * @brief: 转弯函数，主要用于遥控模式
 * @param: pulse1-左侧，pulse2-右侧
 * @retval：NONE
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
	
		TIM_SetCompare1(TIM3,pulse1);	//修改比较值，修改占空比
		TIM_SetCompare2(TIM3,pulse1);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM3,pulse2);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM3,pulse2);	//修改比较值，修改占空比
}


/*Mode config-----------------------------------------------------------------*/
/**
 * @brief: 模式选择初始化函数,用于各个模式的初始化
 * @param: 按键值，遥控和蓝牙模式通用
 * @retval：NONE 
 * @Others: 由于主函数设置为直接用遥控和蓝牙选择模式，
						所以遥控蓝牙模式的初始化在主函数进行，此处不做设置
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
 * @brief: 模式运行功能函数
 * @param: 模式选择按键值，遥控和蓝牙模式通用
 * @retval：NONE
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
 * @brief: 模式选择扫描函数，用于扫描按下的键值从而进行相应的操作
 * @param: NONE
 * @retval：NONE
 * @Others: NONE
**/
void Mode_Scan(void)
{
		static int Start_flag=0;//开始运行标志位，非0时小车开始运行
		static u8 key1;//遥控模式下的选择键值  
		static u8 mode1;//遥控模式下的运行键值  
		static u8 key2;//蓝牙模式下的选择键值    
		static u8 mode2;//蓝牙模式下的运行键值  
		
		//遥控模式扫描
		key1=PS2_DataKey();
		if(key1==13|key1==14|key1==15|key1==16)//模式选择功能有按键按下
		{
		printf("  \r\n   %d  is  light  \r\n",Data[1]);//ID
		printf("  \r\n   %d  is  pressed  \r\n",key1);
		Mode_select(key1);
		mode1=key1;
		Start_flag=0;
		if(key1 == 11)
		{
			PS2_Vibration(0xFF,0x00); //发出震动后必须有延时  delay_ms(1000);
			delay_ms(500);
		}
		else if(key1 == 12)
		{
			PS2_Vibration(0x00,0xFF); //发出震动后必须有延时  delay_ms(1000);
			delay_ms(500);
		}
		else
			 PS2_Vibration(0x00,0x00); 
		}
		//按下start键，标志位置1，准备进入执行状态
		if(key1==4)   
		{
			Start_flag=1;
		}
		//start键按下，标志位置1，进入执行状态
		if(Start_flag==1)
		{
			Mode_run(mode1);
		}
		printf(" %5d %5d %5d %5d\r\n",PS2_AnologData(PSS_LX),PS2_AnologData(PSS_LY),
																	PS2_AnologData(PSS_RX),PS2_AnologData(PSS_RY) );
		
		//蓝牙模式扫描
		if(USART_RX_STA&0x8000)
		{					   
			printf("\r\n您发送的消息为:\r\n");		
			USART_SendData(USART1, USART_RX_BUF[0]);//按键值发送至电脑串口调试助手
			USART_SendData(USART2, USART_RX_BUF[0]); //按键值发送至手机蓝牙串口调试助手
			//模式选择功能有按键按下
			if(USART_RX_BUF[0]==0x41|USART_RX_BUF[0]==0x42|USART_RX_BUF[0]==0x43|USART_RX_BUF[0]==0x44)
			{
				key2=USART_RX_BUF[0];
				Mode_select(key2);
				Start_flag=0;
			}
			mode2=USART_RX_BUF[0];
			//按下start键，标志位置2，准备进入执行状态
			if(mode2==0x46)   
			{
				Start_flag=2;
			}
			
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;

		}
		//start键按下，标志位置2，进入执行状态
		if(Start_flag==2)
		{
			Mode_run(key2);
		}
		
		
		//软急停
		if(key1==1||mode2==83)
		{
			stop();
			Start_flag=0;
		}

}

