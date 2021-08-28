/****************************************************************************
 * @file: trace.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: 本文件包含了智能小车巡线模块的各种功能函数，包括IO口初始化、巡线
					 传感器接收扫描、巡线执行
 * @History: // 修改历史记录列表，每条修改记录包括修改日期、修改者及内容简述
		1.  date:
			author:
		    modification:
		2. ...
****************************************************************************/

#include "delay.h"
#include "sys.h"
#include "car_control.h"
#include "led.h"

/*Trace init-----------------------------------------------------------------*/
/**
 * @brief: 循迹传感器所接IO口初始化
 * @param: NONE
 * @retval：NONE
 * @others: 最初循迹模块上面开关指示灯常亮的原因是选错了IO口，用了PC0和PC2
					  换成PF2和PF4就好啦，why？
**/
void Trace_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//使能GPIOF时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;       //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure);           
	
}
/*Trace scan&&implement-------------------------------------------------------*/
/**
 * @brief: 循迹模块扫描函数，参照STM32按键实验
 * @param: mode，==1表示支持连续按，==0表示不支持连续按
 * @retval: 4个循迹模块的编号，返回1表示1号传感器压线
 * @others: NONE
**/

u8 TRACE_Scan(u8 mode)
{	 
	static u8 trace_up=1;//按键按松开标志
	if(mode)trace_up=1;  //支持连按		  
	if(trace_up&&(TRACE1==1||TRACE2==1||TRACE3==1||TRACE4==1))
	{
		delay_ms(10);//去抖动 
		trace_up=0;
		if(TRACE1==1)return 1;
		else if(TRACE2==1)return 2;
		else if(TRACE3==1)return 3;
		else if(TRACE4==1)return 4;
	}
	else if(TRACE1==0&&TRACE2==0&&TRACE3==0&&TRACE4==0)trace_up=1;
		return 0;// 无按键按下
}
/**
 * @brief: 循迹执行模块（矩形巡线），根据检测到的传感器信号做出相应动作
 * @param: NONE
 * @retval：NONE
 * @others: 由于此项目主要用来练习编程能力，so此处只编写了矩形巡线
						并没有对其他线型情况以及各种算法做深入探究
**/
void TRACE_Implement(void)
{
		extern u16 GEAR;
		u8 Edge_Flag=0;//矩形90度角检测，检测到后此标志位置1
		u8 trace_scan; 
		trace_scan=TRACE_Scan(0);//得到压线的传感器的编号		
	  if(trace_scan)
		{						   
			switch(trace_scan)
			{				 
				case TRACE1_SCAN:	//检测到最左侧传感器
					left_move_2(GEAR+2);//以较大速度左转
					Edge_Flag	=	1;//90度标志位置1
					delay_ms(150);//转向90度结束
					left_move(GEAR);//小左转
					break;
				case TRACE2_SCAN: //检测到最右侧传感器
					right_move_2(GEAR+2);
					Edge_Flag	=	1;
					delay_ms(150); 
					right_move(GEAR);
					LED0=!LED0;
					break;
				case TRACE3_SCAN:	//检测到中间左侧传感器
					if(Edge_Flag==1)//说明刚刚经过90度弯
					{
						right_move(GEAR);//中间左侧传感器压线时向右转
						Edge_Flag=0;
					}
					else
					{
						left_move(GEAR);//没有经过90度弯，中间左侧传感器压线时向左转
					}
					LED1=!LED1;
					break;
				case TRACE4_SCAN:	//检测到中间右侧传感器
					if(Edge_Flag==1)
					{
						left_move(GEAR);
						Edge_Flag=0;
					}
					else
					{
						right_move(GEAR);
					}
					LED0=!LED0;
					LED1=!LED1;
					break;
			}
		}else delay_ms(10); 
}
