/****************************************************************************
 * @brief: 本文件包含了智能小车蓝牙模式的各种功能文件，包括蓝牙模式初始化、
					 蓝牙的控制函数--小车的前进、后退、转向、停车
 * @History: // 修改历史记录列表，每条修改记录包括修改日期、修改者及内容简述
****************************************************************************/

#include "car_control.h"
#include "usart.h"
#include "led.h"

/*Bluetooth init------------------------------------------------------------*/
/**
 * @brief: 蓝牙模式初始化函数，初始化了串口2
**/
void Bluetooth_Ctr_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStrue;  
    NVIC_InitTypeDef NVIC_InitStrue;  
      
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//GPIOA端口使能  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//串口端口使能  
       
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART1); //GPIOA2复用为USART2
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART1); //GPIOA3复用为USART2
		
		//USART2端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
      
    USART_InitStrue.USART_BaudRate=115200;  
    USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None;  
    USART_InitStrue.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;  
    USART_InitStrue.USART_Parity=USART_Parity_No;  
    USART_InitStrue.USART_StopBits=USART_StopBits_1;  
    USART_InitStrue.USART_WordLength=USART_WordLength_8b;  
      
    USART_Init(USART2,&USART_InitStrue);
      
    USART_Cmd(USART2,ENABLE);//使能串口2 
      
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//开启接收中断  
      
    NVIC_InitStrue.NVIC_IRQChannel=USART2_IRQn;  
    NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStrue.NVIC_IRQChannelSubPriority=0;  
    NVIC_Init(&NVIC_InitStrue);  
}


/**
 * @brief: 串口2中断服务函数
 * @param: NONE
 * @retval：NONE
 * @others: 与正点原子例程中串口1中断服务函数无差别
**/

void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
}


/*Bluetooth ctr-----------------------------------------------------------------*/
/**
 * @brief: 蓝牙的控制函数--小车的前进、后退、转向、停车
 * @param: NONE
 * @retval：NONE
 * @others: 打印字符是5，但是ASCII码却不是5而是0x35，大坑出现
**/
void Bluetooth_Ctr(void)
{
		extern u16 GEAR;
		if(USART_RX_BUF[0]==0x31)	//这里是个大坑，打印字符是5，但是ASCII码却不是5而是0x35。折腾了一下午
		{
			LED0=!LED0;
			drive(GEAR);
		}
		else if(USART_RX_BUF[0]==0x32)
		{
			LED1=!LED1;
			reverse(GEAR);
		}
		else if(USART_RX_BUF[0]==0x33)
		{
			LED0=!LED0;
			left_move(GEAR);
		}
		else if(USART_RX_BUF[0]==0x34)
		{
			LED1=!LED1;
			right_move(GEAR);
		}
		else if(USART_RX_BUF[0]==0x30)
		{
			LED0=0;
			LED1=0;
			stop();
		}
}

