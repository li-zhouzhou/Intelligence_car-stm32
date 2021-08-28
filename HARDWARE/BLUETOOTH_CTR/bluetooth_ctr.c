/****************************************************************************
 * @brief: ���ļ�����������С������ģʽ�ĸ��ֹ����ļ�����������ģʽ��ʼ����
					 �����Ŀ��ƺ���--С����ǰ�������ˡ�ת��ͣ��
 * @History: // �޸���ʷ��¼�б�ÿ���޸ļ�¼�����޸����ڡ��޸��߼����ݼ���
****************************************************************************/

#include "car_control.h"
#include "usart.h"
#include "led.h"

/*Bluetooth init------------------------------------------------------------*/
/**
 * @brief: ����ģʽ��ʼ����������ʼ���˴���2
**/
void Bluetooth_Ctr_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;  
    USART_InitTypeDef USART_InitStrue;  
    NVIC_InitTypeDef NVIC_InitStrue;  
      
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//GPIOA�˿�ʹ��  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//���ڶ˿�ʹ��  
       
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART1); //GPIOA2����ΪUSART2
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART1); //GPIOA3����ΪUSART2
		
		//USART2�˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2��GPIOA3
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2��PA3
      
    USART_InitStrue.USART_BaudRate=115200;  
    USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None;  
    USART_InitStrue.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;  
    USART_InitStrue.USART_Parity=USART_Parity_No;  
    USART_InitStrue.USART_StopBits=USART_StopBits_1;  
    USART_InitStrue.USART_WordLength=USART_WordLength_8b;  
      
    USART_Init(USART2,&USART_InitStrue);
      
    USART_Cmd(USART2,ENABLE);//ʹ�ܴ���2 
      
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//���������ж�  
      
    NVIC_InitStrue.NVIC_IRQChannel=USART2_IRQn;  
    NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=0;  
    NVIC_InitStrue.NVIC_IRQChannelSubPriority=0;  
    NVIC_Init(&NVIC_InitStrue);  
}


/**
 * @brief: ����2�жϷ�����
 * @param: NONE
 * @retval��NONE
 * @others: ������ԭ�������д���1�жϷ������޲��
**/

void USART2_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART2);//(USART2->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
  } 
}


/*Bluetooth ctr-----------------------------------------------------------------*/
/**
 * @brief: �����Ŀ��ƺ���--С����ǰ�������ˡ�ת��ͣ��
 * @param: NONE
 * @retval��NONE
 * @others: ��ӡ�ַ���5������ASCII��ȴ����5����0x35����ӳ���
**/
void Bluetooth_Ctr(void)
{
		extern u16 GEAR;
		if(USART_RX_BUF[0]==0x31)	//�����Ǹ���ӣ���ӡ�ַ���5������ASCII��ȴ����5����0x35��������һ����
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

