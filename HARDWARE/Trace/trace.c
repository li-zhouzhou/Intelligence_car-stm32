/****************************************************************************
 * @file: trace.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: ���ļ�����������С��Ѳ��ģ��ĸ��ֹ��ܺ���������IO�ڳ�ʼ����Ѳ��
					 ����������ɨ�衢Ѳ��ִ��
 * @History: // �޸���ʷ��¼�б�ÿ���޸ļ�¼�����޸����ڡ��޸��߼����ݼ���
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
 * @brief: ѭ������������IO�ڳ�ʼ��
 * @param: NONE
 * @retval��NONE
 * @others: ���ѭ��ģ�����濪��ָʾ�Ƴ�����ԭ����ѡ����IO�ڣ�����PC0��PC2
					  ����PF2��PF4�ͺ�����why��
**/
void Trace_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//ʹ��GPIOFʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;       //����
	GPIO_Init(GPIOF,&GPIO_InitStructure);           
	
}
/*Trace scan&&implement-------------------------------------------------------*/
/**
 * @brief: ѭ��ģ��ɨ�躯��������STM32����ʵ��
 * @param: mode��==1��ʾ֧����������==0��ʾ��֧��������
 * @retval: 4��ѭ��ģ��ı�ţ�����1��ʾ1�Ŵ�����ѹ��
 * @others: NONE
**/

u8 TRACE_Scan(u8 mode)
{	 
	static u8 trace_up=1;//�������ɿ���־
	if(mode)trace_up=1;  //֧������		  
	if(trace_up&&(TRACE1==1||TRACE2==1||TRACE3==1||TRACE4==1))
	{
		delay_ms(10);//ȥ���� 
		trace_up=0;
		if(TRACE1==1)return 1;
		else if(TRACE2==1)return 2;
		else if(TRACE3==1)return 3;
		else if(TRACE4==1)return 4;
	}
	else if(TRACE1==0&&TRACE2==0&&TRACE3==0&&TRACE4==0)trace_up=1;
		return 0;// �ް�������
}
/**
 * @brief: ѭ��ִ��ģ�飨����Ѳ�ߣ������ݼ�⵽�Ĵ������ź�������Ӧ����
 * @param: NONE
 * @retval��NONE
 * @others: ���ڴ���Ŀ��Ҫ������ϰ���������so�˴�ֻ��д�˾���Ѳ��
						��û�ж�������������Լ������㷨������̽��
**/
void TRACE_Implement(void)
{
		extern u16 GEAR;
		u8 Edge_Flag=0;//����90�ȽǼ�⣬��⵽��˱�־λ��1
		u8 trace_scan; 
		trace_scan=TRACE_Scan(0);//�õ�ѹ�ߵĴ������ı��		
	  if(trace_scan)
		{						   
			switch(trace_scan)
			{				 
				case TRACE1_SCAN:	//��⵽����ഫ����
					left_move_2(GEAR+2);//�Խϴ��ٶ���ת
					Edge_Flag	=	1;//90�ȱ�־λ��1
					delay_ms(150);//ת��90�Ƚ���
					left_move(GEAR);//С��ת
					break;
				case TRACE2_SCAN: //��⵽���Ҳഫ����
					right_move_2(GEAR+2);
					Edge_Flag	=	1;
					delay_ms(150); 
					right_move(GEAR);
					LED0=!LED0;
					break;
				case TRACE3_SCAN:	//��⵽�м���ഫ����
					if(Edge_Flag==1)//˵���ով���90����
					{
						right_move(GEAR);//�м���ഫ����ѹ��ʱ����ת
						Edge_Flag=0;
					}
					else
					{
						left_move(GEAR);//û�о���90���䣬�м���ഫ����ѹ��ʱ����ת
					}
					LED1=!LED1;
					break;
				case TRACE4_SCAN:	//��⵽�м��Ҳഫ����
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
