/****************************************************************************
 * @file: remote_ctr.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: ���ļ�����������С��ң��ģʽ�Ĳ���������ʵ����С����ң��ģʽ�µ�
					 ǰ�������ˡ�ת��ͣ��
 * @History: // �޸���ʷ��¼�б�ÿ���޸ļ�¼�����޸����ڡ��޸��߼����ݼ���
		1.  date:
			author:
		    modification:
		2. ...
****************************************************************************/

#include "car_control.h"
#include "usart.h"	

/*Remote ctr-----------------------------------------------------------------*/
/**
 * @brief: ʵ����С����ң��ģʽ�µ�ǰ�������ˡ�ת��ͣ��
 * @param: NONE
 * @retval��NONE
 * @others: ��ҡ�˷��ز���������һ�����㷨�Ż�
**/
void Remote_Ctr(void)
{
	static int pss_data[2];//ҡ�˷���ֵ���飬���ڼ�¼����ҡ�˵ķ���ֵ
	static int pulse_run;//����С��ǰ����pulseֵ
	static int turn_rate;//ת���������
	pss_data[0]=PS2_AnologData(PSS_LY);
	pss_data[1]=PS2_AnologData(PSS_RX);
	if(pss_data[0]<64)  //����ǰ��������һ����ٽ���ת��
	{
		pulse_run=(int)(3.91*pss_data[0]);
		drive_pulse(pulse_run);
		if(pss_data[1]<128)
		{
			turn_rate=(int)(1-pss_data[1]/128);
			turn_pulse(pulse_run*(1+turn_rate),pulse_run);  //������		
		}
		else
		{
			turn_rate=(int)((pss_data[1]+1)/128-1);
			turn_pulse(pulse_run,pulse_run*(1+turn_rate));  //�Ҳ����
		}
	}
	else if(pss_data[0]<128)  //����ǰ��������һ����ٽ���ת��
	{
		pulse_run=(int)(3.91*pss_data[0]);
		drive_pulse(pulse_run);
		if(pss_data[1]<128)		
		{
			turn_rate=(int)(1-pss_data[1]/128);
			turn_pulse(pulse_run,pulse_run*(1-turn_rate));		//�Ҳ����		
		}
		else
		{
			turn_rate=(int)((pss_data[1]+1)/128-1);
			turn_pulse(pulse_run*(1-turn_rate),pulse_run);		//������
		}
	}
	else if(pss_data[0]<192)		//���ٺ��ˣ�����һ����ٽ���ת��
	{
		pulse_run=(int)(500-3.91*(pss_data[0]-128));
		reverse_pulse(pulse_run);
	}
	else  //���ٺ��ˣ�����һ����ٽ���ת��
	{
		pulse_run=(int)(500-3.91*(pss_data[0]-128));
		reverse_pulse(pulse_run);
	}
}





