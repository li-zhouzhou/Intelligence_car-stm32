/****************************************************************************
 * @file: remote_ctr.c
 * @author: jiaolu
 * @version: ultimate		 
 * @date:	2019/08/29
 * @brief: 本文件包含了智能小车遥控模式的操作函数，实现了小车在遥控模式下的
					 前进、后退、转向、停车
 * @History: // 修改历史记录列表，每条修改记录包括修改日期、修改者及内容简述
		1.  date:
			author:
		    modification:
		2. ...
****************************************************************************/

#include "car_control.h"
#include "usart.h"	

/*Remote ctr-----------------------------------------------------------------*/
/**
 * @brief: 实现了小车在遥控模式下的前进、后退、转向、停车
 * @param: NONE
 * @retval：NONE
 * @others: 对摇杆返回参数进行了一定的算法优化
**/
void Remote_Ctr(void)
{
	static int pss_data[2];//摇杆返回值数组，用于记录左右摇杆的返回值
	static int pulse_run;//驱动小车前进的pulse值
	static int turn_rate;//转向调整速率
	pss_data[0]=PS2_AnologData(PSS_LY);
	pss_data[1]=PS2_AnologData(PSS_RX);
	if(pss_data[0]<64)  //高速前进，利用一侧减速进行转弯
	{
		pulse_run=(int)(3.91*pss_data[0]);
		drive_pulse(pulse_run);
		if(pss_data[1]<128)
		{
			turn_rate=(int)(1-pss_data[1]/128);
			turn_pulse(pulse_run*(1+turn_rate),pulse_run);  //左侧减速		
		}
		else
		{
			turn_rate=(int)((pss_data[1]+1)/128-1);
			turn_pulse(pulse_run,pulse_run*(1+turn_rate));  //右侧减速
		}
	}
	else if(pss_data[0]<128)  //低速前进，利用一侧加速进行转弯
	{
		pulse_run=(int)(3.91*pss_data[0]);
		drive_pulse(pulse_run);
		if(pss_data[1]<128)		
		{
			turn_rate=(int)(1-pss_data[1]/128);
			turn_pulse(pulse_run,pulse_run*(1-turn_rate));		//右侧加速		
		}
		else
		{
			turn_rate=(int)((pss_data[1]+1)/128-1);
			turn_pulse(pulse_run*(1-turn_rate),pulse_run);		//左侧加速
		}
	}
	else if(pss_data[0]<192)		//低速后退，利用一侧加速进行转弯
	{
		pulse_run=(int)(500-3.91*(pss_data[0]-128));
		reverse_pulse(pulse_run);
	}
	else  //高速后退，利用一侧减速进行转弯
	{
		pulse_run=(int)(500-3.91*(pss_data[0]-128));
		reverse_pulse(pulse_run);
	}
}





