/* Includes ------------------------------------------------------------------*/
#ifndef _TRACE_H
#define _TRACE_H
#include "sys.h"

//下面的方式是通过直接操作库函数方式读取IO
#define TRACE1 	GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_0) //最左
#define TRACE2 	GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_6)	//最右
#define TRACE3 	GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_4) //中左
#define TRACE4 	GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_2)	//中右

#define TRACE1_SCAN 	1
#define TRACE2_SCAN	  2
#define TRACE3_SCAN	  3
#define TRACE4_SCAN   4

/*Trace init-----------------------------------------------------------------*/
void Trace_Init(void);

/*Trace scan&&implement------------------------------------------------------*/
u8 TRACE_Scan(u8);  	
void TRACE_Implement(void);
#endif

