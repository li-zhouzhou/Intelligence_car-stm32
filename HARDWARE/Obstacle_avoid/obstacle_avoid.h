#ifndef _OBSTACLE_AVOID
#define _OBSTACLE_AVOID

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/*Obs_avoid init----------------------------------------------------*/
void TIM4_PWM_Init(u16 arr,u16 psc);
void HCSR04_Init(void);

/*Distance measure--------------------------------------------------*/
void HCSR04_StartMeasure(void);
float HCSR04_GetDistance(u32 count);

/*Obs_avoid implement-----------------------------------------------*/
void Obstacle_avoid(void);

#endif


