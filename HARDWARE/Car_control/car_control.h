#ifndef __CAR_CONT
#define __CAR_CONT

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "trace.h"
#include "obstacle_avoid.h"
#include "remote_ctr.h"
#include "bluetooth_ctr.h"
#include "ps2.h"

/*Car config-----------------------------------------------------------------*/
void CAR_Init(void);
void TIM3_PWM_Init(u16 arr,u16 psc);

/*Car driver-----------------------------------------------------------------*/
void drive(u16 gear);
void reverse(u16 gear);
void stop(void);
void left_move(u16 gear_change);
void left_move_2(u16 gear_change);
void right_move(u16 gear_change);
void right_move_2(u16 gear_change);
void drive_pulse(int pulse);
void reverse_pulse(int pulse);
void turn_pulse(int pulse1,int pulse2);

/*Mode config-----------------------------------------------------------------*/
void Mode_select(u8 key);
void Mode_run(u8 mode);
void Mode_Scan(void);

#endif

