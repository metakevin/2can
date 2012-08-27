/******************************************************************************
* File:              platform.h
* Author:            Kevin Day
* Date:              December, 2004
* Description:       
*                    
*                    
* Copyright (c) 2004 Kevin Day
* All rights reserved.
*******************************************************************************/

#ifndef PLATFORM_H
#define PLATFORM_H
#include "types.h"

#define CPU_FREQ 16000000UL
#define UART_BAUD_RATE 115200L

#define F_CPU CPU_FREQ
#include <util/delay.h>
/* at 16MHz, 16 instructions per microsecond are executed.
 * Each iteration is 4 instructions.
 * So count should be 16/4 * us */
#define delay_us(x) _delay_loop_2(4*x)

void debug_led(u8 led, u8 val);

#endif /* !PLATFORM_H */
