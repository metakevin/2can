/******************************************************************************
* File:              platform.h
* Author:            Kevin Day
* Date:              December, 2004
* Description:       
*                    
*                    
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
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
