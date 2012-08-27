/******************************************************************************
* File:              avrsys.h
* Author:            Kevin Day
* Date:              December, 2004
* Description:       
*                    
*                    
* Copyright (c) 2004 Kevin Day
* All rights reserved.
*******************************************************************************/

#ifndef AVRSYS_H
#define AVRSYS_H

#include <avr/interrupt.h>
#include "types.h"

#define DEBUG(str)

static inline u8 get_interrupt_flag()
{
    u8 i_bit = 1;
    
    __asm__ volatile ("brbs 7, 1f\n"
            "andi %0, 0\n"
            "1:" : "+a" (i_bit));
	return i_bit;	
}

/******************************************************************************
* disable_interrupts
*        Disable all interrupts by clearing the I bit in SREG.
*        If interrupts were previously enabled, return 1.  Else 0.
*******************************************************************************/
static inline u8 disable_interrupts()
{
//    uint8_t i_bit = inb(SREG) & 0x80;
//    cli();

//    return i_bit;

    u8 i_bit = get_interrupt_flag();

    cli();

    return i_bit;
}   


static inline void restore_flags(u8 i_bit)
{
    if (i_bit)
    {
        // was enabled; re-enable
        sei();
    }
    // else leave disabled
}


#endif /* !AVRSYS_H */
