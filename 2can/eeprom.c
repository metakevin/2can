/******************************************************************************
* File:              eeprom.c
* Author:            Kevin Day
* Date:              April, 2005
* Description:       
*                    
*                    
* Copyright (c) 2005 Kevin Day
* All rights reserved.
*******************************************************************************/

#include <avr/io.h>
#include "types.h"
#include "eeprom.h"

void write_eeprom(u16 address, u8 value)
{
#if 0
    while (EECR & (1<<EEPE))
        ;
    
    EEAR = address;
    EEDR = value;
    
    EECR |= (1<<EEMPE);
    EECR |= (1<<EEPE);
#endif
}

u8 read_eeprom(u16 address)
{
#if 0
    while (EECR & (1<<EEPE))
        ;
    
    EEAR = address;
    
    EECR |= (1<<EERE);

    return EEDR;
#else
    return 0;
#endif
}

