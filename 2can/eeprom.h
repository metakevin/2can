/******************************************************************************
* File:              eeprom.h
* Author:            Kevin Day
* Date:              April, 2005
* Description:       
*                    
*                    
* Copyright (c) 2005 Kevin Day
* All rights reserved.
*******************************************************************************/

#ifndef EEPROM_H
#define EEPROM_H

#include "types.h"

/* These are inline because it is assumed they will only be called
 * from one location. */

void write_eeprom(u16 address, u8 value);
u8   read_eeprom(u16 address);

#endif /* !EEPROM_H */
