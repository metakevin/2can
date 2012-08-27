/******************************************************************************
* File:              persist.h
* Author:            Kevin Day
* Date:              April, 2005
* Description:       
*                    
*                    
* Copyright (c) 2005 Kevin Day
* All rights reserved.
*******************************************************************************/

#ifndef PERSIST_H
#define PERSIST_H

#include "types.h"
#include "eeprom.h"

/* Location in EEPROM of start of persistent data */
#define PERSIST_DATA_OFFSET 0

/* The default value for all data is zero.
 * keys not present in eeprom will read as zero.
 * a corrupted eeprom will be erased, and all keys read as zero */

typedef enum {
    PDATA_LAST_MODE,
    PDATA_MAP_UNITS,
    PDATA_ATMOSPHERIC,
    PDATA_ATMOSPHERIC_HIGH8,
    PDATA_IAT_UNITS,
    PDATA_MAP_SENSOR_INDEX,
    PDATA_EGT_UNITS,
    PDATA_ACTIVE_MODES_1,
    PDATA_ACTIVE_MODES_1H,
    PDATA_ACTIVE_MODES_2,
    PDATA_ACTIVE_MODES_2H,
    PDATA_ACTIVE_MODES_3,
    PDATA_ACTIVE_MODES_3H,
    PDATA_OILPRES_UNITS,
    PDATA_CHECKSUM              /* this must be the last entry */
} persist_data_key_t;

void init_persist_data();

static inline u8 load_persist_data(persist_data_key_t key)
{
    return read_eeprom(PERSIST_DATA_OFFSET+key);
}

/* Note: key must be the lesser cardinal value of two enums.
 *       The value in the EEPROM is in little endian format */
static inline u16 load_persist_data_16(persist_data_key_t key)
{
    return  (u16)read_eeprom(PERSIST_DATA_OFFSET+key) |
           ((u16)read_eeprom(PERSIST_DATA_OFFSET+key+1)<<8);
}
    

void save_persist_data(persist_data_key_t key, u8 value);
    
void save_persist_data_16(persist_data_key_t key, u16 value);

void erase_persist_data();

#endif /* !PERSIST_H */
