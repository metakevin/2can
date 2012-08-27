/******************************************************************************
* File:              spimaster.h
* Author:            Kevin Day
* Date:              December, 2008
* Description:       
*                    
*                    
* Copyright (c) 2008 Kevin Day
* All rights reserved.
*******************************************************************************/

#ifndef SPIMASTER_H
#define SPIMASTER_H

typedef enum {
    SLAVE_NONE,
    SLAVE_2515,
} spi_slave_id_t;

void spimaster_init();

void spi_send(spi_slave_id_t sid, u16 len, u8 *data);       
void spi_send_async(spi_slave_id_t sid, u16 len, u8 *data, u8 *status);
void spi_start(spi_slave_id_t sid);
void spi_write(u8 len, u8 *data);
void spi_read(u8 len, u8 *data);
void spi_finish();    
#endif /* !SPIMASTER_H */
