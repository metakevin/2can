/******************************************************************************
* File:              bufferpool.h
* Author:            Kevin Day
* Date:              December, 2004
* Description:       
*                    
*                    
* Copyright (c) 2004 Kevin Day
* All rights reserved.
*******************************************************************************/

#ifndef BUFFERPOOL_H
#define BUFFERPOOL_H

void bufferpool_init();

/* copied from bget.h */
void	brel	    (void *buf);
void   *bget	    (u8 size);

#ifdef EMBEDDED

#define BUFSZ   15
#define BUFF_MAGIC_INUSE    0xA9
#define BUFF_MAGIC_FREE     0x42
typedef struct {
    u8  buf[BUFSZ];
    u8  flags;
} buff_t;

void bufferpool_release(u8 *buf);

u8 *bufferpool_request(u8 size);
#endif


#endif /* !BUFFERPOOL_H */
