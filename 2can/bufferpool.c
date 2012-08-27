/******************************************************************************
* File:              bufferpool.c
* Author:            Kevin Day
* Date:              December, 2004
* Description:       
*                    
*                    
* Copyright (c) 2004 Kevin Day
* All rights reserved.
*******************************************************************************/

#include "types.h"
#include "bufferpool.h"
#include "comms_generic.h"

extern u8 *_bufferpool_start;
extern u8 *_bufferpool_end;

static buff_t *bufferpool;
static u8 num_buffs;

void bufferpool_init()
{
    u8 i;

    bufferpool = (buff_t *)&_bufferpool_start;

    for(i=0; ; )
    {
        bufferpool[i].flags = BUFF_MAGIC_FREE;
        ++num_buffs;
        ++i;
        if ((u8*)&bufferpool[i+1] > (u8*)&_bufferpool_end)
        {
            break;
        }
    }
}

u8 *bufferpool_request(u8 size)
{
    
    //return (u8 *)&_bufferpool_start;
    
    if (size <= BUFSZ)
    {
        u8 i;
        for(i=0; i<num_buffs; i++)
        {
            if (bufferpool[i].flags == BUFF_MAGIC_FREE)
            {
                bufferpool[i].flags = BUFF_MAGIC_INUSE;            
                return (u8*)&bufferpool[i].buf;
            }
        }
    }
    return NULL;
}

void bufferpool_release(u8 *buf)
{
    buff_t *bp = (buff_t *)buf;
    bp->flags = BUFF_MAGIC_FREE;
}




