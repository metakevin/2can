/******************************************************************************
* File:              avrcan.c
* Author:            Kevin Day
* Date:              December, 2008
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
* Copyright (c) 2008 Kevin Day
* All rights reserved.
*******************************************************************************/

#include <string.h>
#include "platform.h"
#include "comms_generic.h"
#include "tasks.h"
#include "avrsys.h"
#include "bufferpool.h"
#include "avrcan.h"
#include "timers.h"
#include "mcp2515.h"

#define CANPORT  PORTD
#define CANDIR   DDRD
#define TXCANPIN 5
#define RXCANPIN 6

#define NUM_MESSAGE_OBJECTS 15
static void clear_current_message_object()
{
    u8 volatile *r;
    for(r=&CANSTMOB; r<&CANSTML; r++)
        *r = 0;
}
#if 0
static void clear_all_message_objects()
{
    u8 page;
    for(page=0; page<NUM_MESSAGE_OBJECTS; page++)
    {
        CANPAGE = (page<<4);
        clear_current_message_object();
    }
}
#endif


/* FIXME: check CANEN1/CANEN2 here? */
static u8 find_free_message_object()
{
    u8 page;
    for(page=0; page<NUM_MESSAGE_OBJECTS; page++)
    {
        CANPAGE = (page<<4);
        if (!(CANCDMOB & ((1<<CONMOB1)|(1<<CONMOB0))))
        {
            /* Disable mode.  I.e. available. */
            return page;
        }
    }
    return 0xFF;
}

static void config_for_rx()
{
    /* Enable reception */
    CANCDMOB = ((1<<CONMOB1));     
}



#define RX_MOB_MAX 8
void canbus_init(u8 canbt1)
{
    /* Turn pullups on */
    CANDIR  &= ~((1<<TXCANPIN) | (1<<RXCANPIN));
    CANPORT |=  ((1<<TXCANPIN) | (1<<RXCANPIN));
    
    /* Initialize CAN unit */
    CANGCON = (1<<SWRES);

    /* Mark 8 MObs for RX */
    u8 page;
    for(page=0; page<RX_MOB_MAX; page++)
    {
        CANPAGE = (page<<4);
        clear_current_message_object();
        config_for_rx();
    }
    /* And 7 for TX (empty for now) */
    for(; page<15; page++)
    {
        CANPAGE = (page<<4);
        clear_current_message_object();
    }

    /* Configure bit timings for 250kbit/sec at 16 MHz */
    //CANBT1 = 0x06;  /* 2 for 500kbit/sec */
	CANBT1 = canbt1;
    CANBT2 = 0x0C;
    CANBT3 = 0x37;


    /* Enable all interrupts except timer */
    CANGIE |= 0xFE;

    /* Enable all MOB interrupts */
    CANIE1 = 0x7F;
    CANIE2 = 0xFF;

    /* Enable CAN unit */
    CANGCON |= (1<<ENASTB);

}

u8 send_can_msg(can_addr_t *a, u8 plen, u8 *payload)
{
    if (find_free_message_object() != 0xFF)
    {
        u8 i;
        for(i=0; i<plen; i++)
        {
            CANMSG = payload[i];
        }        
		if (!a->ver_20b)
		{
			CANIDT1 = a->id >> 3;
			CANIDT2 = a->id << 5;
			CANIDT4 = a->rtr ? _BV(RTRTAG) : 0;
			
			/* Enable TX, standard frame, set length */
			CANCDMOB = ((1<<CONMOB0)|plen);
		}			
		else
		{
			CANIDT1 = a->id >> 21;
			CANIDT2 = a->id >> 13;
			CANIDT3 = a->id >> 5;
			CANIDT4 = ((a->id << 3) | (a->rtr ? _BV(RTRTAG) : 0));
			
		    /* Enable TX, extended frame, set length */
			CANCDMOB = ((1<<CONMOB0)|(1<<IDE)|plen);
		}			
        return 0;
    }
    return 1;
}


task_t can_taskinfo;
static u8 can_mailbox_buf[80];
static u8 can_task();

u16 sidfilter[SIDFILTER_BLOCK_SIZE*SIDFILTER_NUM_BLOCKS];

task_t *can_task_create()
{
	// 250kbps default
    canbus_init(0x06);

    return setup_task(&can_taskinfo, TASK_ID_CAN, can_task,  
                      can_mailbox_buf, sizeof(can_mailbox_buf));
}


/* These bits are XOR'd with the upper nibble of the match result.
 * So for example if the table is empty, all matches will return 0,
 * and if filter_sense is SIDFILTER_RELAY for that interface, then all matches will
 * be 0 ^ SIDFILTER_RELAY.  Likewise if there is a match for a
 * specific ID/DLC and the relay sense bit is set for that
 * interface, the frames matching will not be relayed but all others will. */
static u8 filter_sense;

u8 can_id_match(can_addr_t *addr, u8 interface) 
{
	u8 i;
	u8 r = 0; /* not filtered */
	for(i=0; i<sizeof(sidfilter)/sizeof(sidfilter[0]); i++)
	{
		if ((sidfilter[i]&SIDFILTER_VALID_MASK) == 0)
		{
			/* reached end of list */
			break;			
		}
		// NOTE: this only works with 11 bit IDs
		else if ((sidfilter[i]&SIDFILTER_ID_MASK) == addr->id)
		{
			r = (sidfilter[i]>>12);
			break;
		}
	}
	r ^= filter_sense;
	// now adjust for interface
	r >>= 2*interface; 
	return r;
}

canstats_t canstats;
u8 can_int_err;

void send_canstats(u8 clear_on_send)
{
	u8 flags = disable_interrupts();
	u8 r = send_msg_buffered(0, TASK_ID_CAN<<4|CAN_STATS, sizeof(canstats), (u8*)&canstats);
	if (r == 0 && clear_on_send)
	{
		memset((void*)&canstats, 0, sizeof(canstats));		
	}
	restore_flags(flags);
}


u8 can_task()
{
    u8 ret = 0;
    u8 code, payload_len;
    
    if (mailbox_head(&can_taskinfo.mailbox, &code, &payload_len))
    {
        switch(code)
        {
        case CAN_SEND_RAW_MSG:
            if (payload_len < sizeof(can_addr_t))
            {
                /* error */
                send_msg(0, 
                    TASK_ID_CAN<<4|CAN_ERROR,
                    0, NULL);
            }
            else
            {
				// FIXME: timerinterval_t not used here
                u8 payload[sizeof(timerinterval_t) + sizeof(can_addr_t) + CAN_MAX_PAYLOAD];
                mailbox_copy_payload(&can_taskinfo.mailbox, payload, sizeof(payload), 0);
                send_can_msg((can_addr_t *)payload, payload_len - sizeof(can_addr_t), &payload[sizeof(can_addr_t)]);
            }
            break;
        case CAN_MSG_RX:
            {
                /* Received message from ISR */
                u8 payload[sizeof(timerinterval_t) + sizeof(can_addr_t) + CAN_MAX_PAYLOAD];
                mailbox_copy_payload(&can_taskinfo.mailbox, payload, sizeof(payload), 0);
                send_msg(2, 
                         TASK_ID_CAN<<4|COMMS_MSG_CAN_RAW,
                         payload_len, payload);    
                break;
            }
		case CAN_SET_SID_FILTER:
			if (payload_len == SIDFILTER_BLOCK_SIZE*sizeof(u16)+1)
			{
				u8 block;
				mailbox_copy_payload(&can_taskinfo.mailbox, &block, 1, 0);
				if (block < SIDFILTER_NUM_BLOCKS) 
				{
					mailbox_copy_payload(&can_taskinfo.mailbox, 
										 (u8 *)&sidfilter[SIDFILTER_BLOCK_SIZE*block], 
										 SIDFILTER_BLOCK_SIZE*sizeof(u16), 1);
					
				}
			}
			else if (payload_len == 1)
			{
				mailbox_copy_payload(&can_taskinfo.mailbox, &filter_sense, 1, 0);
			}
			break;			
		case CAN_SET_BT:
			{
				u8 canbt[3];
				mailbox_copy_payload(&can_taskinfo.mailbox, canbt, sizeof(canbt), 0);
								
				if (payload_len >= 1)
				{
				    /* Disable CAN unit */
					CANGCON = 0;
					/* Reinitialize */				
					canbus_init(canbt[0]);
					canbt[0] = CANBT1;
					canbt[1] = CANBT2;
					canbt[2] = CANBT3;
				}
				send_msg(2, TASK_ID_CAN<<4|CAN_SET_BT, sizeof(canbt), canbt);
				break;								
			}
		case CAN_STATS:
		{
			u8 clear = 0;
			if (payload_len >= 1)
			{
				mailbox_copy_payload(&can_taskinfo.mailbox, &clear, 1, 0);
			}				
		    send_canstats(clear);
			break;
		}								
		default:
            break;
        }
        mailbox_advance(&can_taskinfo.mailbox);
        ret = 1;
    }

    if(can_int_err)
	{
		send_canstats(1);
		can_int_err = 0;
	}

    return ret;
}                

    



SIGNAL(SIG_CAN_INTERRUPT1)
{
	u8 demux = 0;
	u8 gen = CANGIT;
	if (gen)
	{
		if (gen & _BV(BOFFIT))
		{
			++canstats.buserr;			
		}
		if (gen & _BV(OVRTIM))
		{
			++canstats.overtime;
		}
		if (gen & _BV(SERG))
		{
			++canstats.stufferr;			
		}
		if (gen & _BV(CERG))
		{
			++canstats.crcerr;
		}
		if (gen & _BV(FERG))
		{		
			++canstats.frameerr;
		}
		if (gen & _BV(AERG))
		{
			++canstats.ackerr;
		}		
		CANGIT = gen;		
		++demux;
	}
	
    u8 pagesave = CANPAGE;
    if ((CANHPMOB & 0xF0) != 0xF0)
    {
		u8 page = CANHPMOB;
        CANPAGE = page;
		u8 stmob = CANSTMOB;
        if (stmob & _BV(RXOK))
        {
            u8 canbuf[sizeof(timerinterval_t) + sizeof(can_addr_t) + CAN_MAX_PAYLOAD];
            can_addr_t *addr = (can_addr_t *)&canbuf[sizeof(timerinterval_t)];
			u8 ctrl = CANCDMOB;
			u8 len = (ctrl&0xF);
			if (ctrl&_BV(IDE))
			{
				/* extended address */
				addr->ver_20b = 1;
				u8 idt4 = CANIDT4;
				addr->id  = (((u32)CANIDT1<<21)|((u32)CANIDT2<<13)|((u32)CANIDT3<<5)|(idt4>>3));				
				addr->rtr = (idt4&_BV(RTRTAG)) ? 1 : 0;								
			}
			else
			{
				/* standard address */
				addr->ver_20b = 0;
				addr->id  = (((u32)CANIDT1<<3)|(CANIDT2>>5));
				addr->rtr = (CANIDT4&_BV(RTRTAG)) ? 1 : 0;								
			}
			u8 match = can_id_match(addr, 0);
			// read payload unless this packet is going to be dropped
			if (!(match&SIDFILTER_NOHOST) || (match&SIDFILTER_RELAY))
			{
				addr->unused = 0;
				u8 i;
				for(i=0; i<len; i++)
				{
					canbuf[sizeof(timerinterval_t)+sizeof(can_addr_t)+i] = CANMSG;
				}								
						
				if (!(match&SIDFILTER_NOHOST))
				{
					timerinterval_t *tm = (timerinterval_t*)canbuf;
					*tm = readtime();
            
					mailbox_deliver(&can_taskinfo.mailbox, 
									CAN_MSG_RX, sizeof(timerinterval_t)+sizeof(can_addr_t)+len, canbuf);
				}
				else 
				{
					++canstats.rx_filtered;				
				}		
				if (match&SIDFILTER_RELAY)
				{
					mailbox_deliver(&mcp_taskinfo.mailbox,
								     CAN_SEND_RAW_MSG, sizeof(can_addr_t)+len, &canbuf[sizeof(timerinterval_t)]);				
					++canstats.rx_relayed;
				}				
			}				
			++demux;
			++canstats.mob_rxok;
        }
        if (stmob & _BV(TXOK))
        {
			++demux;
			++canstats.mob_txok;
        }
		if (stmob & _BV(DLCW))
		{
			++demux;
			++canstats.mob_dlcw;
		}
		if (stmob & _BV(BERR))
		{
			++demux;
			++canstats.mob_biterr;
		}
		if (stmob & _BV(SERR))			
		{
			++demux;
			++canstats.mob_serr;
		}
		if (stmob & _BV(CERR))
		{
			++demux;
			++canstats.mob_cerr;
		}			
		if (stmob & _BV(FERR))	
		{
			++demux;
			++canstats.mob_ferr;
		}
		if (stmob & _BV(AERR))
		{
			++demux;
			++canstats.mob_aerr;
		}									

		/* Note: clearing MOB and discarding message even if it was an error (e.g. AERR, not acknowledged).
		 */
        clear_current_message_object();
		if (page < RX_MOB_MAX)
		{
           config_for_rx();
		}				
		
		if (!((stmob&_BV(TXOK))|(stmob&_BV(RXOK))))
		{
			/* error interrupt - need to explicitly disable this MOB */
			if (page < 8)
			{
				CANEN2 &= ~_BV(page);
			}
			else
			{
				CANEN1 &= ~_BV(page-8);
			}
			can_int_err++;
		}														
						
		/* Acknowledge interrupt */	
		CANSTMOB = 0;						
    }
    CANPAGE = pagesave;
	
	if (demux == 0)
	{
		canstats.unhandled++;
		can_int_err++;	
	}
}


