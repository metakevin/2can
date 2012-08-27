/******************************************************************************
* File:              mcp2515.c
* Author:            Kevin Day
* Date:              December, 2011
* Description:       
*                    
*                    
* Copyright (c) 2011 Kevin Day
* All rights reserved.
*******************************************************************************/

#include "platform.h"
#include "comms_generic.h"
#include "tasks.h"
#include "avrsys.h"
#include "bufferpool.h"
#include "spimaster.h"
#include "avrcan.h"
#include "timers.h"

#define REG_CNF3    0x28
#define REG_CNF2    0x29
#define REG_CNF1    0x2A
#define REG_INT_EN  0x2B
#define REG_INT_FL  0x2C
#define REG_ERROR   0x2D

#define REG_TXB0    0x30
#define REG_TXB1    0x40
#define REG_TXB2    0x50
#define REG_RXB0    0x60
#define REG_RXB1    0x70

#define CMD_REGWRITE    0x02
#define CMD_REGREAD     0x03
#define CMD_RESET       0xC0
#define CMD_STATUS      0xA0
#define CMD_RXSTATUS    0xB0

#define CMD_RTS         0x80  /* 3 LSBs RTS status for TXB0-TXB2 */
#define     RTS_TXB0    0x01
#define     RTS_TXB1    0x02
#define     RTS_TXB2    0x04

#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define CANINTF  0x2C
#define CANINTE  0x2B
#define RX0IF    0    /* of CANINTF register */
#define BFPCTRL  0x0C

/* starting at RXB0SIDH */
typedef struct {
    u8 sid10_3;
    u8 sid2_0;  /* 7:5 sid 2:0 4: srr 3: IDE 1:0 EID 17:16 */
    u8 eid15_8;
    u8 eid7_0;
    u8 dlc; /* 6: RTR 3:0 length */
    u8 data[8];
} canrx_t;


/* Pinning:
 * B5: /RESET
 * C7: OSC
 */
#define MCP_PIN_RESET 5


void mcp_read(u8 addr, u8 len, u8 *buf)
{
    u8 cmd[] = {CMD_REGREAD, addr};
    spi_start(SLAVE_2515);
    spi_write(sizeof(cmd), cmd);
    spi_read(len, buf);
    spi_finish();
}

void mcp_write(u8 addr, u8 len, u8 *buf)
{
    u8 cmd[] = {CMD_REGWRITE, addr};
    spi_start(SLAVE_2515);
    spi_write(sizeof(cmd), cmd);
    spi_write(len, buf);
    spi_finish();
}



void mcp2515_init()
{
	u8 w[3];
	
	DDRB |= _BV(MCP_PIN_RESET);  
	PORTB &= ~_BV(MCP_PIN_RESET);
	delay_us(50);
	PORTB |= _BV(MCP_PIN_RESET);
	delay_us(50);	

#if 0 /* this puts RX interrupts on the main INT pin */	
	u8 inte = 1; /* enable RX0BF interrupt */
	mcp_write(CANINTE, 1, &inte);
#endif

    /* baud rate registers for 250kbps @ 16MHz (16TQ)
     * CNF1/BRGCON1: 0x01
     * CNF2/BRGCON2: 0xB8
     * CNF3/BRGCON3: 0x05 
     *
     * as per bit timing calculator from intrepid control systems 
	 *
	 * if CNF2/CNF3 are held constant, variable baud rates can be set by changing the BRP
	 * bits of CNF1.  Assume 16 TQ per bit (B8/05 in CON2/3):
	 * TQ duration = (bit time in ns)/16
	 * 1 cycle @ Fosc = 62.5 ns
	 * For 100 kbps:
	 *   TQ = (1,000,000,000/100,000)/16 = 625
	 *   625 = (2 x (BRP + 1))*62.5
	 *   BRP = 4
	 *
	 * Rearranged:
	 *   BRP = ((((1,000,000,000/bitrate)/16)/62.5)-2)/2
	 */
	w[0] = 0x05; /* cnf3 */
	w[1] = 0xb8;
	w[2] = 0x01; /* cnf1 */
	mcp_write(REG_CNF3, 3, w);
	
	/* Enable dedicated RX interrupt pins */
	w[0] = 0xF;
	mcp_write(BFPCTRL, 1, w);
	
	/* make interrupts inputs with pullups */
	DDRD &= ~0xF;
	PORTD |= 0xF;
	
	/* make INT2 edge sensitive */
	EICRA |= _BV(ISC21);
	
	/* unmask INT2 (RX0BF) */
	EIMSK |= _BV(2);
}

task_t mcp_taskinfo;
static u8 mcp_mailbox_buf[40];
static u8 mcp_task();

task_t *mcp_task_create()
{
	mcp2515_init();
    return setup_task(&mcp_taskinfo, TASK_ID_MCP, mcp_task,
                      mcp_mailbox_buf, sizeof(mcp_mailbox_buf));
}

u8 rx0_pending;

#define TXB0SIDH	0x31
#define TXB0DLC		0x35
#define TXB0D0		0x36
#define TXB0CTRL	0x30

void send_mcp_msg(can_addr_t *a, u8 plen, u8 *payload)
{
	if (a->ver_20b)
	{
		u8 addr[5];
		u16 sid = a->id>>18;		
		addr[0] = sid>>3;
		addr[1] = ((sid<<5)|(1<<3)|((a->id>>16)&3));
		addr[2] = a->id>>8;
		addr[3] = a->id;
		addr[4] = plen;
		mcp_write(TXB0SIDH, sizeof(addr), addr);	
	}
	else
	{
		u8 addr[2];
		addr[0] = a->id>>3;
		addr[1] = (a->id<<5);
		mcp_write(TXB0SIDH, sizeof(addr), addr);			
		mcp_write(TXB0DLC, 1, &plen);	
	}		
	if (plen > 0)
	{
		mcp_write(TXB0D0, plen, payload);		
	}
	u8 ctrl = (1<<3); /* TXREQ */
	mcp_write(TXB0CTRL, 1, &ctrl);		
}

#define RXB0EID8 0x63
#define RXB0DLC  0x65
#define RXB0D0   0x66
static u8 mcp_task()
{
    u8 ret = 0;
    u8 code, payload_len;
	
	while (rx0_pending > 0)
	{
//		u8 flags = disable_interrupts();
		u8 intf;
		mcp_read(CANINTF, 1, &intf);
		intf &= ~_BV(RX0IF);
		mcp_write(CANINTF, 1, &intf);
		--rx0_pending;

		u8 m[sizeof(timerinterval_t) + sizeof(can_addr_t) + CAN_MAX_PAYLOAD] = {0};
        can_addr_t *a = (can_addr_t *)&m[sizeof(timerinterval_t)];		
		u8 *payload = &m[sizeof(timerinterval_t)+sizeof(can_addr_t)];
		u8 plen;
		u8 sid[2];
		mcp_read(RXB0SIDH, sizeof(sid), sid);
		mcp_read(RXB0DLC, 1, &plen);
		if (sid[1]&(1<<3))
		{
			/* Extended address */
			u8 eid[2];
			mcp_read(RXB0EID8, sizeof(eid), eid);
			a->id = (((u32)sid[0]<<21) | (((u32)sid[1]>>5)<<18) | (((u32)sid[1]&3)<<16) | ((u32)eid[0]<<8) | eid[1]);
			a->ver_20b = 1;
			if (plen&(1<<6))
			{
				a->rtr = 1;
			}
			else
			{
				a->rtr = 0;
			}
		}
		else
		{
			a->id = ((sid[0]<<3) | (sid[1]>>5));			
			a->ver_20b = 0;
			if (sid[1]&(1<<4))
			{				
				a->rtr = 1;
			}
			else
			{
				a->rtr = 0;
			}							
		}
		//a->unused = 0;
		plen &= 0xF;
		
		u8 match = can_id_match(a, 1);
		if (!(match&SIDFILTER_NOHOST) || (match&SIDFILTER_RELAY))
		{
			/* possible optimization: burst read of N+1 bytes where N is the value of the first byte read */
			if (plen > 0)
			{
				mcp_read(RXB0D0, plen, payload);
			}

			if (!(match&SIDFILTER_NOHOST))
			{
				timerinterval_t *tm = (timerinterval_t*)m;
				*tm = readtime();				
				send_msg(2, TASK_ID_MCP<<4|COMMS_MSG_CAN_RAW, sizeof(timerinterval_t)+sizeof(can_addr_t) + plen, m);    					
			}			
			if (match&SIDFILTER_RELAY)
			{
				mailbox_deliver(&can_taskinfo.mailbox,
								    CAN_SEND_RAW_MSG, sizeof(can_addr_t)+plen, &m[sizeof(timerinterval_t)]);				
			}			
		}				
		ret = 1;
//		restore_flags(flags);
	}
    
    if (mailbox_head(&mcp_taskinfo.mailbox, &code, &payload_len))
    {
        switch(code)
        {
        case MCP_REG_READ:
            if (payload_len != 2)
            {
                send_msg(0, TASK_ID_MCP<<4|CAN_ERROR,0,NULL);
            }
            else
            {
                u8 p[2];
                u8 buf[16];
                mailbox_copy_payload(&mcp_taskinfo.mailbox,p,2,0);
                if (p[1] > sizeof(buf))
                {
                    p[1] = sizeof(buf);
                }
                mcp_read(p[0], p[1], buf);
                send_msg(0, TASK_ID_MCP<<4|MCP_REG_READ, p[1], buf);
            }
            break;                
        case MCP_REG_WRITE:
            if (payload_len < 2)
            {
                send_msg(0, TASK_ID_MCP<<4|CAN_ERROR,0,NULL);
            }
            else
            {
                u8 p[16];
                mailbox_copy_payload(&mcp_taskinfo.mailbox,p,sizeof(p),0);
                mcp_write(p[0], payload_len-1, p+1);
            }
            break;                
        case CAN_SEND_RAW_MSG:       
            if (payload_len < sizeof(can_addr_t))
            {
                /* error */
                send_msg(0, 
                    TASK_ID_MCP<<4|CAN_ERROR,
                    0, NULL);
            }
            else
            {
                u8 payload[sizeof(can_addr_t) + CAN_MAX_PAYLOAD];
                mailbox_copy_payload(&mcp_taskinfo.mailbox, payload, sizeof(payload), 0);
                send_mcp_msg((can_addr_t *)payload, payload_len - sizeof(can_addr_t), &payload[sizeof(can_addr_t)]);
            }
            break;
        default:
            break;
        }
        mailbox_advance(&mcp_taskinfo.mailbox);
        ret = 1;
    }

    return ret;
}                

/* /RX0BF 
 * */
SIGNAL(SIG_INTERRUPT2)
{
	rx0_pending++;		
	/* mask INT2 (RX0BF) */
//	EIMSK &= ~_BV(2);
}	
	

