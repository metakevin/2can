#include <string.h>
#include <avr/wdt.h>
#include "platform.h"
#include "comms_generic.h"
#include "tasks.h"
#include "avrsys.h"
#include "bufferpool.h"
#include "spimaster.h"

#define COMMS_MSG_HEADER    0x10
#define COMMS_MSG_PAYLOAD   0x11

u8 get_node_id()
{
    return 1;
}

static task_t   comms_taskinfo;
static u8       comms_mailbox_buf[128];
u8 comms_task();



/* pinning info for 2CAN:
 * Inputs: 
 *   DTR#  B7
 *   RTS#  E6
 * Outputs:
 *   RI#   E5
 *   DSR#  E4
 *   DCD#  E3 
 *   CTS#  E2
 *   CBUS2 RST
 *   CBUS3 G3  (GPIO)
 *   CBUS4 B6
 */

static inline void cfg_modem_control()
{
	DDRE &= ~_BV(6); // rts input
	DDRB &= ~_BV(7); // dtr input
	DDRE |= (_BV(2)|_BV(3)|_BV(4)|_BV(5));		
}

static inline void set_cts(u8 clear_to_send)
{
	if (clear_to_send)
	{	
		PORTE &= ~_BV(2); // inverted logic
	}
	else
	{
		PORTE |= _BV(2);
	}			
	//debug_led(1,clear_to_send);
}


/* called by main */
task_t* comms_task_create()
{
    /* Configure the baud rate. */
#if 1
    /* On the at90can128, the -1 that was in the baud rate expressions
     * (as used on the atmega88/168) had to be removed. 
     * On the at90can32, it needed to be added back.  ??? */
//	UBRR0L = (uint8_t)(CPU_FREQ/(UART_BAUD_RATE*16L) - 1);
//	UBRR0H = (CPU_FREQ/(UART_BAUD_RATE*16L) - 1) >> 8;
    UBRR0L = 103; // 9600
    UBRR0H = 0; 
    
    UCSR0A = 0x0;
	UCSR0B = _BV(TXEN0)|_BV(RXEN0);
   	UCSR0C = _BV(UCSZ01)|_BV(UCSZ00);
#else
    /* 1M baud with 16MHz osc */
    /* UBRR0 = 0
     * U2X0  = 1 */
    UBRR0L = 0;
    UBRR0H = 0;
//    UCSR0A = _BV(U2X0);
    UCSR0A = 0;
	UCSR0B = _BV(TXEN0)|_BV(RXEN0);
   	UCSR0C = _BV(UCSZ01)|_BV(UCSZ00);
#endif

    /* Enable RX interrupt */
    UCSR0B |= _BV(RXCIE0);

    cfg_modem_control();
 
    return setup_task(&comms_taskinfo, TASK_ID_COMMS, comms_task, 
            comms_mailbox_buf, sizeof(comms_mailbox_buf));
}


#define RCVBUFSZ 16 /* must be power of 2 */
static u8 rcvbuf[RCVBUFSZ];
static volatile u8 rcvhead, rcvtail;
static u8 adv_rcvbuf(u8 ptr)
{
    return ((ptr+1)&(RCVBUFSZ-1));
}

#define TXBUFSZ 16 /* must be a power of 2 */
static u8 txbuf[TXBUFSZ];
static volatile u8 txhead, txtail;
static u8 adv_txbuf(u8 ptr)
{
	return ((ptr+1)&(TXBUFSZ-1));
}

static void txie(u8 en)
{
	if (en)
	{
		UCSR0B |= _BV(UDRIE0);
	}
	else
	{
		UCSR0B &= ~_BV(UDRIE0);
	}
}

SIGNAL(SIG_UART0_DATA)
{
	if (txhead != txtail)
	{
		UDR0 = txbuf[txhead];
		txhead = adv_txbuf(txhead);
	}
	if (txhead == txtail)
	{
		/* disable interrupt until more data to send */
		txie(0);
	}		
}

void tx_enqueue(u8 byte)
{
    u8 nt = adv_txbuf(txtail);
retry:
	if (nt == txhead)
	{
		if (get_interrupt_flag())
		{
			/* interrupts are enabled, so we can busy wait for space in the buffer */
			while (txhead == nt)
				;
			++commstats.tx_ringwait;
			goto retry;
		}
		else
		{
			/* can't wait with interrupts disabled. 
			 * could call TX ISR directly I guess... */
			++commstats.tx_ringdrop;			
		}
	}
	else
	{
		txbuf[txtail] = byte;
		txtail = nt;
		txie(1);
		++commstats.phy_bytes_tx;		
	}
}

u8 nak;

SIGNAL(SIG_UART0_RECV)
{
	u8 r,i=0;
	set_cts(0);
	while ((r=UCSR0A) & _BV(RXC0))
	{		
		if (i>0)
		{
			++commstats.rx_uart_multi;
		}
		if(r&(_BV(DOR0)|_BV(FE0)|_BV(UPE0)))
		{			
			if (r&_BV(DOR0))
			{
				++commstats.rx_uart_overrun;
			}
			if (r&_BV(FE0))
			{
				++commstats.rx_uart_frame_error;			
			}
			if (r&_BV(UPE0))
			{
				++commstats.rx_uart_parity_error;
			}
			nak = 1;				
		}							
	    u8 nh = adv_rcvbuf(rcvhead);
	    if (nh == rcvtail)
	    {
			++commstats.rx_ring_overflow;
			UDR0;
		}
		else
		{   
	        rcvbuf[rcvhead] = UDR0;
			rcvhead = nh;
		}    
		++i;		
	}		
}	

void debug_led(u8 led, u8 val);
/* called by task_dispatcher periodically.
 * returns 1 if work was done that may require
 * a reschedule */
u8 comms_task()
{
    u8 ret = 0;
    u8 code, payload_len;

    while (rcvtail != rcvhead)
    {		        
        rx_notify(rcvbuf[rcvtail], 0);
        rcvtail = adv_rcvbuf(rcvtail);
        ret = 1;
    }
	if (nak) 
	{
		send_msg(0,TASK_ID_COMMS<<4|COMMS_MSG_NAK,0,NULL);
		nak = 0;
	}
	
	set_cts(1);

readmailbox:
    if (mailbox_head(&comms_taskinfo.mailbox, &code, &payload_len))
    {        
        static u8 saved_code, saved_to;
        u8 again = 0;
        switch(code)
        {
            case COMMS_MSG_HEADER:
                if (payload_len == 2)
                {
                    mailbox_copy_payload(&comms_taskinfo.mailbox,
                            &saved_to, 1, 0);
                    mailbox_copy_payload(&comms_taskinfo.mailbox,
                            &saved_code, 1, 1);
                }
                again = 1;
                break;
            case COMMS_MSG_PAYLOAD:
            {
                u8 msgbuf[MAX_BUFFERED_MSG_SIZE];
                u8 sz;
                sz = mailbox_copy_payload(&comms_taskinfo.mailbox,
                    msgbuf, MAX_BUFFERED_MSG_SIZE, 0);
                send_msg(saved_to, saved_code, sz, msgbuf);
                break;
            }
        }
        mailbox_advance(&comms_taskinfo.mailbox);
        if (again)
            goto readmailbox;
    }
    

    return ret;
}

void packet_received(msgaddr_t addr, u8 code, u8 length, u8 flags, u8 *payload)
{
    static u8 led;
    led=!led;

//    debug_led(1, 0);

    /* code is divided into two nibbles.
     * the first nibble indicates the task mailbox for the
     * message, and the second is determined by the task. */
    u8 taskid = code>>4;
    code &= 0xF;
    task_t *task = get_task_by_id(taskid);
    
    u8 delivered = 0;

    if (taskid == TASK_ID_COMMS)
    {

        if (code == COMMS_MSG_ECHO_REQUEST)
        {
            send_msg(addr.from, 
                    TASK_ID_COMMS<<4|COMMS_MSG_ECHO_REPLY, 
                    length, payload);
            delivered = 1;
        }
        else if (code == COMMS_MSG_RESET_BOARD)
        {
            wdt_enable(WDTO_15MS);

            /* Wait for the watchdog to reset the chip */
            while(1)
                ;
        }
        else if (code == COMMS_MSG_STATS)
        {
            u8 clear = 0;
            if (length >= 1)
            {
                clear = payload[0];
            }
            send_commstats(clear);
            delivered = 1;
        }            
		else if (code == COMMS_MSG_SET_BAUD)
		{
			if (length == 2)
			{
				UBRR0L = payload[0];			
				if (payload[1])
				{
					UCSR0A |= _BV(U2X0);					
				}
				else
				{
					UCSR0A &= ~_BV(U2X0);
				}
				delivered = 1;
			}
		}							
				
    }
    else if (task)
    {
        if (mailbox_deliver(&(task->mailbox), 
                        code & 0x0F, length, payload))
        {
            ++commstats.rx_mailbox_overflow;
        }
        else
        {
            delivered = 1;
        }            
    }

    if (!delivered)
    {
        u8 pl[] = {taskid, code};
        send_msg(addr.from, 
                TASK_ID_COMMS<<4|COMMS_MSG_BADTASK, 
                sizeof(pl), pl);
        ++commstats.badcode_pkts;
    }
    
    if(payload)
    {
        bufferpool_release(payload);
    }
}

/* This should be changed to be interrupt-driven.
 * A simple Tx ring queue using the TX done interrupt
 * would suffice... */
void tx_enqueue_poll(u8 data)
{
    while (!(UCSR0A & _BV(UDRE0)))
        ;
    UDR0 = data;
    ++commstats.phy_bytes_tx;
}

int send_msg_buffered(u8 to, u8 code, u8 payload_len, u8 *payload)
{
    u8 flags;
    u8 header[2];
    u8 ret;
    header[0] = to;
    header[1] = code;

    /* Must disable interrupts to assure that both header and payload
     * arrive in comms mailbox sequentially. */
    ret = 1;
    flags = disable_interrupts();
    if (mailbox_deliver(&comms_taskinfo.mailbox, COMMS_MSG_HEADER, 2, (u8 *)header))
    {
        ++commstats.tx_nobuf;
    }
    else if (mailbox_deliver(&comms_taskinfo.mailbox, COMMS_MSG_PAYLOAD, payload_len, payload))
    {
        ++commstats.tx_nobuf;
    }
    else
    {
        ret = 0;
    }
    restore_flags(flags);
    return ret;
}

extern u32 uptime;
extern u8 reset_cause;

void send_commstats(u8 clear_on_send)
{
    u8 flags = disable_interrupts();

    commstats.rxstat = rx_stat;
    commstats.uptime = uptime;
	commstats.reset_cause = reset_cause;
    u8 r = send_msg_buffered(0, TASK_ID_COMMS<<4|COMMS_MSG_STATS, sizeof(commstats_t), (u8 *)&commstats);
    if (r == 0 && clear_on_send)
    {
        memset((void *)&commstats, 0, sizeof(commstats));
    }
    
    restore_flags(flags);
}

