/******************************************************************************
* File:              2can.c
* Author:            Kevin Day
* Date:              December, 2011
* Description:       
*                    2CAN bus analyzer main file
*                    
* Copyright (c) 2011 Kevin Day
* All rights reserved.
*******************************************************************************/


#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include "types.h"
#include "tasks.h"
#include "comms_generic.h"
#include "timers.h"
#include "adc.h"
#include "persist.h"
#include "bufferpool.h"
#include "comms_avr.h"
#include "spimaster.h"
#include "avrcan.h"
#include "avrsys.h"
#include "mcp2515.h"

/* LEDs on port F: 0,1,2 red,white,blue */

#define LED_PORT PORTF
#define LED_DIR  DDRF
#define LED_PIN  PINF
#define LED_BIT  1

static void start_blink_timer();

u8 reset_cause;

int main()
{    
    reset_cause = MCUSR;
    MCUSR = 0;
    wdt_disable();
    wdt_enable(WDTO_2S);

    // turn on all LEDs until initialization is complete
    LED_DIR |= 7;

    bufferpool_init();
    
    init_persist_data();
    
    systimer_init();

    spimaster_init();

    /* Enable interrupts */
    sei();

    /* Populate the tasklist in priority order */    
    tasklist[num_tasks++] = comms_task_create();
    tasklist[num_tasks++] = can_task_create();
    tasklist[num_tasks++] = mcp_task_create();

	// turn LEDs off
    LED_PORT |= 7;

    start_blink_timer();

//    send_msg_buffered(0, 0x99, 0, NULL);


    /* non-preemptive static priority scheduler */    
    while(1)
    {
        wdt_reset();

        u8 taskidx;
        u8 r;
        for(taskidx=0; taskidx<num_tasks; taskidx++)
        {
            r = tasklist[taskidx]->taskfunc();
            if (r)
                break;
        }
    }
}
void debug_led(u8 led, u8 val)
{
    u8 flags = disable_interrupts();
    
    if (!val)
    {
        LED_PORT |= (1<<led);
    }
    else
    {
        LED_PORT &= ~(1<<led);
    }
    restore_flags(flags);
}

timerentry_t blink_timer;

u32 uptime;
u8 led;
void onesec_callback(timerentry_t *t)
{
    u16 ms = 1000;
    if (t->key == 0)
    {
        debug_led(0, 0);
    }
    else
    {
        debug_led(0, 1);
    }

    ++uptime;

//    static u8 b;
//    tx_enqueue_uart(b++);

//    static u16 pl;
//    send_msg_buffered(0, 0x55, sizeof(pl), (u8*)&pl);
//    ++pl;

//    send_commstats(0);

    register_timer_callback(&blink_timer, MS_TO_TICK(ms), onesec_callback,
            !t->key);
}

static void start_blink_timer()
{
    onesec_callback(&blink_timer);
}

		
