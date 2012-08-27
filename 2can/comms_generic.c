/******************************************************************************
* File:              comms_generic.c
* Author:            Kevin Day
* Date:              August, 2004
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
* Copyright (c) 2004 Kevin Day
* All rights reserved.
*******************************************************************************/

#include "comms_generic.h"
#include "bufferpool.h"

commstats_t commstats;

/* External byte transmit routine.
 * Will block on full buffer.
 */
void tx_enqueue(u8 data);

/* External node ID routine 
 * Return unique 4 bit value
 * '7' is invalid. */
u8 get_node_id();


static u8 tx_enqueue_with_escape(u8 octet)
{
    if (octet == 0x7D || octet == 0x7E)
    {
        tx_enqueue(0x7D);
        tx_enqueue(octet ^ 0x20);
    }
    else
    {
        tx_enqueue(octet);
    }
    return octet;
}


#define ETOOBIG 2
#define EBADADDR 3

#define SANITY_CHECK
int send_msg(u8 to, u8 code, u8 payload_len, u8 *payload)
{
    s8 i, prefix;
    //u8 flags = 0;
    u16 csum = 0;
     
    /* Build the message up one byte at a time rather than
     * using the structure, to save some RAM. */

    tx_enqueue(COMM_PREAMBLE);

    if (payload_len >= 16)
    {
        /* Set "payload" length to 1 indicating there
         * is one octet of payload size preceeding the
         * payload itself. */
        csum += tx_enqueue_with_escape(((1<<4) | MSG_FLAGS_BIGPACKET));
        prefix = -1;
    }
    else
    {
        /* flags . payload_length -- flags are 0 */
        csum += tx_enqueue_with_escape(payload_len<<4);
        prefix = 0;
    }
    
    csum += tx_enqueue_with_escape(to<<4 | get_node_id());
    
    csum += tx_enqueue_with_escape(code);
        
    for(i=prefix; i<payload_len; i++)
    {
        u8 v;
        if (i>=0)
        {
            v = payload[i];
        }
        else
        {
            v = payload_len>>((i-prefix)*8);
        }

        csum += tx_enqueue_with_escape(v);
    }
    
    tx_enqueue_with_escape(csum>>8);
    tx_enqueue_with_escape(csum);

    ++commstats.packets_tx;

    return 0;
}

/* External packet dispatcher.
 * Called when a packet is successfully received. */
void packet_received(msgaddr_t addr, u8 code, u8 length, u8 flags, u8 *payload);

/* Buffer allocator */
u8 *bufferpool_request(u8 size);
void bufferpool_release(u8 *ptr);

rx_status_t     rx_stat = {PREAMBLE_HUNT, 0};
msghdr_t        rx_hdr_buf;
msgtrlr_t       rx_trlr_buf;
u8              *rx_buf_ptr;
u8              *rx_payload_ptr;
u16             rx_payload_len;

u16             csum;
extern u8       nak;

u8  rx_notify(u8 data, u8 error_detected)
{
    u8 ret = 0;
    
    ++commstats.phy_bytes_rx;

    if (data == COMM_PREAMBLE)
    {
        if (rx_stat.state != PREAMBLE_HUNT)
        {
            COMMS_DEBUG("Unexpected preamble in state %d\n", rx_stat.state);
            ++commstats.errors;
        }
        if (rx_payload_ptr)
        {
            bufferpool_release(rx_payload_ptr);
        }
        
        csum = 0;
        
        rx_stat.state = IN_HEADER;
        rx_stat.esc = 0;
        rx_buf_ptr = (u8*)&rx_hdr_buf;
        return 0;
    }

    if (rx_stat.esc)
    {
        /* Last character was 0x7D */
        data ^= COMM_ESCXOR;
        rx_stat.esc = 0;
    }
    else if (data == COMM_ESCAPE)
    {
        /* No character available yet */
        rx_stat.esc = 1;
        return 0;
    }

    csum += data;
    
    switch(rx_stat.state)
    {
        case PREAMBLE_HUNT:
            ++commstats.junk;
            break;
        case IN_HEADER:
            *rx_buf_ptr++ = data;
            if (rx_buf_ptr == (u8*)&rx_hdr_buf + sizeof(msghdr_t))
            {
                if (rx_hdr_buf.payload_length > 0)
                {
                    /* Payload size is known.  Get a buffer. */
                    rx_payload_ptr = bufferpool_request(rx_hdr_buf.payload_length);
                    if (rx_payload_ptr == 0)
                    {
                        COMMS_DEBUG("Failed to allocate buffer of %d bytes\n",
                                rx_hdr_buf.payload_length);
                        rx_stat.state = PREAMBLE_HUNT;
                        ++commstats.rx_pkts_overflow;
						nak=1;
                        return 0;
                    }
                    rx_stat.state = IN_PAYLOAD;                    
                    rx_buf_ptr = rx_payload_ptr;
                    rx_payload_len = rx_hdr_buf.payload_length;
                }
                else
                {                    
                    rx_stat.state = IN_TRAILER;
                    rx_buf_ptr = (u8*) &rx_trlr_buf;
                }                    
            }
            break;
        case IN_PAYLOAD:
            *rx_buf_ptr++ = data;
            if ((rx_buf_ptr - rx_payload_ptr) == rx_payload_len)
            {
#ifndef EMBEDDED
                /* big packet receive not supported on AVR. */
                if (rx_hdr_buf.flags & MSG_FLAGS_BIGPACKET)
                {
                    COMMS_DEBUG("Got big packet\n");
                    u8 i;
                    u32 real_payload_length = 0;
                    for(i=0; i<rx_hdr_buf.payload_length; i++)
                    {
                        real_payload_length += rx_payload_ptr[i]<<(8*i);
                    }
                    bufferpool_release(rx_payload_ptr);
                    /* BUG: all comparisons between rx_payload_ptr and
                     * rx_hdr_buf will break from here on.
                     * Bug packet RX support is not working yet.
                     * Implement this as a separate set of states... */
                    rx_payload_ptr = bufferpool_request(real_payload_length);
                    rx_hdr_buf.flags &= ~(MSG_FLAGS_BIGPACKET);
                }
                else
#endif
                {                
                    /* Payload done. */
                    rx_stat.state = IN_TRAILER;
                    rx_buf_ptr = (u8*)&rx_trlr_buf;
                }
            }
            break;
        case IN_TRAILER:
            *rx_buf_ptr++ = data;

            if (rx_buf_ptr == (u8*)&rx_trlr_buf + sizeof(msgtrlr_t))
            {
                /* Do checksum verification */
                u16 rxcsum = (rx_trlr_buf.crc16_high<<8 | rx_trlr_buf.crc16_low);
                u16 mcsum = csum - (rx_trlr_buf.crc16_high + rx_trlr_buf.crc16_low);

                if (rxcsum != mcsum)
                {                        
                    ++commstats.checksum_err;                        
					nak=1;
                }
                else
                {
                    packet_received(rx_hdr_buf.address, rx_hdr_buf.message_code,
                                rx_hdr_buf.payload_length,
                                rx_hdr_buf.flags,
                                rx_payload_ptr);
                    ret = 1;
                    ++commstats.packets_rx;
                }
                rx_stat.state = PREAMBLE_HUNT;
                rx_payload_ptr = 0;
            }
            break;
        default:
            /* bad state */
            rx_stat.state = PREAMBLE_HUNT;
            ++commstats.errors;
            break;
    }
    return ret;
}

    
