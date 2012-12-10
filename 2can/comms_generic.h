/******************************************************************************
* File:              comms_generic.h
* Author:            Kevin Day
* Date:              December, 2004
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

#ifndef COMMS_GENERIC_H
#define COMMS_GENERIC_H

#include "types.h"

#define COMMS_MSG_ECHO_REQUEST 0x1
#define COMMS_MSG_ECHO_REPLY   0x2
#define COMMS_MSG_STATS        0x3
#define COMMS_MSG_SET_BAUD     0x4
#define COMMS_MSG_NAK          0xA
#define COMMS_MSG_RESET_BOARD  0xB
#define COMMS_MSG_HELLO        0xF
#define COMMS_MSG_BADTASK      0xE

#define CAN_ERROR              0x0
#define COMMS_MSG_CAN_RAW      0x1 
#define CAN_SEND_RAW_MSG       0x2
#define CAN_MSG_RX             0x3
#define CAN_STATS              0x4
#define CAN_RX_ERROR           0xE
#define CAN_SET_ACCEPT_FILTER  0xA
#define CAN_SET_BT             0xB
#define CAN_SET_SID_FILTER     0xC

#define BROADCAST_NODE_ID      0xF

#define MCP_REG_READ           0x4
#define MCP_REG_WRITE          0x5


#define COMMS_DEBUG(fmt, ...)

#define COMM_PREAMBLE   0x7E
#define COMM_ESCAPE     0x7D
#define COMM_ESCXOR     0x20

void send_commstats(u8 clear_on_send);

/* preamble
 * 0x7E ala HDLC 
 * Not part of structure, but required to begin message on line */
//u8      preamble;       
typedef struct {
    /* from
     *  sender's address 
     *  '7' is invalid */
    u8      from        : 4;
    /* to
     *  recipient's address, or 0x7 (broadcast) */
    u8      to          : 4;
} msgaddr_t;

#define MSG_FLAGS_BIGPACKET 1

typedef struct {
    /* flags
     *  not yet defined */
    /* Flags:
     *      xxx0    Small packet (< 16 byte payload)
     *      xxx1    Big packet.  Payload length field
     *              is set to the number of octets in the
     *              start of the payload which hold the
     *              additional size.  (if more than one
     *              octet of size, the octets are in 
     *              little endian order). */
    u8      flags       : 4;
    /* payload_length
     *  in bytes, not inclusive of header or trailer or checksum */
    u8      payload_length : 4;    

    msgaddr_t address;
    
    /* message_code
     *  defined by recipient.  may constitute entire message.   */
    u8      message_code;
} msghdr_t;

/* The header, payload, and trailer are escaped such that 0x7E will never appear.
 * 0x7D is the escape character.
 * 0x7D (0x7D^0x20) --> 0x7D in payload
 * 0x7D (0x7E^0x20) --> 0x7E in payload */

typedef struct {
    /* checksum (optional)
     *  includes header and payload
     *  escape sequence for payload is in effect here also
     *  If value is 0x7D 0x00 then checksum is not present.
     */
    u8      crc16_high;  
    u8      crc16_low;  
} msgtrlr_t;

int send_msg(u8 to, u8 code, u8 payload_len, u8 *payload);

enum {DEV_UART0, DEV_SPI_AVRDISP} msg_dest;
void set_msg_dest(u8 to);

/* Note: must use _buffered version when interrupts are disabled.
 * If the TX queue fills up, send_msg (actually uart_putc) will 
 * busy wait until the UART TX done interrupt fires. 
 * send_msg_buffered will return non-zero if the comms task mailbox
 * fills up, and the message will be discarded.  */
#define MAX_BUFFERED_MSG_SIZE 64
int send_msg_buffered(u8 to, u8 code, u8 payload_len, u8 *payload);

u8 rx_notify(u8 data, u8 error_detected);

/* Asynchronous notification of byte reception. 
 * Called by serial driver. 
 * On error case (i.e. bad async frame), the current
 * packet (if any) is aborted and hunt mode is entered. */
typedef enum {
    PREAMBLE_HUNT=0,
    IN_HEADER=1,
    IN_PAYLOAD=2,
    IN_TRAILER=3
} rx_state_t;

typedef struct {
    rx_state_t  state;
    u8          esc;
} rx_status_t;

extern rx_status_t  rx_stat;

//#pragma pack(push,1)
typedef struct {
    u32     packets_rx;                 //0-3
    u32     packets_tx;                 //4-7
    u32     phy_bytes_rx;               //8-11
    u32     phy_bytes_tx;               //12-15
    u16     junk;                       //16-17
    u16     errors;                     //18-19
    u16     checksum_err;               //20-21
    u16     badcode_pkts;               //22-23
    u16     rx_ring_overflow;           //24-25
    u16     rx_pkts_overflow;           //26-27
    rx_status_t rxstat;                 //28-29
    u32     uptime;                     //30-33
    u16     tx_nobuf;                   //34-35
	u16     rx_mailbox_overflow;        //36-37
    u16     rx_uart_overrun;            //38-39
    u16     rx_uart_frame_error;        //40-41
    u16     rx_uart_parity_error;       //42-43
	u16     rx_uart_multi;              //44-45
	u16     tx_ringdrop;                //46-47
	u16     tx_ringwait;                //48-49
	u8      reset_cause;                //50-50
} commstats_t;
//#pragma pack(pop)
extern commstats_t commstats;

#endif /* !COMMS_GENERIC_H */
