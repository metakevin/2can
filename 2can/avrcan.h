/******************************************************************************
* File:              avrcan.h
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

#ifndef AVRCAN_H
#define AVRCAN_H

task_t *can_task_create();

#define CAN_20A 1
#define CAN_20B 2

typedef enum {
	v20a,
	v20b
} can_ver_t;

typedef u16 can_20a_addr_t;
typedef u32 can_20b_addr_t;

typedef struct {
	u8  ver_20b : 1; /* 11 bit IDs if zero; 29 bit if one */	
	u8  rtr     : 1;	
	u8  unused  : 6;
	u32 id;
} can_addr_t;

#define CAN_MAX_PAYLOAD 8

typedef struct {
	can_addr_t addr;
	u8 payload_len;
	u8 payload[CAN_MAX_PAYLOAD];	
} can_msg_t;

typedef struct {
	u16 buserr;
	u16 overtime;
	u16 stufferr;
	u16 crcerr;
	u16 frameerr;
	u16 ackerr;
	u16 unhandled;
	u16 mob_rxok;
	u16 mob_txok;
	u16 mob_dlcw;
	u16 mob_biterr;
	u16 mob_serr;
	u16 mob_cerr;
	u16 mob_ferr;
	u16 mob_aerr;
	u16 rx_filtered;
	u16 rx_relayed;
} canstats_t;


/* returns 0 on success */
u8 send_can_msg(can_addr_t *a, u8 plen, u8 *payload);

/* SID filter update message payload:
 * 1 byte: block
 * 14 bytes: 7 SID values for that block
 */
#define SIDFILTER_BLOCK_SIZE 7
#define SIDFILTER_NUM_BLOCKS 2
#define SIDFILTER_ID_MASK 0x7FF
#define SIDFILTER_VALID_MASK 0x800

/* Top nibble is divided into 2 halves, one per interface.
 * 15:14 is interface 1 and 13:12 is interface 0 */
#define SIDFILTER_NOHOST 1
#define SIDFILTER_RELAY 2

extern u16 sidfilter[SIDFILTER_BLOCK_SIZE*SIDFILTER_NUM_BLOCKS];

extern task_t can_taskinfo;

u8 can_id_match(can_addr_t *addr, u8 interface);

#endif /* !AVRCAN_H */
