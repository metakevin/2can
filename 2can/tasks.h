/******************************************************************************
* File:              tasks.h
* Author:            Kevin Day
* Date:              December, 2004
* Description:       
*                    Mailbox and task definitions.
*                    The only interesting property about a task is that
*                    it has a mailbox, really.
*                    
* Copyright (c) 2004 Kevin Day
* All rights reserved.
*******************************************************************************/

#ifndef TASKS_H
#define TASKS_H

#include "types.h"

#define MAX_TASKS           16

#define TASK_ID_MCP             0x4
#define TASK_ID_CAN             0x8
#define TASK_ID_COMMS           0xC

// messages can be as small as 7 bits.
// if larger than 7 bits, then the MSB of the first
// word is set, indicating that a length byte follows,
// and subsequently length bytes of payload.
// i.e. 0x84, 0x2, 0xA, 0xB -- code is 4, length 2, payload A, B
//typedef struct {
//    u8      has_payload : 1;
//    u8      code        : 7;
//    u8      length; // if payload
//    // ...
//    // payload
//} taskmsg_t;

typedef struct {
    u8 *    start;
    u8      size;     
    u8 *    head;
    u8 *    tail;
} mailbox_t;

typedef u8 (*taskfunc_t)();
typedef struct {
    u8          task_id  : 4;
    taskfunc_t  taskfunc;
    mailbox_t   mailbox;
} task_t;

extern task_t *tasklist[MAX_TASKS];
extern u8      num_tasks;


u8   mailbox_deliver(mailbox_t *box, u8 code, u8 payload_len, u8 *payload);
u8   mailbox_head(mailbox_t *box, u8 *code, u8 *payload_len);
u8   mailbox_copy_payload(mailbox_t *box, u8 *buf, u8 buflen, u8 offset);
void mailbox_advance(mailbox_t *box);

task_t *get_task_by_id(u8 task_id);
u8 send_to_task(u8 taskid, u8 code, u8 payload_len, u8 *payload);

task_t *setup_task(task_t *task, u8 taskid, taskfunc_t tf, u8 *buf, u8 size);

#endif /* !TASKS_H */
