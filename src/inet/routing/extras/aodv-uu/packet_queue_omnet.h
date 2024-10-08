/*****************************************************************************
 *
 * Copyright (C) 2001 Uppsala University & Ericsson AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Erik Nordstr�m, <erik.nordstrom@it.uu.se>
 *
 *****************************************************************************/
#ifndef _PACKET_QUEUE_H
#define _PACKET_QUEUE_H

#ifndef NS_NO_GLOBALS

#define MAX_QUEUE_LENGTH 512
#define MAX_QUEUE_TIME 10000 /* Maximum time packets can be queued (ms) */
#define GARBAGE_COLLECT_TIME 1000 /* Interval between running the
* garbage collector (ms) */
#include "aodv-uu/defs_aodv.h"
#include "aodv-uu/list.h"

namespace inet {

namespace inetmanet {

/* Verdicts for queued packets: */
enum
{
    PQ_DROP = 0,
    PQ_SEND = 1,
    PQ_ENC_SEND = 2
};
#ifndef AODV_USE_STL
struct q_pkt
{
    list_t l;
    struct in_addr  dest_addr;
    struct timeval q_time;
    cPacket *p;
};

struct packet_queue
{
    list_t head;
    unsigned int len;
    struct timer garbage_collect_timer;
    unsigned int length() { return len; }
};
#else

struct q_pkt : public omnetpp::cObject
{
    struct in_addr  dest_addr;
    struct timeval q_time;
    Packet *p;
};

struct packet_queue
{
    std::vector<q_pkt*> pkQueue;
    struct timer garbage_collect_timer;
    unsigned int length() { return pkQueue.size(); }
};

#endif

} // namespace inetmanet

} // namespace inet

#endif              /* NS_NO_GLOBALS */

#ifndef NS_NO_DECLARATIONS
struct packet_queue PQ;
void packet_queue_add(Packet * p, struct in_addr dest_addr);
void packet_queue_add_inject(Packet * p, struct in_addr dest_addr);
void packet_queue_init();
void packet_queue_destroy();
int packet_queue_set_verdict(struct in_addr dest_addr, int verdict);
int packet_queue_garbage_collect(void);

#endif              /* NS_NO_DECLARATIONS */

#endif
