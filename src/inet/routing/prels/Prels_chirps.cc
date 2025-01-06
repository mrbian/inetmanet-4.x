/*
 * Prels_mpr.cc
 *
 *  Created on: Dec 29, 2024
 *      Author: bman
 */


#include "Prels.h"

namespace inet
{

#ifndef state_
#define state_ (*state_ptr)
#endif

void
Prels::add_dup_tuple(Prels_dup_tuple* tuple)
{
    state_.insert_dup_tuple(tuple);
}

void
Prels::rm_dup_tuple(Prels_dup_tuple* tuple)
{
    state_.erase_dup_tuple(tuple);
}

void
Prels::add_nb_tuple(Prels_nb_tuple* tuple)
{
    state_.erase_nb_tuple(tuple->nb_main_addr());
    state_.insert_nb_tuple(tuple);
}

void
Prels::rm_nb_tuple(Prels_nb_tuple* tuple)
{
    state_.erase_nb_tuple(tuple);
}

void
Prels::add_nb2hop_tuple(Prels_nb2hop_tuple* tuple)
{
    state_.insert_nb2hop_tuple(tuple);
}
void
Prels::rm_nb2hop_tuple(Prels_nb2hop_tuple* tuple)
{
    state_.erase_nb2hop_tuple(tuple);
}

void
Prels::add_mprsel_tuple(Prels_mprsel_tuple* tuple)
{
    state_.insert_mprsel_tuple(tuple);
    ansn_ = (ansn_ + 1)%(PRELS_MAX_SEQ_NUM + 1);
}

void
Prels::rm_mprsel_tuple(Prels_mprsel_tuple* tuple)
{
    state_.erase_mprsel_tuple(tuple);
    ansn_ = (ansn_ + 1)%(PRELS_MAX_SEQ_NUM + 1);
}


void
Prels::add_node_tuple(Prels_node_tuple* tuple)
{
    state_.erase_node_tuple(tuple);
    state_.insert_node_tuple(tuple);
}

void
Prels::rm_node_tuple(Prels_node_tuple* tuple)
{
    state_.erase_node_tuple(tuple);
}

void
Prels::recv_prels(Packet* msg)
{
    auto sh = msg->removeAtFront<SignalHeader>();
    nsaddr_t src_addr = sh->getPrevAddr();

    // All routing messages are sent from and to port RT_PORT,
    // so we check it.
//    emit(packetReceivedSignal, msg);
    auto &op = msg->popAtFront<PrelsPkt>();

    // If the packet contains no messages must be silently discarded.
    // There could exist a message with an empty body, so the size of
    // the packet would be pkt-hdr-size + msg-hdr-size.
    if (op->getChunkLength() < B(PRELS_PKT_HDR_SIZE + PRELS_MSG_HDR_SIZE))
    {
        delete msg;
        return;
    }

// Process Prels information
    assert(op->msgArraySize() >= 0 && op->msgArraySize() <= PRELS_MAX_MSGS);
    for (int i = 0; i < (int) op->msgArraySize(); i++)
    {
         auto msgAux = op->msg(i);
         PrelsMsg msg = msgAux;

        // If ttl is less than or equal to zero, or
        // the receiver is the same as the originator,
        // the message must be silently dropped
        // if (msg.ttl() <= 0 || msg.orig_addr() == ra_addr())
        if (msg.ttl() <= 0 || msg.orig_addr() == m_selfIpv4Address)
            continue;

        // If the message has been processed it must not be
        // processed again
        bool do_forwarding = true;

        Prels_dup_tuple* duplicated = state_.find_dup_tuple(msg.orig_addr(), msg.msg_seq_num());
        if (duplicated == nullptr)
        {
            // Process the message according to its type
            if (msg.msg_type() == PRELS_HELLO_MSG)
                process_hello(msg);
            else if (msg.msg_type() == PRELS_TC_MSG)
                process_tc(msg);
            else
                throw cRuntimeError("Prels:: Unknown message Type !");
        }
        else
        {
            // If the message has been considered for forwarding, it should
            // not be retransmitted again
            for (auto it = duplicated->iface_list().begin();
                    it != duplicated->iface_list().end();
                    it++)
            {
                if (*it == m_selfIpv4Address)
                {
                    do_forwarding = false;
                    break;
                }
            }
        }

        if (do_forwarding)
        {
            // HELLO messages are never forwarded.
            // TC and MID messages are forwarded using the default algorithm.
            // Remaining messages are also forwarded using the default algorithm.
            if (msg.msg_type() != PRELS_HELLO_MSG)
                forward_default(msg, duplicated, src_addr);
        }

    }
    delete msg;

    // After processing all messages, we must recompute routing table
    rtable_computation();
}

void
Prels::CoverTwoHopNeighbors(const nsaddr_t &neighborMainAddr, nb2hopset_t & N2)
{
    std::set<nsaddr_t> toRemove;
    for (auto it = N2.begin(); it != N2.end(); it++)
    {
        Prels_nb2hop_tuple* twoHopNeigh = *it;
        if (twoHopNeigh->nb_main_addr() == neighborMainAddr)
        {
            toRemove.insert(twoHopNeigh->nb2hop_addr());
        }
    }
    // Now remove all matching records from N2
    for (auto it = N2.begin(); it != N2.end();)
    {
        Prels_nb2hop_tuple* twoHopNeigh = *it;
        if (toRemove.find(twoHopNeigh->nb2hop_addr()) != toRemove.end())
            it = N2.erase(it);
        else
            it++;
    }
}


void
Prels::mpr_computation()
{
    // MPR computation should be done for each interface. See section 8.3.1
    // (RFC 3626) for details.
    state_.clear_mprset();

    nbset_t N; nb2hopset_t N2;
    // N is the subset of neighbors of the node, which are
    // neighbor "of the interface I"
    for (auto it = nbset().begin(); it != nbset().end(); it++)
        if ((*it)->nb_type != PRELS_NOT_NEIGH) // I think that we need this check
            N.push_back(*it);

    // N2 is the set of 2-hop neighbors reachable from "the interface
    // I", excluding:
    // (i)   the nodes only reachable by members of N with willingness WILL_NEVER
    // (ii)  the node performing the computation
    // (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
    //       link to this node on some interface.
    for (auto it = nb2hopset().begin(); it != nb2hopset().end(); it++)
    {
        Prels_nb2hop_tuple* nb2hop_tuple = *it;
        // (ii)  the node performing the computation
        if (nb2hop_tuple->nb2hop_addr() == m_selfIpv4Address)
        {
            continue;
        }
        // excluding:
        // (i) the nodes only reachable by members of N with willingness WILL_NEVER
        bool ok = false;
        for (nbset_t::const_iterator it2 = N.begin(); it2 != N.end(); it2++)
        {
            Prels_nb_tuple* neigh = *it2;
            if (neigh->nb_main_addr() == nb2hop_tuple->nb_main_addr())
            {
                ok = true;
                break;
            }
        }
        if (!ok)
        {
            continue;
        }

        // excluding:
        // (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
        //       link to this node on some interface.
        for (auto it2 = N.begin(); it2 != N.end(); it2++)
        {
            Prels_nb_tuple* neigh = *it2;
            if (neigh->nb_main_addr() == nb2hop_tuple->nb2hop_addr())
            {
                ok = false;
                break;
            }
        }
        if (ok)
            N2.push_back(nb2hop_tuple);
    }

    // 1. Start with an MPR set made of all members of N with
    // N_willingness equal to WILL_ALWAYS
//    for (auto it = N.begin(); it != N.end(); it++)
//    {
//        Prels_nb_tuple* nb_tuple = *it;
//        if (nb_tuple->willingness() == OLSR_WILL_ALWAYS)
//        {
//            state_.insert_mpr_addr(nb_tuple->nb_main_addr());
//            // (not in RFC but I think is needed: remove the 2-hop
//            // neighbors reachable by the MPR from N2)
//            CoverTwoHopNeighbors (nb_tuple->nb_main_addr(), N2);
//        }
//    }

    // 2. Calculate D(y), where y is a member of N, for all nodes in N.
    // We will do this later.

    // 3. Add to the MPR set those nodes in N, which are the *only*
    // nodes to provide reachability to a node in N2. Remove the
    // nodes from N2 which are now covered by a node in the MPR set.

    std::set<nsaddr_t> coveredTwoHopNeighbors;
    for (auto it = N2.begin(); it != N2.end(); it++)
    {
        Prels_nb2hop_tuple* twoHopNeigh = *it;
        bool onlyOne = true;
        // try to find another neighbor that can reach twoHopNeigh->twoHopNeighborAddr
        for (nb2hopset_t::const_iterator it2 = N2.begin(); it2 != N2.end(); it2++)
        {
            Prels_nb2hop_tuple* otherTwoHopNeigh = *it2;
            if (otherTwoHopNeigh->nb2hop_addr() == twoHopNeigh->nb2hop_addr()
                    && otherTwoHopNeigh->nb_main_addr() != twoHopNeigh->nb_main_addr())
            {
                onlyOne = false;
                break;
            }
        }
        if (onlyOne)
        {
            state_.insert_mpr_addr(twoHopNeigh->nb_main_addr());

            // take note of all the 2-hop neighbors reachable by the newly elected MPR
            for (nb2hopset_t::const_iterator it2 = N2.begin(); it2 != N2.end(); it2++)
            {
                Prels_nb2hop_tuple* otherTwoHopNeigh = *it2;
                if (otherTwoHopNeigh->nb_main_addr() == twoHopNeigh->nb_main_addr())
                {
                    coveredTwoHopNeighbors.insert(otherTwoHopNeigh->nb2hop_addr());
                }
            }
        }
    }
    // Remove the nodes from N2 which are now covered by a node in the MPR set.
    for (auto it = N2.begin(); it != N2.end();)
    {
        Prels_nb2hop_tuple* twoHopNeigh = *it;
        if (coveredTwoHopNeighbors.find(twoHopNeigh->nb2hop_addr()) != coveredTwoHopNeighbors.end())
        {
            // This works correctly only because it is known that twoHopNeigh is reachable by exactly one neighbor,
            // so only one record in N2 exists for each of them. This record is erased here.
            it = N2.erase(it);
        }
        else
        {
            it++;
        }
    }
    // 4. While there exist nodes in N2 which are not covered by at
    // least one node in the MPR set:

    while (N2.begin() != N2.end())
    {

        // 4.1. For each node in N, calculate the reachability, i.e., the
        // number of nodes in N2 which are not yet covered by at
        // least one node in the MPR set, and which are reachable
        // through this 1-hop neighbor
        std::map<int, std::vector<Prels_nb_tuple*> > reachability;
        std::set<int> rs;
        for (auto it = N.begin(); it != N.end(); it++)
        {
            Prels_nb_tuple* nb_tuple = *it;
            int r = 0;
            for (auto it2 = N2.begin(); it2 != N2.end(); it2++)
            {
                Prels_nb2hop_tuple* nb2hop_tuple = *it2;

                if (nb_tuple->nb_main_addr() == nb2hop_tuple->nb_main_addr())
                    r++;
            }
            rs.insert(r);
            reachability[r].push_back(nb_tuple);
        }

        // 4.2. Select as a MPR the node with highest N_willingness among
        // the nodes in N with non-zero reachability. In case of
        // multiple choice select the node which provides
        // reachability to the maximum number of nodes in N2. In
        // case of multiple nodes providing the same amount of
        // reachability, select the node as MPR whose D(y) is
        // greater. Remove the nodes from N2 which are now covered
        // by a node in the MPR set.
        Prels_nb_tuple *max = nullptr;
        int max_r = 0;
        for (auto it = rs.begin(); it != rs.end(); it++)
        {
            int r = *it;
            if (r == 0)
            {
                continue;
            }
            for (auto it2 = reachability[r].begin();
                    it2 != reachability[r].end(); it2++)
            {
                Prels_nb_tuple *nb_tuple = *it2;
                if (max == nullptr)
                {
                    max = nb_tuple;
                    max_r = r;
                }
                else
                {
                    if (r > max_r)
                    {
                        max = nb_tuple;
                        max_r = r;
                    }
                    else if (r == max_r)
                    {
                        if (degree(nb_tuple) > degree(max))
                        {
                            max = nb_tuple;
                            max_r = r;
                        }
                    }
                }
            }
        }
        if (max != nullptr)
        {
            state_.insert_mpr_addr(max->nb_main_addr());
            CoverTwoHopNeighbors(max->nb_main_addr(), N2);
            EV_DETAIL << N2.size () << " 2-hop neighbors left to cover! \n";
        }
    }

}

bool
Prels::process_hello(PrelsMsg& msg)
{
    assert(msg.msg_type() == PRELS_HELLO_MSG);
    populate_nbset(msg);
    populate_nb2hopset(msg);
    mpr_computation();
    populate_mprselset(msg);
    return false;
}

bool
Prels::process_tc(PrelsMsg& msg)
{
    assert(msg.msg_type() == PRELS_TC_MSG);
    Prels_tc& tc = msg.tc();
    for (int i = 0; i < tc.count; i++)
    {
        assert(i >= 0 && i < PRELS_MAX_ADDRS);
        nsaddr_t addr = tc.nb_etx_main_addr(i).iface_address();
        Prels_node_tuple* node_tuple = state_.find_node_tuple(addr);
        if(node_tuple == nullptr)
        {
            node_tuple = new Prels_node_tuple;
            node_tuple->main_addr() = addr;
            node_tuple->mob_info() = tc.nb_etx_main_addr(i).mob_info();
            node_tuple->mob_update_time() = tc.nb_etx_main_addr(i).mob_update_time();
            add_node_tuple(node_tuple);
        }
        else
        {
            if(tc.nb_etx_main_addr(i).mob_update_time() > node_tuple->mob_update_time())
            {
                node_tuple->mob_info() = tc.nb_etx_main_addr(i).mob_info();
                node_tuple->mob_update_time() = tc.nb_etx_main_addr(i).mob_update_time();
            }
        }
    }
    return false;
}

void
Prels::forward_default(PrelsMsg& msg, Prels_dup_tuple* dup_tuple, const nsaddr_t &src_addr)
{
    double now = CURRENT_TIME;
    // If the sender interface address is not in the symmetric
    // 1-hop neighborhood the message must not be forwarded

    // If the message has already been considered for forwarding,
    // it must not be retransmitted again
    if (dup_tuple != nullptr && dup_tuple->retransmitted())
    {
        return;
    }

    // If the sender interface address is an interface address
    // of a MPR selector of this node and ttl is greater than 1,
    // the message must be retransmitted
    bool retransmitted = false;
    if (msg.ttl() > 1)
    {
        Prels_mprsel_tuple* mprsel_tuple = state_.find_mprsel_tuple(src_addr);
        if (mprsel_tuple != nullptr)
        {
            PrelsMsg& new_msg = msg;
            new_msg.ttl()--;
            new_msg.hop_count()++;
            // We have to introduce a random delay to avoid
            // synchronization with neighbors.
            enque_msg(new_msg, JITTER);
            retransmitted = true;
        }
    }

    // Update duplicate tuple...
    if (dup_tuple != nullptr)
    {
//        dup_tuple->time() = now + OLSR_DUP_HOLD_TIME;
        dup_tuple->retransmitted() = retransmitted;
        dup_tuple->iface_list().push_back(m_selfIpv4Address);
    }
    // ...or create a new one
    else
    {
        Prels_dup_tuple* new_dup = new Prels_dup_tuple;
        new_dup->getAddr() = msg.orig_addr();
        new_dup->seq_num() = msg.msg_seq_num();
//        new_dup->time() = now + OLSR_DUP_HOLD_TIME;
        new_dup->retransmitted() = retransmitted;
        new_dup->iface_list().push_back(m_selfIpv4Address);
        add_dup_tuple(new_dup);
        // Schedules dup tuple deletion
//        Olsr_DupTupleTimer* dup_timer =
//            new Olsr_DupTupleTimer(this, new_dup);
//        dup_timer->resched(DELAY(new_dup->time()));
    }
}

void
Prels::enque_msg(PrelsMsg& msg, double delay)
{
    assert(delay >= 0);

    msgs_.push_back(msg);
    Prels_SendPktTimer* timer = new Prels_SendPktTimer(this);
    timer->resched(delay);
}

void
Prels::send_hello()
{
    PrelsMsg msg;
    double now = CURRENT_TIME;
    msg.msg_type() = PRELS_HELLO_MSG;
//    msg.vtime() = Olsr::seconds_to_emf(OLSR_NEIGHB_HOLD_TIME);
    msg.orig_addr() = m_selfIpv4Address;
    msg.ttl() = 1;
    msg.hop_count() = 0;
    msg.msg_seq_num() = msg_seq();

    msg.hello().reserved() = 0;
//    msg.hello().htime() = Olsr::seconds_to_emf(hello_ival());
//    msg.hello().willingness() = willingness();
    msg.hello().count = 0;
    msg.hello().mob_info() = self_node_id;
    msg.hello().mob_update_time() = now;

    std::map<uint8_t, int> nbtypes_count;
    for (auto it = nbset().begin(); it != nbset().end(); it ++)
    {
        Prels_nb_tuple* nb_tuple = *it;
        if(nb_tuple->nb_type != PRELS_NOT_NEIGH)
        {
            if (state_.find_mpr_addr(nb_tuple->nb_main_addr()))
                nb_tuple->nb_type = PRELS_MPR_NEIGH;

            int count = msg.hello().count;
            auto pos = nbtypes_count.find(nb_tuple->nb_type);
            if (pos == nbtypes_count.end())
            {
                nbtypes_count[nb_tuple->nb_type] = count;
                assert(count >= 0 && count < PRELS_MAX_HELLOS);
                msg.hello().hello_msg(count).count = 0;
                msg.hello().hello_msg(count).nbtype_code() = nb_tuple->nb_type;
                msg.hello().hello_msg(count).reserved() = 0;
                msg.hello().count++;
            }
            else
                count = (*pos).second;

            int i = msg.hello().hello_msg(count).count;
            assert(i >= 0 && i < PRELS_MAX_ADDRS);

            msg.hello().hello_msg(count).nb_iface_addr(i) = nb_tuple->nb_main_addr();
            msg.hello().hello_msg(count).count++;
//            msg.hello().hello_msg(count).link_msg_size() =
//                msg.hello().hello_msg(count).size();
        }
    }

    msg.msg_size() = msg.size();

    enque_msg(msg, JITTER);
}

void
Prels::send_tc()
{
    PrelsMsg msg;
    msg.msg_type() = PRELS_TC_MSG;
//    msg.vtime() = Olsr::seconds_to_emf(OLSR_TOP_HOLD_TIME);
    msg.orig_addr() = m_selfIpv4Address;
    msg.ttl() = 255;
    msg.hop_count() = 0;
    msg.msg_seq_num() = msg_seq();

    msg.tc().ansn() = ansn_;
    msg.tc().reserved() = 0;
    msg.tc().count = 0;

    for (auto it = mprselset().begin(); it != mprselset().end(); it++)
    {
        Prels_mprsel_tuple* mprsel_tuple = *it;
        Prels_node_tuple* node_tuple = state_.find_node_tuple(mprsel_tuple->main_addr());
        int count = msg.tc().count;
        assert(count >= 0 && count < PRELS_MAX_ADDRS);
        msg.tc().nb_etx_main_addr(count).iface_address() = mprsel_tuple->main_addr();
        msg.tc().nb_etx_main_addr(count).mob_info() = node_tuple->mob_info();
        msg.tc().nb_etx_main_addr(count).mob_update_time() = node_tuple->mob_update_time();
        msg.tc().count++;
    }

    for (auto it = mprset().begin(); it != mprset().end(); it++)
    {
        nsaddr_t mpr_addr = *it;
        Prels_node_tuple* node_tuple = state_.find_node_tuple(mpr_addr);
        int count = msg.tc().count;
        assert(count >= 0 && count < PRELS_MAX_ADDRS);
        msg.tc().nb_etx_main_addr(count).iface_address() = mpr_addr;
        msg.tc().nb_etx_main_addr(count).mob_info() = node_tuple->mob_info();
        msg.tc().nb_etx_main_addr(count).mob_update_time() = node_tuple->mob_update_time();
        msg.tc().count++;
    }

    msg.msg_size() = msg.size();
    enque_msg(msg, JITTER);
}

// 发送队列中的信令
void
Prels::send_pkt()
{
    int num_msgs = msgs_.size();
    if (num_msgs == 0)
        return;

    // Calculates the number of needed packets
    int num_pkts = (num_msgs%PRELS_MAX_MSGS == 0) ? num_msgs/PRELS_MAX_MSGS :
                   (num_msgs/PRELS_MAX_MSGS + 1);

    for (int i = 0; i < num_pkts; i++)
    {
        const auto& op = makeShared<PrelsPkt>();

        op->setChunkLength(B(PRELS_PKT_HDR_SIZE));
        op->setPkt_seq_num(pkt_seq());

        int j = 0;
        for (auto it = msgs_.begin(); it != msgs_.end();)
        {
            if (j == PRELS_MAX_MSGS)
                break;

            op->setMsgArraySize(j+1);
            op->setMsg(j++, *it);
            op->setChunkLength(op->getChunkLength()+ B((*it).size()));

            it = msgs_.erase(it);
        }
        Packet *outPacket = new Packet("PRELS Pkt");
        outPacket->insertAtBack(op);

        auto sh = makeShared<SignalHeader>();
        sh->setPrevAddr(m_selfIpv4Address);
        int totalByteLength = sizeof(m_selfIpv4Address);
        sh->setChunkLength(B(totalByteLength));
        outPacket->insertAtFront(sh);

        setDownControlInfo(outPacket, MacAddress::BROADCAST_ADDRESS);
        sendDown(outPacket);
    }
}

bool
Prels::populate_nbset(PrelsMsg& msg)
{
    double now = CURRENT_TIME;
    Prels_hello& hello = msg.hello();

    Prels_nb_tuple* nb_tuple = state_.find_nb_tuple(msg.orig_addr());
    if (nb_tuple == nullptr)
    {
        // Creates associated neighbor tuple
        nb_tuple = new Prels_nb_tuple;
        nb_tuple->nb_main_addr() = msg.orig_addr();
        nb_tuple->nb_type = PRELS_SYM_NEIGH;
        add_nb_tuple(nb_tuple);

        nb_tuple->lost_time() = now + hello_ival();
//        Prels_NbTupleTimer* nb_timer = new Prels_NbTupleTimer(this, nb_tuple);
//        nb_timer->resched(DELAY(nb_tuple->lost_time()));
    }
    else
    {
        nb_tuple->lost_time() = now + hello_ival();  // 延长超时时刻
    }

    Prels_node_tuple* node_tuple = state_.find_node_tuple(msg.orig_addr());
    if(node_tuple == nullptr)
    {
        node_tuple = new Prels_node_tuple;
        node_tuple->mob_info() = hello.mob_info();
        node_tuple->mob_update_time() = hello.mob_update_time();
        node_tuple->main_addr() = msg.orig_addr();
        add_node_tuple(node_tuple);
    }
    else
    {
        if(node_tuple->mob_update_time() < hello.mob_update_time())
        {
            node_tuple->mob_info() = hello.mob_info();
            node_tuple->mob_update_time() = hello.mob_update_time();
        }
    }

    return false;
}

bool
Prels::populate_nb2hopset(PrelsMsg& msg)
{
    double now = CURRENT_TIME;
    Prels_hello& hello = msg.hello();

    assert(hello.count >= 0 && hello.count <= PRELS_MAX_HELLOS);
    for (int i = 0; i < hello.count; i++)
    {
        Prels_hello_msg& hello_msg = hello.hello_msg(i);
        int nb_type = hello_msg.nbtype_code();
        assert(hello_msg.count >= 0 && hello_msg.count <= PRELS_MAX_ADDRS);

        for (int j = 0; j < hello_msg.count; j++)
        {
            nsaddr_t nb2hop_addr = hello_msg.nb_iface_addr(j);
            if (nb2hop_addr != m_selfIpv4Address)
            {
                if (nb_type == PRELS_SYM_NEIGH || nb_type == PRELS_MPR_NEIGH)
                {
                    // Otherwise, a 2-hop tuple is created
                    Prels_nb2hop_tuple* nb2hop_tuple = state_.find_nb2hop_tuple(msg.orig_addr(), nb2hop_addr);
                    if (nb2hop_tuple == nullptr)
                    {
                        nb2hop_tuple = new Prels_nb2hop_tuple;
                        nb2hop_tuple->nb_main_addr() = msg.orig_addr();
                        nb2hop_tuple->nb2hop_addr() = nb2hop_addr;
                        add_nb2hop_tuple(nb2hop_tuple);
//                                nb2hop_tuple->time() =
//                                    now + Olsr::emf_to_seconds(msg.vtime());
                        // Schedules nb2hop tuple
                        // deletion
//                                Olsr_Nb2hopTupleTimer* nb2hop_timer =
//                                    new Olsr_Nb2hopTupleTimer(this, nb2hop_tuple);
//                                nb2hop_timer->resched(DELAY(nb2hop_tuple->time()));
                    }

//                    else
//                    {
//                                nb2hop_tuple->time() =
//                                    now + Olsr::emf_to_seconds(msg.vtime());
//                    }
                }
                else if (nb_type == PRELS_NOT_NEIGH)
                {
                    state_.erase_nb2hop_tuples(msg.orig_addr(), nb2hop_addr);
                }

                Prels_iface_address iface_addr = hello_msg.nb_etx_iface_addr(j);
                Prels_node_tuple* node_tuple = state_.find_node_tuple(nb2hop_addr);
                if(node_tuple == nullptr)
                {
                    node_tuple = new Prels_node_tuple;
                    node_tuple->mob_info() = iface_addr.mob_info();
                    node_tuple->mob_update_time() = iface_addr.mob_update_time();
                    node_tuple->main_addr() = nb2hop_addr;
                    add_node_tuple(node_tuple);
                }
                else
                {
                    if(node_tuple->mob_update_time() < iface_addr.mob_update_time())
                    {
                        node_tuple->mob_info() = iface_addr.mob_info();
                        node_tuple->mob_update_time() = iface_addr.mob_update_time();
                    }
                }

            }

        }
    }

    return false;
}

void
Prels::populate_mprselset(PrelsMsg& msg)
{
    double now = CURRENT_TIME;
    Prels_hello& hello = msg.hello();

    assert(hello.count >= 0 && hello.count <= PRELS_MAX_HELLOS);
    for (int i = 0; i < hello.count; i++)
    {
        Prels_hello_msg& hello_msg = hello.hello_msg(i);
        int nb_type = hello_msg.nbtype_code();
        if (nb_type == PRELS_MPR_NEIGH)
        {
            assert(hello_msg.count >= 0 && hello_msg.count <= PRELS_MAX_ADDRS);
            for (int j = 0; j < hello_msg.count; j++)
            {
                if (hello_msg.nb_iface_addr(j) == m_selfIpv4Address)
                {
                    // We must create a new entry into the mpr selector set
                    Prels_mprsel_tuple* mprsel_tuple = state_.find_mprsel_tuple(msg.orig_addr());
                    if (mprsel_tuple == nullptr)
                    {
                        mprsel_tuple = new Prels_mprsel_tuple;
                        mprsel_tuple->main_addr() = msg.orig_addr();
//                        mprsel_tuple->time() =
//                            now + Olsr::emf_to_seconds(msg.vtime());
                        add_mprsel_tuple(mprsel_tuple);
                        // Schedules mpr selector tuple deletion
//                        Olsr_MprSelTupleTimer* mprsel_timer =
//                            new Olsr_MprSelTupleTimer(this, mprsel_tuple);
//                        mprsel_timer->resched(DELAY(mprsel_tuple->time()));
                    }
//                    else
//                        mprsel_tuple->time() =
//                            now + Olsr::emf_to_seconds(msg.vtime());
                }
            }
        }
    }
}

int
Prels::degree(Prels_nb_tuple* tuple)
{
    int degree = 0;
    for (auto it = nb2hopset().begin(); it != nb2hopset().end(); it++)
    {
        Prels_nb2hop_tuple* nb2hop_tuple = *it;
        if (nb2hop_tuple->nb_main_addr() == tuple->nb_main_addr())
        {
            //OLSR_nb_tuple* nb_tuple =
            //    state_.find_nb_tuple(nb2hop_tuple->nb_main_addr());
            Prels_nb_tuple* nb_tuple = state_.find_nb_tuple(nb2hop_tuple->nb2hop_addr());
            if (nb_tuple == nullptr)
                degree++;
        }
    }
    return degree;
}

bool
Prels::seq_num_bigger_than(uint16_t s1, uint16_t s2)
{
    return (s1 > s2 && s1-s2 <= PRELS_MAX_SEQ_NUM/2)
           || (s2 > s1 && s2-s1 > PRELS_MAX_SEQ_NUM/2);
}

L3Address
Prels::getIfaceAddressFromIndex(int index)
{
    NetworkInterface * entry = ift->getInterface(index);
    return L3Address(entry->getProtocolData<Ipv4InterfaceData>()->getIPAddress());
}


}
