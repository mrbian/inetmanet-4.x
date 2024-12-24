//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "inet/routing/extras/olsr/olrs_Etx_TVT.h"

#include <math.h>
#include <limits.h>

#include <omnetpp.h>
#include "inet/routing/extras/olsr/Olrs_Etx_dijkstra.h"
#include "inet/routing/extras/olsr/OlrsPkt_m.h"
#include "inet/node/base/NodeBase.h"

namespace inet {
namespace inetmanet {

//Register_Class(Olsr_Etx_TVT)
Define_Module(Olsr_Etx_TVT);

/// Length (in bytes) of UDP header.
#define UDP_HDR_LEN 8
/// Port Number
#define RT_PORT 698
#define IP_DEF_TTL 32


#define state_      (*state_etx_ptr)

Olsr_Etx_TVT ::Olsr_Etx_TVT () {
    // TODO Auto-generated constructor stub

}

Olsr_Etx_TVT ::~Olsr_Etx_TVT () {
    // TODO Auto-generated destructor stub
}

void
Olsr_Etx_TVT::initialize(int stage){
    Olsr_Etx::initialize(stage);
    if (stage == INITSTAGE_ROUTING_PROTOCOLS)
    {
        cModule* topModule = getModuleByPath("Net80211_aodv");
        int numHosts = topModule->par("numHosts");
        int numFixHosts = topModule->par("numFixHosts");
        char mob_path_str[100];
        char node_path_str[100];
        for(int i = 0; i < numHosts; i ++)
        {
            sprintf(mob_path_str, "Net80211_aodv.host[%d].mobility", i);
            sprintf(node_path_str, "Net80211_aodv.host[%d]", i);
            ExtendedBonnMotionMobility* mob = check_and_cast<ExtendedBonnMotionMobility*>(
                    getModuleByPath(mob_path_str));
            NodeBase *node = check_and_cast<NodeBase*>(
                    getModuleByPath(node_path_str));
            _globalMob.insert(std::make_pair(node->getId(), mob));
        }
        for (int i = 0; i < numFixHosts; i ++)
        {
            sprintf(mob_path_str, "Net80211_aodv.fixhost[%d].mobility", i);
            sprintf(node_path_str, "Net80211_aodv.fixhost[%d]", i);
            ExtendedBonnMotionMobility* mob = check_and_cast<ExtendedBonnMotionMobility*>(
                    getModuleByPath(mob_path_str));
            NodeBase *node = check_and_cast<NodeBase*>(
                    getModuleByPath(node_path_str));
            _globalMob.insert(std::make_pair(node->getId(), mob));
        }

        self_node_id = getContainingNode(this)->getId();
        load_losmap = par("load_losmap");
        Rmax = par("Rmax");
        pathloss_model = check_and_cast<physicallayer::FactoryFading*>(getModuleByPath("Net80211_aodv.radioMedium.pathLoss"));
    }
}


void
Olsr_Etx_TVT::send_hello()
{
    OlsrMsg msg;
    double now = CURRENT_TIME;
    msg.msg_type() = OLSR_HELLO_MSG;
    msg.vtime() = Olsr::seconds_to_emf(OLSR_NEIGHB_HOLD_TIME);
    msg.orig_addr() = ra_addr();
    msg.ttl() = 1;
    msg.hop_count() = 0;
    msg.msg_seq_num() = msg_seq();

    msg.hello().reserved() = 0;
    msg.hello().htime() = Olsr::seconds_to_emf(hello_ival());
    msg.hello().willingness() = willingness();
    msg.hello().count = 0;
    msg.hello().node_mob_info_ = self_node_id;


    std::map<uint8_t, int> linkcodes_count;   //  按照link_type对所有的邻居节点进行了分类，放到不同的hello_msg里面
                                              //  有利于接收端处理和减少hello消息的大小
    for (auto it = linkset().begin(); it != linkset().end(); it++)
    {
        Olsr_Etx_link_tuple* link_tuple = dynamic_cast<Olsr_Etx_link_tuple*>(*it);
        if (!link_tuple)
            throw cRuntimeError("\n Error conversion link tuple");


        if (link_tuple->local_iface_addr() == ra_addr() && link_tuple->time() >= now)
        {
            uint8_t link_type, nb_type, link_code;

            // Establishes link type
            if (use_mac() && link_tuple->lost_time() >= now)
                link_type = OLSR_LOST_LINK;
            else if (link_tuple->sym_time() >= now)
                link_type = OLSR_SYM_LINK;
            else if (link_tuple->asym_time() >= now)
                link_type = OLSR_ASYM_LINK;
            else
                link_type = OLSR_LOST_LINK;
            // Establishes neighbor type.
            if (state_.find_mpr_addr(get_main_addr(link_tuple->nb_iface_addr())))
                nb_type = OLSR_MPR_NEIGH;
            else
            {
                bool ok = false;
                for (auto nb_it = nbset().begin();
                        nb_it != nbset().end();
                        nb_it++)
                {
                    Olsr_nb_tuple* nb_tuple = *nb_it;
                    if (nb_tuple->nb_main_addr() == get_main_addr(link_tuple->nb_iface_addr()))
                    {
                        if (nb_tuple->getStatus() == OLSR_STATUS_SYM)
                            nb_type = OLSR_SYM_NEIGH;
                        else if (nb_tuple->getStatus() == OLSR_STATUS_NOT_SYM)
                            nb_type = OLSR_NOT_NEIGH;
                        else
                        {
                            throw cRuntimeError("There is a neighbor tuple with an unknown status!");
                        }
                        ok = true;
                        break;
                    }
                }
                if (!ok)
                {
                    EV_INFO << "I don't know the neighbor " << get_main_addr(link_tuple->nb_iface_addr()) << "!!! \n";
                    continue;
                }
            }

            int count = msg.hello().count;

            link_code = (link_type & 0x03) | ((nb_type << 2) & 0x0f);
            auto pos = linkcodes_count.find(link_code);
            if (pos == linkcodes_count.end())
            {
                linkcodes_count[link_code] = count;
                assert(count >= 0 && count < OLSR_MAX_HELLOS);
                msg.hello().hello_msg(count).count = 0;
                msg.hello().hello_msg(count).link_code() = link_code;
                msg.hello().hello_msg(count).reserved() = 0;
                msg.hello().hello_msg(count).set_qos_behaviour(parameter_.link_quality());
                msg.hello().count++;
            }
            else
                count = (*pos).second;

            int i = msg.hello().hello_msg(count).count;
            assert(count >= 0 && count < OLSR_MAX_HELLOS);
            assert(i >= 0 && i < OLSR_MAX_ADDRS);

            msg.hello().hello_msg(count).nb_iface_addr(i) =
                link_tuple->nb_iface_addr();


//////////////////////////////
/// Link Quality extensions
//////////////////////////////
            // publish link quality information we have found out and the one we
            // have received from our neighbors
            msg.hello().hello_msg(count).nb_etx_iface_addr(i).link_quality() =
                link_tuple->link_quality();
            msg.hello().hello_msg(count).nb_etx_iface_addr(i).nb_link_quality() =
                link_tuple->nb_link_quality();

            /// Link delay extension
            msg.hello().hello_msg(count).nb_etx_iface_addr(i).link_delay() =
                link_tuple->link_delay();
            msg.hello().hello_msg(count).nb_etx_iface_addr(i).nb_link_delay() =
                link_tuple->nb_link_delay();
// End link QoS

//////////////////////////////
/// Mobility extensions
//////////////////////////////



            msg.hello().hello_msg(count).count++;
            msg.hello().hello_msg(count).link_msg_size() =
                msg.hello().hello_msg(count).size();



        }
    }

    msg.msg_size() = msg.size();

    enque_msg(msg, JITTER);
}

bool
Olsr_Etx_TVT::link_sensing
(OlsrMsg& msg, const nsaddr_t &receiver_iface, const nsaddr_t &sender_iface,
 uint16_t pkt_seq_num, const int & index)
{
    Olsr_hello& hello = msg.hello();
    double now = CURRENT_TIME;
    bool updated = false;
    bool created = false;

    // 1 Upon receiving a HELLO message, if there exists no link tuple
    //   with L_neighbor_iface_addr == Source Address a new tuple is created with
    //           L_neighbor_iface_addr = Source Address
    //           L_local_iface_addr    = Address of the interface
    //                                   which received the
    //                                   HELLO message
    //           L_SYM_time            = current time - 1 (expired)
    //           L_time                = current time + validity time
    Olsr_Etx_link_tuple* link_tuple = nullptr;
    Olsr_link_tuple* link_tuple_aux = state_.find_link_tuple(sender_iface);
    if (link_tuple_aux)
    {
        link_tuple = dynamic_cast<Olsr_Etx_link_tuple*>(link_tuple_aux);
        if (link_tuple == nullptr)
            throw cRuntimeError("\n Error conversion link tuple");
    }
    if (link_tuple == nullptr)
    {
        // We have to create a new tuple
        link_tuple = new Olsr_Etx_link_tuple;
// Omnet
        link_tuple->set_qos_behaviour(parameter_);
        link_tuple->set_owner(this);
//
        link_tuple->nb_iface_addr() = sender_iface;
        link_tuple->local_iface_addr() = receiver_iface;
        link_tuple->sym_time() = now - 1;
        link_tuple->lost_time() = 0.0;
        link_tuple->time() = now + Olsr::emf_to_seconds(msg.vtime());
        // Init link quality information struct for this link tuple
//        link_tuple->link_quality_init(pkt_seq_num, DEFAULT_LOSS_WINDOW_SIZE);
        link_tuple->link_quality_init(pkt_seq_num, ETX_loss_window_size);
        /// Link delay extension
        link_tuple->link_delay_init();
        // This call will be also in charge of creating a new tuple in
        // the neighbor set
        add_link_tuple(link_tuple, hello.willingness());
        created = true;
    }
    else
        updated = true;

    // Link expiration time extension, update let firstly
    link_tuple->mob_info_idx_ = hello.node_mob_info_;
    link_tuple->mob_update_time_ = now;
    double let = predict_link_expire_time(link_tuple->mob_info_idx_);
    link_tuple->link_expire_time_ = let;

    // Account link quality information for this link
//    double link_expire_time = predict_link_expire_time(hello.node_mob_info_);
//    link_tuple->receive(pkt_seq_num, Olsr::emf_to_seconds(hello.htime()), link_expire_time);
    link_tuple->receive(pkt_seq_num, Olsr::emf_to_seconds(hello.htime()));

    // 2    The tuple (existing or new) with:
    //           L_neighbor_iface_addr == Source Address
    //      is then modified as follows:
    //      2.1  L_ASYM_time = current time + validity time;

    link_tuple->asym_time() = now + Olsr::emf_to_seconds(msg.vtime());
    assert(hello.count >= 0 && hello.count <= OLSR_ETX_MAX_HELLOS);
    for (int i = 0; i < hello.count; i++)
    {
        OLSR_ETX_hello_msg& hello_msg = hello.hello_msg(i);
        int lt = hello_msg.link_code() & 0x03;
        int nt = hello_msg.link_code() >> 2;

        // We must not process invalid advertised links
        if ((lt == OLSR_ETX_SYM_LINK && nt == OLSR_ETX_NOT_NEIGH) ||
                (nt != OLSR_ETX_SYM_NEIGH && nt != OLSR_ETX_MPR_NEIGH
                 && nt != OLSR_ETX_NOT_NEIGH))
            continue;

        assert(hello_msg.count >= 0 && hello_msg.count <= OLSR_ETX_MAX_ADDRS);
        for (int j = 0; j < hello_msg.count; j++)
        {
            //      2.2  if the node finds the address of the interface which
            //           received the HELLO message among the addresses listed in
            //           the link message then the tuple is modified as follows:
            if (hello_msg.nb_etx_iface_addr(j).iface_address() == receiver_iface)
            {
                //           2.2.1 if Link Type is equal to LOST_LINK then
                //                     L_SYM_time = current time - 1 (i.e., expired)
                if (lt == OLSR_ETX_LOST_LINK)
                {
                    link_tuple->sym_time() = now - 1;
                    updated = true;
                }
                //           2.2.2 else if Link Type is equal to SYM_LINK or ASYM_LINK
                //                then
                //                     L_SYM_time = current time + validity time,
                //                     L_time     = L_SYM_time + NEIGHB_HOLD_TIME
                else if (lt == OLSR_ETX_SYM_LINK || lt == OLSR_ETX_ASYM_LINK)
                {
                    link_tuple->sym_time() =
                        now + Olsr::emf_to_seconds(msg.vtime());
                    link_tuple->time() =
                        link_tuple->sym_time() + OLSR_ETX_NEIGHB_HOLD_TIME;
                    link_tuple->lost_time() = 0.0;
                    updated = true;
                }
                // Update our neighbor's idea of link quality and link delay
//                link_tuple->update_link_quality(hello_msg.nb_etx_iface_addr(j).link_quality(), link_expire_time);
                link_tuple->update_link_quality(hello_msg.nb_etx_iface_addr(j).link_quality());
                link_tuple->update_link_delay(hello_msg.nb_etx_iface_addr(j).link_delay());

                break;
            }
        }

    }

    //      2.3  L_time = max(L_time, L_ASYM_time)
    link_tuple->time() = MAX(link_tuple->time(), link_tuple->asym_time());

    if (updated)
        updated_link_tuple(link_tuple, hello.willingness());
    // Schedules link tuple deletion
    if (created && link_tuple != nullptr)
    {
        Olsr_LinkTupleTimer* link_timer =
            new Olsr_LinkTupleTimer(this, link_tuple);
        link_timer->resched(DELAY(MIN(link_tuple->time(), link_tuple->sym_time())));
    }
    return false;
}


void Olsr_Etx_TVT::send_tc()
{
    OlsrMsg msg;
    msg.msg_type() = OLSR_ETX_TC_MSG;
    msg.vtime() = Olsr::seconds_to_emf(OLSR_ETX_TOP_HOLD_TIME);
    msg.orig_addr() = ra_addr();
    if (parameter_.fish_eye())
    {
        msg.ttl() = tc_msg_ttl_[tc_msg_ttl_index_];
        tc_msg_ttl_index_ = (tc_msg_ttl_index_ + 1) % (MAX_TC_MSG_TTL);
    }
    else
    {
        msg.ttl() = 255;
    }
    msg.hop_count() = 0;
    msg.msg_seq_num() = msg_seq();

    msg.tc().ansn() = ansn_;
    msg.tc().reserved() = 0;
    msg.tc().count = 0;
    msg.tc().set_qos_behaviour(parameter_.link_quality());

    // we have to check which mpr selection algorithm is being used
    // prior to adding neighbors to the TC message being generated
    switch (parameter_.mpr_algorithm())
    {
        case OLSR_ETX_MPR_OLSRD:
            // Report all 1 hop neighbors we have
            for (auto it = nbset().begin(); it != nbset().end(); it++)
            {
                Olsr_nb_tuple* nb_tuple = *it;
                int count = msg.tc().count;
                Olsr_Etx_link_tuple *link_tuple;

                if (nb_tuple->getStatus() == OLSR_STATUS_SYM)
                {
                    assert(count >= 0 && count < OLSR_MAX_ADDRS);
                    link_tuple = state_.find_best_sym_link_tuple(nb_tuple->nb_main_addr(), CURRENT_TIME);
                    if (link_tuple != nullptr)
                    {
                        msg.tc().nb_etx_main_addr(count).iface_address() = nb_tuple->nb_main_addr();

                        // Report link quality and link link delay of the best link
                        // that we have to this node.
                        msg.tc().nb_etx_main_addr(count).link_quality() = link_tuple->link_quality();
                        msg.tc().nb_etx_main_addr(count).nb_link_quality() = link_tuple->nb_link_quality();
                        msg.tc().nb_etx_main_addr(count).link_delay() = link_tuple->link_delay();
                        msg.tc().nb_etx_main_addr(count).nb_link_delay() = link_tuple->nb_link_delay();

                        msg.tc().count++;
                    }
                }
            }
            break;

        default:
            //if (parameter_.tc_redundancy() & OLSR_ETX_TC_REDUNDANCY_MPR_SEL_SET) {
            // Reported by Mohamed Belhassen
            // we have to check which mpr selection algorithm is being used
            // prior to adding neighbors to the TC message being generated

            switch (parameter_.tc_redundancy())
            {
                case OLSR_ETX_TC_REDUNDANCY_MPR_SEL_SET:
                    // publish only nodes in mpr sel set (RFC 3626)
                    for (auto it = mprselset().begin(); it != mprselset().end(); it++)
                    {
                        OLSR_ETX_mprsel_tuple* mprsel_tuple = *it;
                        int count = msg.tc().count;
                        Olsr_Etx_link_tuple *link_tuple;

                        assert(count >= 0 && count < OLSR_MAX_ADDRS);
                        link_tuple = state_.find_best_sym_link_tuple(mprsel_tuple->main_addr(), CURRENT_TIME);
                        if (link_tuple != nullptr)
                        {
                            msg.tc().nb_etx_main_addr(count).iface_address() = mprsel_tuple->main_addr();

                            // Report link quality and link link delay of the best link
                            // that we have to this node.
                            msg.tc().nb_etx_main_addr(count).link_quality() = link_tuple->link_quality();
                            msg.tc().nb_etx_main_addr(count).nb_link_quality() = link_tuple->nb_link_quality();
                            msg.tc().nb_etx_main_addr(count).link_delay() = link_tuple->link_delay();
                            msg.tc().nb_etx_main_addr(count).nb_link_delay() = link_tuple->nb_link_delay();

                            msg.tc().count++;
                        }
                    }

                    break;

                case OLSR_ETX_TC_REDUNDANCY_MPR_SEL_SET_PLUS_MPR_SET:
                    // publish nodes in mpr sel set plus nodes in mpr set (RFC 3626)
                    for (auto it = mprselset().begin(); it != mprselset().end(); it++)
                    {
                        Olsr_mprsel_tuple* mprsel_tuple = *it;
                        int count = msg.tc().count;
                        Olsr_Etx_link_tuple *link_tuple;

                        assert(count >= 0 && count < OLSR_MAX_ADDRS);
                        link_tuple = state_.find_best_sym_link_tuple(mprsel_tuple->main_addr(), CURRENT_TIME);
                        if (link_tuple != nullptr)
                        {
                            msg.tc().nb_etx_main_addr(count).iface_address() = mprsel_tuple->main_addr();

                            // Report link quality and link link delay of the best link
                            // that we have to this node.
                            msg.tc().nb_etx_main_addr(count).link_quality() = link_tuple->link_quality();
                            msg.tc().nb_etx_main_addr(count).nb_link_quality() = link_tuple->nb_link_quality();
                            msg.tc().nb_etx_main_addr(count).link_delay() = link_tuple->link_delay();
                            msg.tc().nb_etx_main_addr(count).nb_link_delay() = link_tuple->nb_link_delay();

                            msg.tc().count++;
                        }
                    }

                    for (auto it = mprset().begin(); it != mprset().end(); it++)
                    {
                        nsaddr_t mpr_addr = *it;
                        int count = msg.tc().count;
                        Olsr_Etx_link_tuple *link_tuple;

                        assert(count >= 0 && count < OLSR_MAX_ADDRS);
                        link_tuple = state_.find_best_sym_link_tuple(mpr_addr, CURRENT_TIME);
                        if (link_tuple != nullptr)
                        {
                            msg.tc().nb_etx_main_addr(count).iface_address() = mpr_addr;

                            // Report link quality and link link delay of the best link
                            // that we have to this node.
                            msg.tc().nb_etx_main_addr(count).link_quality() = link_tuple->link_quality();
                            msg.tc().nb_etx_main_addr(count).nb_link_quality() = link_tuple->nb_link_quality();
                            msg.tc().nb_etx_main_addr(count).link_delay() = link_tuple->link_delay();
                            msg.tc().nb_etx_main_addr(count).nb_link_delay() = link_tuple->nb_link_delay();

                            msg.tc().count++;
                        }
                    }

                    break;

                case OLSR_ETX_TC_REDUNDANCY_FULL:
                    // publish full neighbor link set (RFC 3626)
                    for (auto it = nbset().begin(); it != nbset().end(); it++)
                    {
                        OLSR_ETX_nb_tuple* nb_tuple = *it;
                        int count = msg.tc().count;
                        Olsr_Etx_link_tuple *link_tuple;

                        if (nb_tuple->getStatus() == OLSR_STATUS_SYM)
                        {
                            assert(count >= 0 && count < OLSR_MAX_ADDRS);
                            link_tuple = state_.find_best_sym_link_tuple(nb_tuple->nb_main_addr(), CURRENT_TIME);
                            if (link_tuple != nullptr)
                            {
                                msg.tc().nb_etx_main_addr(count).iface_address() = nb_tuple->nb_main_addr();

                                // Report link quality and link link delay of the best link
                                // that we have to this node.
                                msg.tc().nb_etx_main_addr(count).link_quality() = link_tuple->link_quality();
                                msg.tc().nb_etx_main_addr(count).nb_link_quality() = link_tuple->nb_link_quality();
                                msg.tc().nb_etx_main_addr(count).link_delay() = link_tuple->link_delay();
                                msg.tc().nb_etx_main_addr(count).nb_link_delay() = link_tuple->nb_link_delay();

                                msg.tc().count++;
                            }
                        }
                    }

                    break;
                case OLSR_ETX_TC_REDUNDANCY_MPR_SET:
                    // non-OLSR standard: publish mpr set only
                    for (auto it = mprset().begin(); it != mprset().end(); it++)
                    {
                        nsaddr_t mpr_addr = *it;
                        int count = msg.tc().count;
                        Olsr_Etx_link_tuple *link_tuple;

                        assert(count >= 0 && count < OLSR_MAX_ADDRS);
                        link_tuple = state_.find_best_sym_link_tuple(mpr_addr, CURRENT_TIME);
                        if (link_tuple != nullptr)
                        {
                            msg.tc().nb_etx_main_addr(count).iface_address() = mpr_addr;

                            // Report link quality and link link delay of the best link
                            // that we have to this node.
                            msg.tc().nb_etx_main_addr(count).link_quality() = link_tuple->link_quality();
                            msg.tc().nb_etx_main_addr(count).nb_link_quality() = link_tuple->nb_link_quality();
                            msg.tc().nb_etx_main_addr(count).link_delay() = link_tuple->link_delay();
                            msg.tc().nb_etx_main_addr(count).nb_link_delay() = link_tuple->nb_link_delay();

                            msg.tc().count++;
                        }
                    }
                    break;
            }
            break;
    }

    msg.msg_size() = msg.size();
    enque_msg(msg, JITTER);
}


bool
Olsr_Etx_TVT::process_tc(OlsrMsg& msg, const nsaddr_t &sender_iface, const int &index)
{
    assert(msg.msg_type() == OLSR_ETX_TC_MSG);
    double now = CURRENT_TIME;
    Olsr_tc& tc = msg.tc();

    // 1. If the sender interface of this message is not in the symmetric
    // 1-hop neighborhood of this node, the message MUST be discarded.
    Olsr_Etx_link_tuple* link_tuple = nullptr;
    Olsr_link_tuple *tuple_aux = state_.find_sym_link_tuple(sender_iface, now);
    if (tuple_aux)
    {
        link_tuple = dynamic_cast<Olsr_Etx_link_tuple*> (tuple_aux);
        if (!link_tuple)
            throw cRuntimeError("\n Error conversion link tuple");
    }

    if (link_tuple == nullptr)
        return false;
    // 2. If there exist some tuple in the topology set where:
    //   T_last_addr == originator address AND
    //   T_seq       >  ANSN,
    // then further processing of this TC message MUST NOT be
    // performed. This might be a message received out of order.
    OLSR_ETX_topology_tuple* topology_tuple = nullptr;
    Olsr_topology_tuple* topology_tuple_aux = state_.find_newer_topology_tuple(msg.orig_addr(), tc.ansn());
    if (topology_tuple_aux)
    {
        topology_tuple = dynamic_cast<OLSR_ETX_topology_tuple*> (topology_tuple_aux);
        if (!topology_tuple)
            throw cRuntimeError("\n error conversion Topology tuple");
    }

    if (topology_tuple != nullptr)
        return false;

    // 3. All tuples in the topology set where:
    //  T_last_addr == originator address AND
    //  T_seq       <  ANSN
    // MUST be removed from the topology set.
    state_.erase_older_topology_tuples(msg.orig_addr(), tc.ansn());

    // 4. For each of the advertised neighbor main address received in
    // the TC message:
    for (int i = 0; i < tc.count; i++)
    {
        assert(i >= 0 && i < OLSR_ETX_MAX_ADDRS);
        nsaddr_t addr = tc.nb_etx_main_addr(i).iface_address();
        // 4.1. If there exist some tuple in the topology set where:
        //   T_dest_addr == advertised neighbor main address, AND
        //   T_last_addr == originator address,
        // then the holding time of that tuple MUST be set to:
        //   T_time      =  current time + validity time.
        OLSR_ETX_topology_tuple* topology_tuple = nullptr;
        Olsr_topology_tuple* topology_tuple_aux = state_.find_topology_tuple(addr, msg.orig_addr());
        if (topology_tuple_aux)
        {
            topology_tuple = dynamic_cast<OLSR_ETX_topology_tuple*> (topology_tuple_aux);
            if (!topology_tuple)
                throw cRuntimeError("\n error conversion Topology tuple");
        }


        if (topology_tuple != nullptr)
            topology_tuple->time() = now + Olsr::emf_to_seconds(msg.vtime());
        // 4.2. Otherwise, a new tuple MUST be recorded in the topology
        // set where:
        //  T_dest_addr = advertised neighbor main address,
        //  T_last_addr = originator address,
        //  T_seq       = ANSN,
        //  T_time      = current time + validity time.
        else
        {
            topology_tuple = new OLSR_ETX_topology_tuple;
            topology_tuple->dest_addr() = addr;
            topology_tuple->last_addr() = msg.orig_addr();
            topology_tuple->seq() = tc.ansn();
            topology_tuple->set_qos_behaviour(parameter_.link_quality());
            topology_tuple->time() = now + Olsr::emf_to_seconds(msg.vtime());
            add_topology_tuple(topology_tuple);
            // Schedules topology tuple deletion
            Olsr_TopologyTupleTimer* topology_timer =
                new Olsr_TopologyTupleTimer(this, topology_tuple);
            topology_timer->resched(DELAY(topology_tuple->time()));
        }
        // Update link quality and link delay information

        topology_tuple->update_link_quality(tc.nb_etx_main_addr(i).link_quality(),
                                            tc.nb_etx_main_addr(i).nb_link_quality());

        topology_tuple->update_link_delay(tc.nb_etx_main_addr(i).link_delay(),
                                          tc.nb_etx_main_addr(i).nb_link_delay());
    }
    return false;
}



bool
Olsr_Etx_TVT::populate_nb2hopset(OlsrMsg& msg)
{
    Olsr_hello& hello = msg.hello();
    double now = CURRENT_TIME;

    // Upon receiving a HELLO message, the "validity time" MUST be computed
    // from the Vtime field of the message header (see section 3.3.2).
    double validity_time = now + Olsr::emf_to_seconds(msg.vtime());

    //  If the Originator Address is the main address of a
    //  L_neighbor_iface_addr from a link tuple included in the Link Set with
    //         L_SYM_time >= current time (not expired)
    //  then the 2-hop Neighbor Set SHOULD be updated as follows:
    for (auto it_lt = linkset().begin(); it_lt != linkset().end(); it_lt++)
    {
        Olsr_Etx_link_tuple* link_tuple = dynamic_cast<Olsr_Etx_link_tuple*>(*it_lt);
        if (!link_tuple)
            throw cRuntimeError("\n Error conversion link tuple");



        if (get_main_addr(link_tuple->nb_iface_addr()) == msg.orig_addr() &&
                link_tuple->sym_time() >= now)
        {
            assert(hello.count >= 0 && hello.count <= OLSR_ETX_MAX_HELLOS);

            for (int i = 0; i < hello.count; i++)
            {
                OLSR_ETX_hello_msg& hello_msg = hello.hello_msg(i);
                int nt = hello_msg.link_code() >> 2;
                assert(hello_msg.count >= 0 && hello_msg.count <= OLSR_ETX_MAX_ADDRS);

                // 1    for each address (henceforth: 2-hop neighbor address), listed
                //      in the HELLO message with Neighbor Type equal to SYM_NEIGH or
                //      MPR_NEIGH:
                if (nt == OLSR_ETX_SYM_NEIGH || nt == OLSR_ETX_MPR_NEIGH)
                {

                    for (int j = 0; j < hello_msg.count; j++)
                    {
                        // Weverton Cordeiro: was not verifying 2hop main addr
                        nsaddr_t nb2hop_addr = get_main_addr(hello_msg.nb_etx_iface_addr(j).iface_address());

                        // 1.1  if the main address of the 2-hop neighbor address = main
                        // address of the receiving node: silently discard the 2-hop neighbor address.
                        if (nb2hop_addr == ra_addr())
                            continue;
                        // 1.2  Otherwise, a 2-hop tuple is created with:
                        // N_neighbor_main_addr =  Originator Address;
                        // N_2hop_addr          =  main address of the 2-hop neighbor;
                        // N_time               =  current time + validity time.
                        OLSR_ETX_nb2hop_tuple* nb2hop_tuple = nullptr;
                        Olsr_nb2hop_tuple* nb2hop_tuple_aux =
                            state_.find_nb2hop_tuple(msg.orig_addr(), nb2hop_addr);
                        if (nb2hop_tuple_aux)
                        {
                            nb2hop_tuple = dynamic_cast<OLSR_ETX_nb2hop_tuple*>(nb2hop_tuple_aux);
                            if (!nb2hop_tuple)
                                throw cRuntimeError("\n Error conversion nd2hop tuple");
                        }

                        if (nb2hop_tuple == nullptr)
                        {
                            nb2hop_tuple = new OLSR_ETX_nb2hop_tuple;
                            nb2hop_tuple->nb_main_addr() = msg.orig_addr();
                            nb2hop_tuple->nb2hop_addr() = nb2hop_addr;
                            nb2hop_tuple->set_qos_behaviour(parameter_.link_quality());

                            // Init link quality and link delay information
                            nb2hop_tuple->update_link_quality(0.0, 0.0);
                            nb2hop_tuple->update_link_delay(1.0, 1.0);

                            add_nb2hop_tuple(nb2hop_tuple);
                            nb2hop_tuple->time() = validity_time;
                            // Schedules nb2hop tuple deletion
                            Olsr_Nb2hopTupleTimer* nb2hop_timer =
                                new Olsr_Nb2hopTupleTimer(this, nb2hop_tuple);
                            nb2hop_timer->resched(DELAY(nb2hop_tuple->time()));
                        }
                        else
                            // This tuple may replace an older similar tuple with same
                            // N_neighbor_main_addr and N_2hop_addr values.
                            nb2hop_tuple->time() = validity_time;

                        // Update Link Quality information. Note: we only want information about the best link
                        switch (parameter_.link_quality())
                        {
                        case OLSR_ETX_BEHAVIOR_ETX:
                        case OLSR_ETX_BEHAVIOR_LET:
                            nb2hop_tuple->update_link_quality(
                                hello_msg.nb_etx_iface_addr(j).link_quality(),
                                hello_msg.nb_etx_iface_addr(j).nb_link_quality());
                            break;

                        case OLSR_ETX_BEHAVIOR_ML:
                            nb2hop_tuple->update_link_quality(
                                hello_msg.nb_etx_iface_addr(j).link_quality(),
                                hello_msg.nb_etx_iface_addr(j).nb_link_quality());
                            break;

                        case OLSR_ETX_BEHAVIOR_NONE:
                        default:
                            //
                            break;
                        }

                        nb2hop_tuple->update_link_delay(
                                                        hello_msg.nb_etx_iface_addr(j).link_delay(),
                                                        hello_msg.nb_etx_iface_addr(j).nb_link_delay());

                    }
                }
                // 2 For each 2-hop node listed in the HELLO message with Neighbor
                //   Type equal to NOT_NEIGH, all 2-hop tuples where:
                else if (nt == OLSR_ETX_NOT_NEIGH)
                {

                    for (int j = 0; j < hello_msg.count; j++)
                    {
                        nsaddr_t nb2hop_addr = get_main_addr(hello_msg.nb_etx_iface_addr(j).iface_address());

                        state_.erase_nb2hop_tuples(msg.orig_addr(), nb2hop_addr);
                    }
                }
            }
            // this hello message was already processed, and processing it for another symmetric
            // link we find in our link set will not make any new changes
            break;
        }
    }
    return false;
}


double
Olsr_Etx_TVT::predict_link_expire_time(int node_mob_info)
{
    double link_expire_time = 0;
    int Np = 60;
    double period = 0.1;
    for(int k = 0; k < Np; k ++)
    {
        Coord PosA = forecast_node_position_optimal(self_node_id, k*period);
        Coord PosB = forecast_node_position_optimal(node_mob_info, k*period);
        double distance = PosA.distance(PosB);
        double commRange;
        if(load_losmap)
        {
            commRange = get_commrange_for_link_expire_time(PosA, PosB);
        }
        else
        {
            commRange = get_commrange_for_link_expire_time();
        }
        if(distance <= commRange)
        {
            link_expire_time += period;
        }
        else
        {
            break;
        }
    }
    return link_expire_time;
}

Coord
Olsr_Etx_TVT::forecast_node_position_optimal(int node_id, double duration)
{
    ExtendedBonnMotionMobility *mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob.find(node_id)->second);
    Coord fut = mob->getFuturePosition(duration, simTime());
    return fut;
}

double
Olsr_Etx_TVT::get_commrange_for_link_expire_time()
{
    return Rmax;
}

double
Olsr_Etx_TVT::get_commrange_for_link_expire_time(Coord PosA, Coord PosB)
{
    bool nlos_cond = pathloss_model->checkNlos(PosA, PosB);
    if(nlos_cond)
    {
        return 50;
    }
    else
    {
        return 123;
    }
}

bool
Olsr_Etx_TVT::process_hello(OlsrMsg& msg, const nsaddr_t &receiver_iface, const nsaddr_t &sender_iface, uint16_t pkt_seq_num, const int &index)
{
    assert(msg.msg_type() == OLSR_ETX_HELLO_MSG);
    link_sensing(msg, receiver_iface, sender_iface, pkt_seq_num, index);
    populate_nbset(msg);
    populate_nb2hopset(msg);
    switch (parameter_.mpr_algorithm())
    {
    case OLSR_ETX_MPR_R1:
        olsr_r1_mpr_computation();
        break;
    case OLSR_ETX_MPR_R2:
        olsr_r2_mpr_computation();
        break;
    case OLSR_ETX_MPR_QOLSR:
        qolsr_mpr_computation();
        break;
    case OLSR_ETX_MPR_OLSRD:
        olsrd_mpr_computation();
        break;
    case OLSR_ETX_MPR_TVT:
        olsr_mpr_computation_TVT();
        break;
    case OLSR_ETX_DEFAULT_MPR:
    default:
        olsr_mpr_computation();
        break;
    }
    populate_mprselset(msg);
    return false;
}


// inherit from olsr_r1_mpr_computation
void
Olsr_Etx_TVT::olsr_mpr_computation_TVT()
{
    // For further details please refer to paper
    // Quality of Service Routing in Ad Hoc Networks Using OLSR

    bool increment;
    state_.clear_mprset();

    nbset_t N; nb2hopset_t N2;
    // N is the subset of neighbors of the node, which are
    // neighbor "of the interface I" and have willigness different
    // from OLSR_ETX_WILL_NEVER
    for (auto it = nbset().begin(); it != nbset().end(); it++)
    {
        Olsr_Etx_link_tuple *nb_link_tuple = nullptr;
        Olsr_link_tuple * link_tuple_aux;
        double now = CURRENT_TIME;
        link_tuple_aux = state_.find_sym_link_tuple((*it)->nb_main_addr(), now);
        if (link_tuple_aux)
        {
            nb_link_tuple = dynamic_cast<Olsr_Etx_link_tuple *>(link_tuple_aux);
            if (!nb_link_tuple)
                throw cRuntimeError("\n Error conversion link tuple");

            if ((*it)->getStatus() == OLSR_ETX_STATUS_SYM) // I think that we need this check
            {
                if(nb_link_tuple->mob_update_time_ + nb_link_tuple->link_expire_time_ > now)
                {
                    N.push_back(*it);
                }
            }
        }

    }

    // N2 is the set of 2-hop neighbors reachable from "the interface
    // I", excluding:
    // (i)   the nodes only reachable by members of N with willingness WILL_NEVER
    // (ii)  the node performing the computation
    // (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
    //       link to this node on some interface.
    for (auto it = nb2hopset().begin(); it != nb2hopset().end(); it++)
    {
        OLSR_ETX_nb2hop_tuple* nb2hop_tuple = dynamic_cast<OLSR_ETX_nb2hop_tuple*>( *it);


        if (!nb2hop_tuple)
            throw cRuntimeError("\n Error conversion nd2hop tuple");

        if (isLocalAddress(nb2hop_tuple->nb2hop_addr()))
        {
            continue;
        }
        // excluding:
        // (i) the nodes only reachable by members of N with willingness WILL_NEVER
        bool ok = false;
        for (nbset_t::const_iterator it2 = N.begin(); it2 != N.end(); it2++)
        {
            Olsr_nb_tuple* neigh = *it2;
            if (neigh->nb_main_addr() == nb2hop_tuple->nb_main_addr())
            {
                if (neigh->willingness() == OLSR_WILL_NEVER)
                {
                    ok = false;
                    break;
                }
                else
                {
                    ok = true;
                    break;
                }
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
            OLSR_ETX_nb_tuple* neigh =dynamic_cast<OLSR_ETX_nb_tuple*>(*it2);
            if (neigh == nullptr)
                throw cRuntimeError("Error in tupe");
            if (neigh->nb_main_addr() == nb2hop_tuple->nb2hop_addr())
            {
                ok = false;
                break;
            }
        }

        if (ok)
            N2.push_back(nb2hop_tuple);
    }

    // Start with an MPR set made of all members of N with
    // N_willingness equal to WILL_ALWAYS
    for (auto it = N.begin(); it != N.end(); it++)
    {
        OLSR_ETX_nb_tuple* nb_tuple = *it;
        if (nb_tuple->willingness() == OLSR_ETX_WILL_ALWAYS)
            state_.insert_mpr_addr(nb_tuple->nb_main_addr());
    }

    // Add to Mi the nodes in N which are the only nodes to provide reachability
    // to a node in N2. Remove the nodes from N2 which are now covered by
    // a node in the MPR set.
    mprset_t foundset;
    std::set<nsaddr_t> deleted_addrs;
    // iterate through all 2 hop neighbors we have
    for (auto it = N2.begin(); it != N2.end();)
    {
        OLSR_ETX_nb2hop_tuple* nb2hop_tuple1 = dynamic_cast<OLSR_ETX_nb2hop_tuple*>(*it);
        if (!nb2hop_tuple1)
            throw cRuntimeError("\n Error conversion nd2hop tuple");

        increment = true;

        // check if this two hop neighbor has more that one hop neighbor in N
        // it would mean that there is more than one node in N that reaches
        // the current 2 hop node
        auto pos = foundset.find(nb2hop_tuple1->nb2hop_addr());
        if (pos != foundset.end())
        {
            it++;
            continue;
        }

        bool found = false;
        // find the one hop neighbor that provides reachability to the
        // current two hop neighbor.
        for (auto it2 = N.begin(); it2 != N.end(); it2++)
        {
            if ((*it2)->nb_main_addr() == nb2hop_tuple1->nb_main_addr())
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            it++;
            continue;
        }

        found = false;
        // check if there is another one hop neighbor able to provide
        // reachability to the current 2 hop neighbor
        for (auto it2 = it + 1; it2 != N2.end(); it2++)
        {
            OLSR_ETX_nb2hop_tuple* nb2hop_tuple2 = dynamic_cast<OLSR_ETX_nb2hop_tuple*>(*it2);
            if (!nb2hop_tuple2)
                throw cRuntimeError("\n Error conversion nd2hop tuple");

            if (nb2hop_tuple1->nb2hop_addr() == nb2hop_tuple2->nb2hop_addr())
            {
                foundset.insert(nb2hop_tuple1->nb2hop_addr());
                found = true;
                break;
            }
        }
        // if there is only one node, add our one hop neighbor to the MPR set
        if (!found)
        {
            state_.insert_mpr_addr(nb2hop_tuple1->nb_main_addr());

            // erase all 2 hop neighbor nodes that are now reached through this
            // newly added MPR
            for (auto it2 = it + 1; it2 != N2.end();)
            {
                OLSR_ETX_nb2hop_tuple* nb2hop_tuple2 = dynamic_cast<OLSR_ETX_nb2hop_tuple*>(*it2);
                if (!nb2hop_tuple2)
                    throw cRuntimeError("\n Error conversion nd2hop tuple");

                if (nb2hop_tuple1->nb_main_addr() == nb2hop_tuple2->nb_main_addr())
                {
                    deleted_addrs.insert(nb2hop_tuple2->nb2hop_addr());
                    it2 = N2.erase(it2);
                }
                else
                    it2++;
            }
            int distanceFromEnd = std::distance(it, N2.end());
            int distance = std::distance(N2.begin(), it);
            int i = 0;
            for (auto it2 = N2.begin(); i < distance; i++) // check now the first section
            {

                Olsr_nb2hop_tuple* nb2hop_tuple2 = *it2;
                if (nb2hop_tuple1->nb_main_addr() == nb2hop_tuple2->nb_main_addr())
                {
                    deleted_addrs.insert(nb2hop_tuple2->nb2hop_addr());
                    it2 = N2.erase(it2);
                }
                else
                    it2++;

            }
            it = N2.end() - distanceFromEnd; // the standard doesn't guarantee that the iterator is valid if we have delete something in the vector, reload the iterator.

            it = N2.erase(it);
            increment = false;
        }

        // erase all 2 hop neighbor nodes that are now reached through this
        // newly added MPR. We are now looking for the backup links
        for (auto it2 = deleted_addrs.begin();
                it2 != deleted_addrs.end(); it2++)
        {
            for (auto it3 = N2.begin(); it3 != N2.end();)
            {
                if ((*it3)->nb2hop_addr() == *it2)
                {
                    it3 = N2.erase(it3);
                    // I have to reset the external iterator because it
                    // may have been invalidated by the latter deletion
                    it = N2.begin();
                    increment = false;
                }
                else
                    it3++;
            }
        }
        deleted_addrs.clear();
        if (increment)
            it++;
    }

    // While there exist nodes in N2 which are not covered by at
    // least one node in the MPR set:
    while (N2.begin() != N2.end())
    {
        // For each node in N, calculate the reachability, i.e., the
        // number of nodes in N2 that it can reach
        std::map<int, std::vector<OLSR_ETX_nb_tuple*> > reachability;
        std::set<int> rs;
        for (auto it = N.begin(); it != N.end(); it++)
        {
            OLSR_ETX_nb_tuple* nb_tuple = *it;
            int r = 0;
            for (auto it2 = N2.begin(); it2 != N2.end(); it2++)
            {
                OLSR_ETX_nb2hop_tuple* nb2hop_tuple = dynamic_cast<OLSR_ETX_nb2hop_tuple*>(*it2);
                if (!nb2hop_tuple)
                    throw cRuntimeError("\n Error conversion nd2hop tuple");


                if (nb_tuple->nb_main_addr() == nb2hop_tuple->nb_main_addr())
                    r++;
            }
            rs.insert(r);
            reachability[r].push_back(nb_tuple);
        }

        // Select as a MPR the node with highest N_willingness among
        // the nodes in N with non-zero reachability. In case of
        // multiple choice select the node which provides
        // reachability to the maximum number of nodes in N2. In
        // case of multiple choices select the node with best conectivity
        // to the current node. Remove the nodes from N2 which are now covered
        // by a node in the MPR set.
        OLSR_ETX_nb_tuple* max = nullptr;
        int max_r = 0;
        for (auto it = rs.begin(); it != rs.end(); it++)
        {
            int r = *it;
            if (r > 0)
            {
                for (auto it2 = reachability[r].begin();
                        it2 != reachability[r].end(); it2++)
                {
                    OLSR_ETX_nb_tuple* nb_tuple = *it2;
                    if (max == nullptr || nb_tuple->willingness() > max->willingness())
                    {
                        max = nb_tuple;
                        max_r = r;
                    }
                    else if (nb_tuple->willingness() == max->willingness())
                    {
                        if (r > max_r)
                        {
                            max = nb_tuple;
                            max_r = r;
                        }
                        else if (r == max_r)
                        {
                            Olsr_Etx_link_tuple *nb_link_tuple = nullptr, *max_link_tuple = nullptr;
                            Olsr_link_tuple * link_tuple_aux;
                            double now = CURRENT_TIME;
                            link_tuple_aux = state_.find_sym_link_tuple(nb_tuple->nb_main_addr(), now);
                            if (link_tuple_aux)
                            {
                                nb_link_tuple = dynamic_cast<Olsr_Etx_link_tuple *>(link_tuple_aux);
                                if (!nb_link_tuple)
                                    throw cRuntimeError("\n Error conversion link tuple");
                            }
                            link_tuple_aux = state_.find_sym_link_tuple(max->nb_main_addr(), now);
                            if (link_tuple_aux)
                            {
                                max_link_tuple = dynamic_cast<Olsr_Etx_link_tuple *>(link_tuple_aux);
                                if (!max_link_tuple)
                                    throw cRuntimeError("\n Error conversion link tuple");
                            }
                            if (parameter_.link_delay())
                            {
                                if (nb_link_tuple == nullptr)
                                    continue;
                                else if (nb_link_tuple != nullptr && max_link_tuple == nullptr)
                                {
                                    max = nb_tuple;
                                    max_r = r;
                                    continue;
                                }
                                if (nb_link_tuple->link_delay() < max_link_tuple->link_delay())
                                {
                                    max = nb_tuple;
                                    max_r = r;
                                }
                            }
                            else
                            {
                                switch (parameter_.link_quality())
                                {
                                case OLSR_ETX_BEHAVIOR_ETX:
                                    if (nb_link_tuple == nullptr)
                                        continue;
                                    else if (nb_link_tuple != nullptr && max_link_tuple == nullptr)
                                    {
                                        max = nb_tuple;
                                        max_r = r;
                                        continue;
                                    }
                                    if (nb_link_tuple->etx() < max_link_tuple->etx())
                                    {
                                        max = nb_tuple;
                                        max_r = r;
                                    }
                                    break;

                                case OLSR_ETX_BEHAVIOR_LET:
                                    if (nb_link_tuple == nullptr)
                                        continue;
                                    else if (nb_link_tuple != nullptr && max_link_tuple == nullptr)
                                    {
                                        max = nb_tuple;
                                        max_r = r;
                                        continue;
                                    }
                                    if (nb_link_tuple->improved_ext(now) < max_link_tuple->improved_ext(now))
                                    {
                                        max = nb_tuple;
                                        max_r = r;
                                    }
                                    break;

                                case OLSR_ETX_BEHAVIOR_ML:
                                    if (nb_link_tuple == nullptr)
                                        continue;
                                    else if (nb_link_tuple != nullptr && max_link_tuple == nullptr)
                                    {
                                        max = nb_tuple;
                                        max_r = r;
                                        continue;
                                    }
                                    if (nb_link_tuple->etx() > max_link_tuple->etx())
                                    {
                                        max = nb_tuple;
                                        max_r = r;
                                    }
                                    break;
                                case OLSR_ETX_BEHAVIOR_NONE:
                                default:
                                    // max = nb_tuple;
                                    // max_r = r;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (max != nullptr)
        {
            state_.insert_mpr_addr(max->nb_main_addr());
            std::set<nsaddr_t> nb2hop_addrs;
            for (auto it = N2.begin(); it != N2.end();)
            {
                OLSR_ETX_nb2hop_tuple* nb2hop_tuple = dynamic_cast<OLSR_ETX_nb2hop_tuple*>(*it);
                if (!nb2hop_tuple)
                    throw cRuntimeError("\n Error conversion nd2hop tuple");


                if (nb2hop_tuple->nb_main_addr() == max->nb_main_addr())
                {
                    nb2hop_addrs.insert(nb2hop_tuple->nb2hop_addr());
                    it = N2.erase(it);
                }
                else
                    it++;
            }
            for (auto it = N2.begin(); it != N2.end();)
            {
                OLSR_ETX_nb2hop_tuple* nb2hop_tuple = dynamic_cast<OLSR_ETX_nb2hop_tuple*>(*it);
                if (!nb2hop_tuple)
                    throw cRuntimeError("\n Error conversion nd2hop tuple");


                auto it2 =
                    nb2hop_addrs.find(nb2hop_tuple->nb2hop_addr());
                if (it2 != nb2hop_addrs.end())
                {
                    it = N2.erase(it);
                }
                else
                    it++;

            }
        }
    }
}



} /* namespace inetmanet */
} /* namespace inet */
