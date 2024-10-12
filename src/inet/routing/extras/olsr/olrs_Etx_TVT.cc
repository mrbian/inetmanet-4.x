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

namespace inet {
namespace inetmanet {

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


    std::map<uint8_t, int> linkcodes_count;
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



} /* namespace inetmanet */
} /* namespace inet */
