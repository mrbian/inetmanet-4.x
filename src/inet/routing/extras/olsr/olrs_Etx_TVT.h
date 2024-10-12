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

#ifndef INET_ROUTING_EXTRAS_OLSR_OLRS_ETX_TVT_H_
#define INET_ROUTING_EXTRAS_OLSR_OLRS_ETX_TVT_H_

#include "inet/routing/extras/olsr/Olrs_Etx.h"
#include "inet/routing/extras/olsr/Olrs_Etx_parameter.h"
#include "inet/routing/extras/olsr/Olrs_Etx_repositories.h"
#include "inet/routing/extras/olsr/Olrs_Etx_state.h"

namespace inet {
namespace inetmanet {

class Olsr_Etx_TVT : public Olsr_Etx {

    friend class Olsr_Etx_LinkQualityTimer;
    friend class Olsr_HelloTimer;
    friend class Olsr_TcTimer;
    friend class Olsr_MidTimer;
    friend class Olsr_DupTupleTimer;
    friend class Olsr_LinkTupleTimer;
    friend class Olsr_Nb2hopTupleTimer;
    friend class Olsr_MprSelTupleTimer;
    friend class Olsr_TopologyTupleTimer;
    friend class Olsr_IfaceAssocTupleTimer;
    friend class Olsr_MsgTimer;
    friend class Olsr_Etx_state;
    friend class Dijkstra;
    Olsr_Etx_parameter parameter_;

protected:
    Olsr_Etx_state *state_etx_ptr;

public:
    Olsr_Etx_TVT ();
    virtual ~Olsr_Etx_TVT ();


    virtual void send_hello() override;

};

} /* namespace inetmanet */
} /* namespace inet */

#endif /* INET_ROUTING_EXTRAS_OLSR_OLRS_ETX_TVT_H_ */
