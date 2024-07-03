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

#ifndef INET_LINKLAYER_BLACKBOX_BLACKBOXROUTING_H_
#define INET_LINKLAYER_BLACKBOX_BLACKBOXROUTING_H_

#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3Address.h"

namespace inet {
namespace blackbox {

class BlackBoxRouting: public omnetpp::cSimpleModule {
public:
    BlackBoxRouting();
    virtual ~BlackBoxRouting();
protected:
    static simsignal_t updateTopology;
    virtual void initialize(int stage) override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual void modifyTopology();

};
}
} /* namespace inet */

#endif /* INET_LINKLAYER_BLACKBOX_BLACKBOXROUTING_H_ */
