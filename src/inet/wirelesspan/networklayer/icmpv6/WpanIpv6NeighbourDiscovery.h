/**
 * Copyright (C) 2005 Andras Varga
 * Copyright (C) 2005 Wei Yang, Ng
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __INET_WPANIPV6NEIGHBOURDISCOVERY_H
#define __INET_WPANIPV6NEIGHBOURDISCOVERY_H

#include <map>
#include <set>
#include <vector>

#include "inet/networklayer/icmpv6/Ipv6NeighbourDiscovery.h"

namespace inet {
namespace wirelesspan {

/**
 * Implements RFC 2461 Neighbor Discovery for Ipv6.
 */
class INET_API WpanIpv6NeighbourDiscovery : public Ipv6NeighbourDiscovery
{
  public:
    WpanIpv6NeighbourDiscovery();
    virtual ~WpanIpv6NeighbourDiscovery();

  protected:
    // CUSTOM PARAMETERS FOR WIRELESS NEIGHBOR DISCOVERY (WiND)
    double nsFwdDelay;
    bool pAddRandomDelays;
  protected:

    // CUSTOM CLASS REQUIRED FOR WiND
    class Ipv6NdPacketInfo : public cObject {
        public:
            Ipv6NdPacketInfo() {}
            Ipv6NdPacketInfo(Packet *pkt, const Ipv6Address &destAddr, const Ipv6Address &srcAddr, int ie)
            {
                this->msgPtr = pkt;
                this->destAddr = destAddr;
                this->srcAddr = srcAddr;
                this->interfaceId = ie;
            }

            const Ipv6Address& getDestAddr() const { return destAddr; }
            void setDestAddr(const Ipv6Address &destAddr) { this->destAddr = destAddr; }

            int getInterfaceId() const { return interfaceId; }
            void setInterfaceId(int interfaceId) { this->interfaceId = interfaceId; }

            Packet*& getMsgPtr() { return msgPtr; }
            void setMsgPtr(Packet *&msgPtr) { this->msgPtr = msgPtr; }

            const Ipv6Address& getSrcAddr() const { return srcAddr; }
            void setSrcAddr(const Ipv6Address &srcAddr) { this->srcAddr = srcAddr; }

        private:
            Packet *msgPtr;
            Ipv6Address destAddr;
            Ipv6Address srcAddr;
            int interfaceId;
    };
    // END CUSTOM CLASS


    virtual void sendPacketToIpv6Module(Packet *msg, const Ipv6Address& destAddr, const Ipv6Address& srcAddr, int interfaceId, double delay = 0);


    /** CUSTOM WIND PART **/
    bool isAppPacket(Packet *packet);
    double pRandomDelayMin = 0;
    double pRandomDelayMax = 0;

    // override methods from Ipv6NeighbourDiscovery
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void initiateNeighbourUnreachabilityDetection(Neighbour *nce) override;
    virtual void initiateAddressResolution(const Ipv6Address& dgSrcAddr, Neighbour *nce) override;
    virtual void processArTimeout(cMessage *arTimeoutMsg) override;
    virtual void assignLinkLocalAddress(cMessage *timerMsg) override;
    virtual void createRaTimer(NetworkInterface *ie) override;
    virtual void createAndSendNsPacket(const Ipv6Address& nsTargetAddr, const Ipv6Address& dgDestAddr,
            const Ipv6Address& dgSrcAddr, NetworkInterface *ie) override;
    virtual void processNaForIncompleteNceState(const Ipv6NeighbourAdvertisement *na, Neighbour *nce) override;
    virtual void processNaForOtherNceStates(const Ipv6NeighbourAdvertisement *na, Neighbour *nce) override;
};

}
} // namespace inet

#endif    //IPV6NEIGHBOURDISCOVERY_H

