//
// Copyright (C) 2014 OpenSim Ltd.
// Author: Benjamin Seregi
// Copyright (C) 2019 Universidad de Malaga
// Author: Alfonso Ariza
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


#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/packet/Packet.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/networklayer/common/HopLimitTag_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/ipv4/IcmpHeader.h"
#include "inet/networklayer/ipv4/Ipv4Header_m.h"
#include "inet/networklayer/ipv4/Ipv4Route.h"
#include "inet/wirelesspan/routing/loadng/LoadNgBase.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/common/packet/dissector/PacketDissector.h"


namespace inet {
namespace wirelesspan {
namespace routing {


Define_Module(LoadNgBase);


inline std::ostream& operator<<(std::ostream& out, const int64_t& d)
{
    out << "Seq num = " << std::to_string(d);
    return out;
}


void LoadNgBase::initialize(int stage)
{
    if (stage == INITSTAGE_ROUTING_PROTOCOLS)
        addressType = getSelfIPAddress().getAddressType();  // needed for handleStartOperation()

    RoutingProtocolBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        lastBroadcastTime = SIMTIME_ZERO;
        rebootTime = SIMTIME_ZERO;
        rreqId = sequenceNum = 0;
        rreqCount = rerrCount = 0;
        host = getContainingNode(this);
        routingTable = getModuleFromPar<IRoutingTable>(par("routingTableModule"), this);
        interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        networkProtocol = getModuleFromPar<INetfilter>(par("networkProtocolModule"), this);

        aodvUDPPort = par("udpPort");
        activeRouteTimeout = par("activeRouteTimeout");
        netDiameter = par("netDiameter");
        nodeTraversalTime = par("nodeTraversalTime");
        rerrRatelimit = par("rerrRatelimit");
        rreqRetries = par("rreqRetries");
        rreqRatelimit = par("rreqRatelimit");
        timeoutBuffer = par("timeoutBuffer");
        ttlStart = par("ttlStart");
        ttlIncrement = par("ttlIncrement");
        ttlThreshold = par("ttlThreshold");
        localAddTTL = par("localAddTTL");
        jitterPar = &par("jitter");

        myRouteTimeout = par("myRouteTimeout");
        deletePeriod = par("deletePeriod");
        blacklistTimeout = par("blacklistTimeout");
        netTraversalTime = par("netTraversalTime");
        nextHopWait = par("nextHopWait");
        pathDiscoveryTime = par("pathDiscoveryTime");
        expungeTimer = new cMessage("ExpungeTimer");
        counterTimer = new cMessage("CounterTimer");
        // rrepAckTimer = new cMessage("RrepAckTimer");
        blacklistTimer = new cMessage("BlackListTimer");

        if (par("RSSIThreshol").intValue() > 0)
            weakMetric = true;


        WATCH_MAP(seqNumbers);
    }
    else if (stage == INITSTAGE_ROUTING_PROTOCOLS) {
        registerProtocol(Protocol::manet, gate("ipOut"), gate("ipIn"));
        networkProtocol->registerHook(0, this);
        host->subscribe(linkBrokenSignal, this);
        cModule *en = host->getSubmodule("energyStorage");
        if (en != nullptr)
            energyStorage = check_and_cast<power::IEpEnergyStorage *>(host->getSubmodule("energyStorage"));
    }
}

void LoadNgBase::actualizeDelayed(Packet *pkt) {
    const auto header = pkt->peekAtFront<FieldsChunk>();
    const auto rreqSt = dynamicPtrCast<const Rreq>(header);
    if (rreqSt) {
        // check if better metric has been received.
        IRoute * route = routingTable->findBestMatchingRoute(rreqSt->getOriginatorAddr());
        LoadNgRouteData *routeData = route ? dynamic_cast<LoadNgRouteData *>(route->getProtocolData()) : nullptr;
        double metric = -1;
        if (routeData) {
            auto rreq = pkt->removeAtFront<Rreq>();
            if (getMetric(rreq, metric)) {
                if (routeData->getMetric() < metric || (routeData->getMetric() == metric && (int)rreqSt->getHopCount() < route->getMetric())) {
                    setMetric(rreq, metric);
                    rreq->setHopCount(route->getMetric());
                }
            }
            pkt->insertAtFront(rreq);
        }
    }
}

void LoadNgBase::handleMessageWhenUp(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if (msg->isScheduled())
            cancelEvent(msg);
        if (auto waitForRrep = dynamic_cast<WaitForRrep *>(msg))
            handleWaitForRREP(waitForRrep);
        else if (msg == expungeTimer)
            expungeRoutes();
        else if (msg == counterTimer) {
            rreqCount = rerrCount = 0;
            scheduleAt(simTime() + 1, counterTimer);
        }
        else if (msg->getKind() == KIND_RREPACKTIMER) {
            handleRREPACKTimer(check_and_cast<RrepAckTimer *>(msg)->getNextHopAddress());
        }
        else if (msg == blacklistTimer)
            handleBlackListTimer();
        else if (msg->getKind() == KIND_DELAYEDSEND) {
            auto timer = check_and_cast<PacketHolderMessage*>(msg);
            auto pkt = timer->removeOwnedPacket();
            actualizeDelayed(pkt);
            send(pkt, "ipOut");
            auto it = pendingSend.find(timer);
            if (it != pendingSend.end())
                pendingSend.erase(it);
            delete timer;
        }
        else if (msg->getKind() == KIND_SMALLDELAY) {
            auto timer = check_and_cast<SmallDelayPacket*>(msg);
            auto target = timer->getDestAddress();
            auto lt = targetAddressToDelayedPackets.lower_bound(target);
            auto ut = targetAddressToDelayedPackets.upper_bound(target);
            // reinject the delayed datagrams
            for (auto it = lt; it != ut; it++) {
                Packet *datagram = it->second;
                const auto& networkHeader = getNetworkProtocolHeader(datagram);
                EV_DETAIL << "Sending queued datagram: source "
                                 << networkHeader->getSourceAddress()
                                 << ", destination "
                                 << networkHeader->getDestinationAddress()
                                 << endl;
                networkProtocol->reinjectQueuedDatagram(const_cast<const Packet *>(datagram));
            }
            // clear the multimap
            targetAddressToDelayedPackets.erase(lt, ut);
            delete msg;
        }
        // we have a route for the destination, thus we must cancel the WaitForRREPTimer events

        else
            throw cRuntimeError("Unknown self message");
    }
    else {

        auto packet = check_and_cast<Packet *>(msg);
        auto protocol = packet->getTag<PacketProtocolTag>()->getProtocol();

        //auto signalPowerInd = packet->findTag<SignalPowerInd>();
        //auto snirInd = packet->findTag<SnirInd>();


        if (protocol == &Protocol::icmpv4) {
            //auto icmpPacket = mspcheck_and_cast<IcmpHeader *>(msg);
            auto icmpPacket = packet->peekAtFront<IcmpHeader>();
            // ICMP packet arrived, dropped
            delete msg;
        }
        else if (protocol == &Protocol::icmpv6) {
            auto icmpPacket = packet->peekAtFront<IcmpHeader>();
            // ICMP packet arrived, dropped
            delete msg;
        }
        else if (true) {  //FIXME protocol == ???
            Packet *udpPacket = check_and_cast<Packet *>(msg);
            //udpPacket->popAtFront<UdpHeader>();
            L3Address sourceAddr = udpPacket->getTag<L3AddressInd>()->getSrcAddress();
            MacAddress macSenderAddr = udpPacket->getTag<MacAddressInd>()->getSrcAddress();
            // KLUDGE: I added this -1 after TTL decrement has been moved in Ipv4
            unsigned int arrivalPacketTTL = udpPacket->getTag<HopLimitInd>()->getHopLimit() - 1;

            const auto& ctrlPacket = udpPacket->popAtFront<LoadNgControlPacket>();
//            ctrlPacket->copyTags(*msg);

            switch (ctrlPacket->getPacketType()) {
                case RREQ:
                    handleRREQ(dynamicPtrCast<Rreq>(ctrlPacket->dupShared()), sourceAddr, arrivalPacketTTL, macSenderAddr);
                    break;

                case RREP:
                    handleRREP(dynamicPtrCast<Rrep>(ctrlPacket->dupShared()), sourceAddr, macSenderAddr);
                    break;

                case RERR:
                    handleRERR(dynamicPtrCast<const Rerr>(ctrlPacket), sourceAddr);
                    break;

                case RREPACK:
                    handleRREPACK(dynamicPtrCast<const RrepAck>(ctrlPacket), sourceAddr);
                    break;
                default:
                    throw cRuntimeError("AODV Control Packet arrived with undefined packet type: %d", ctrlPacket->getPacketType());
            }
            delete udpPacket;
        }
    }
}

INetfilter::IHook::Result LoadNgBase::ensureRouteForDatagram(Packet *datagram)
{
    const auto& networkHeader = getNetworkProtocolHeader(datagram);
    const L3Address& destAddr = networkHeader->getDestinationAddress();
    const L3Address& sourceAddr = networkHeader->getSourceAddress();

    if (destAddr.isBroadcast() || routingTable->isLocalAddress(destAddr) || destAddr.isMulticast())
        return ACCEPT;
    else {
        EV_INFO << "Finding route for source " << sourceAddr << " with destination " << destAddr << endl;
        // refresh route to origin

        if (!sourceAddr.isUnspecified() && !routingTable->isLocalAddress(sourceAddr)) {
            IRoute *routeOrigin = routingTable->findBestMatchingRoute(sourceAddr);
            if (routeOrigin) {
                std::vector<L3Address> pathNode;
                auto macSender = datagram->getTag<MacAddressInd>()->getSrcAddress();
                auto it = macToIpAddress.find(macSender);
                if (it != macToIpAddress.end()) {
                    auto senderAddr = it->second;
                    std::vector<L3Address> pathNode;
                    updateValidRouteLifeTime(senderAddr, simTime() + activeRouteTimeout);
                    updateValidRouteLifeTime(sourceAddr, simTime() + activeRouteTimeout);
                }
            }
        }

        IRoute *route = routingTable->findBestMatchingRoute(destAddr);
        LoadNgRouteData *routeData = route ? dynamic_cast<LoadNgRouteData *>(route->getProtocolData()) : nullptr;
        bool isActive = routeData && routeData->isActive();
        if (isActive && !route->getNextHopAsGeneric().isUnspecified()) {
            EV_INFO << "Active route found: " << route << endl;

            // Each time a route is used to forward a data packet, its Active Route
            // Lifetime field of the source, destination and the next hop on the
            // path to the destination is updated to be no less than the current
            // time plus ACTIVE_ROUTE_TIMEOUT.

            updateValidRouteLifeTime(destAddr, simTime() + activeRouteTimeout);
            updateValidRouteLifeTime(route->getNextHopAsGeneric(), simTime() + activeRouteTimeout);

            auto it = specialPaths.find(destAddr);
            if (it != specialPaths.end())
                it->second.lifetime = simTime() + activeRouteTimeout;

            return ACCEPT;
        }
        else {
            bool isInactive = routeData && !routeData->isActive();

            // TODO: check in the neighbor list if you have an alternative

            if (!sourceAddr.isUnspecified() && !routingTable->isLocalAddress(sourceAddr)) {
                sendRERRWhenNoRouteToForward(destAddr, sourceAddr);
                return DROP;
            }

            auto it = specialPaths.find(destAddr);
            if (it != specialPaths.end()) {

                if (it->second.lifetime < simTime())
                    specialPaths.erase(it);
                else {
                    // force special RREP
                    it->second.lifetime = simTime() + activeRouteTimeout;
                    // delay the datadram
                    delayDatagram(datagram);
                    auto rrep = createRREPSp(it->first, it->second);
                    rrep->setHopLimit(par("maxHopLimit"));
                    sendRREP(rrep, it->first, 255, -1);
                    // small delay
                    auto msgDelay = new SmallDelayPacket();
                    msgDelay->setDestAddress(it->first);
                    scheduleAt(simTime()+0.0001,msgDelay);
                    return QUEUE;
                }
            }

            // A node disseminates a RREQ when it determines that it needs a route
            // to a destination and does not have one available.  This can happen if
            // the destination is previously unknown to the node, or if a previously
            // valid route to the destination expires or is marked as invalid.

            EV_INFO << (isInactive ? "Inactive" : "Missing") << " route for destination " << destAddr << endl;

            delayDatagram(datagram);

            if (!hasOngoingRouteDiscovery(destAddr)) {
                // When a new route to the same destination is required at a later time
                // (e.g., upon route loss), the TTL in the RREQ IP header is initially
                // set to the Hop Count plus TTL_INCREMENT.
                if (isInactive)
                    startRouteDiscovery(destAddr, route->getMetric() + ttlIncrement);
                else
                    startRouteDiscovery(destAddr);
            }
            else
                EV_DETAIL << "Route discovery is in progress, originator " << getSelfIPAddress() << " target " << destAddr << endl;

            return QUEUE;
        }
    }
}

LoadNgBase::LoadNgBase()
{
}

bool LoadNgBase::hasOngoingRouteDiscovery(const L3Address& target)
{
    return waitForRREPTimers.find(target) != waitForRREPTimers.end();
}

void LoadNgBase::startRouteDiscovery(const L3Address& target, unsigned timeToLive)
{
    EV_INFO << "Starting route discovery with originator " << getSelfIPAddress() << " and destination " << target << endl;
    ASSERT(!hasOngoingRouteDiscovery(target));
    auto rreq = createRREQ(target);
    addressToRreqRetries[target] = 0;
    sendRREQ(rreq, addressType->getBroadcastAddress(), timeToLive);
}

L3Address LoadNgBase::getSelfIPAddress() const
{
    return routingTable->getRouterIdAsGeneric();
}

void LoadNgBase::delayDatagram(Packet *datagram)
{
    const auto& networkHeader = getNetworkProtocolHeader(datagram);
    EV_DETAIL << "Queuing datagram, source " << networkHeader->getSourceAddress() << ", destination " << networkHeader->getDestinationAddress() << endl;
    const L3Address& target = networkHeader->getDestinationAddress();
    targetAddressToDelayedPackets.insert(std::pair<L3Address, Packet *>(target, datagram));
}

void LoadNgBase::sendRREQ(const Ptr<Rreq>& rreq, const L3Address& destAddr, unsigned int timeToLive)
{
    // In an expanding ring search, the originating node initially uses a TTL =
    // TTL_START in the RREQ packet IP header and sets the timeout for
    // receiving a RREP to RING_TRAVERSAL_TIME milliseconds.
    // RING_TRAVERSAL_TIME is calculated as described in section 10.  The
    // TTL_VALUE used in calculating RING_TRAVERSAL_TIME is set equal to the
    // value of the TTL field in the IP header.  If the RREQ times out
    // without a corresponding RREP, the originator broadcasts the RREQ
    // again with the TTL incremented by TTL_INCREMENT.  This continues
    // until the TTL set in the RREQ reaches TTL_THRESHOLD, beyond which a
    // TTL = NET_DIAMETER is used for each attempt.

    if (rreqCount >= rreqRatelimit) {
        EV_WARN << "A node should not originate more than RREQ_RATELIMIT RREQ messages per second. Canceling sending RREQ" << endl;
        return;
    }

    auto rrepTimer = waitForRREPTimers.find(rreq->getDestAddr());
    WaitForRrep *rrepTimerMsg = nullptr;
    if (rrepTimer != waitForRREPTimers.end()) {
        rrepTimerMsg = rrepTimer->second;
        unsigned int lastTTL = rrepTimerMsg->getLastTTL();
        rrepTimerMsg->setDestAddr(rreq->getDestAddr());

        // The Hop Count stored in an invalid routing table entry indicates the
        // last known hop count to that destination in the routing table.  When
        // a new route to the same destination is required at a later time
        // (e.g., upon route loss), the TTL in the RREQ IP header is initially
        // set to the Hop Count plus TTL_INCREMENT.  Thereafter, following each
        // timeout the TTL is incremented by TTL_INCREMENT until TTL =
        // TTL_THRESHOLD is reached.  Beyond this TTL = NET_DIAMETER is used.
        // Once TTL = NET_DIAMETER, the timeout for waiting for the RREP is set
        // to NET_TRAVERSAL_TIME, as specified in section 6.3.

        if (timeToLive != 0) {
            rrepTimerMsg->setLastTTL(timeToLive);
            rrepTimerMsg->setFromInvalidEntry(true);
            cancelEvent(rrepTimerMsg);
        }
        else if (lastTTL + ttlIncrement < ttlThreshold) {
            ASSERT(!rrepTimerMsg->isScheduled());
            timeToLive = lastTTL + ttlIncrement;
            rrepTimerMsg->setLastTTL(lastTTL + ttlIncrement);
        }
        else {
            ASSERT(!rrepTimerMsg->isScheduled());
            timeToLive = netDiameter;
            rrepTimerMsg->setLastTTL(netDiameter);
        }
    }
    else {
        rrepTimerMsg = new WaitForRrep();
        waitForRREPTimers[rreq->getDestAddr()] = rrepTimerMsg;
        ASSERT(hasOngoingRouteDiscovery(rreq->getDestAddr()));

        timeToLive = ttlStart;
        rrepTimerMsg->setLastTTL(ttlStart);
        rrepTimerMsg->setFromInvalidEntry(false);
        rrepTimerMsg->setDestAddr(rreq->getDestAddr());
    }

    rreq->setHopLimit(timeToLive);

    if (rreq->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RERR" << endl;
        return;
    }


    // Each time, the timeout for receiving a RREP is RING_TRAVERSAL_TIME.
    simtime_t ringTraversalTime = 2.0 * nodeTraversalTime * (timeToLive + timeoutBuffer);
    scheduleAt(simTime() + ringTraversalTime, rrepTimerMsg);

    EV_INFO << "Sending a Route Request with target " << rreq->getDestAddr() << " and TTL= " << timeToLive << endl;
    if (destAddr.isUnicast())
        sendLoadNgPacket(rreq, destAddr, timeToLive, 0);
    else
        sendLoadNgPacket(rreq, destAddr, timeToLive, *jitterPar);
   // sendLoadNgPacket(rreq, destAddr, timeToLive, *jitterPar);
    rreqCount++;
}

void LoadNgBase::sendRREP(const Ptr<Rrep>& rrep, const L3Address& destAddr, unsigned int timeToLive, const double &cost)
{
    EV_INFO << "Sending Route Reply to " << destAddr << endl;

    // When any node transmits a RREP, the precursor list for the
    // corresponding destination node is updated by adding to it
    // the next hop node to which the RREP is forwarded.

    if (rrep->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RREP" << endl;
        return;
    }

    IRoute *destRoute = routingTable->findBestMatchingRoute(destAddr);
    if (destRoute == nullptr)
        throw cRuntimeError("Route doesn't exist");
    const L3Address& nextHop = destRoute->getNextHopAsGeneric();
    //LoadNgRouteData *destRouteData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());

    IRoute *nextHopRoute = routingTable->findBestMatchingRoute(nextHop);
    if (nextHopRoute == nullptr)
        throw cRuntimeError("Route next hop doesn't exist");
    if (par("generateRrepAck")) {
        auto ackReq = new LoadNgAckRrepReq();
        rrep->getTlvOptionsForUpdate().appendTlvOption(ackReq);
        auto it = pendingAckTimers.find(nextHop);
        RrepAckTimer *timer = nullptr;
        if (it != pendingAckTimers.end()) {
            timer = it->second;
            if (timer->isScheduled())
                cancelEvent(timer);
        }
        else {
            timer = new RrepAckTimer();
            timer->setNextHopAddress(nextHop);
            pendingAckTimers[nextHop] = timer;
        }
        scheduleAt(simTime() + nextHopWait, timer);
    }

    // The node we received the Route Request for is our neighbor,
    // it is probably an unidirectional link
    /*
    if (destRoute->getMetric() == 1 || !destRouteDataNextHop->getIsBidirectiona()) {
        // It is possible that a RREP transmission may fail, especially if the
        // RREQ transmission triggering the RREP occurs over a unidirectional
        // link.

        auto ackReq = new LoadNgAckRrepReq();
        rrep->getTlvOptionsForUpdate().appendTlvOption(ackReq);
        //rrep->setAckRequiredFlag(true);
        // when a node detects that its transmission of a RREP message has failed,
        // it remembers the next-hop of the failed RREP in a "blacklist" set.

        failedNextHop = nextHop;

        if (rrepAckTimer->isScheduled())
            cancelEvent(rrepAckTimer);

        scheduleAt(simTime() + nextHopWait, rrepAckTimer);
    }
    */


    sendLoadNgPacket(rrep, nextHop, timeToLive, 0);
}

const Ptr<Rreq> LoadNgBase::createRREQ(const L3Address& destAddr)
{
    auto rreqPacket = makeShared<Rreq>(); // TODO: "AODV-RREQ");
    //IRoute *lastKnownRoute = routingTable->findBestMatchingRoute(destAddr);

    rreqPacket->setPacketType(RREQ);

    // The Originator Sequence Number in the RREQ message is the
    // node's own sequence number, which is incremented prior to
    // insertion in a RREQ.
    sequenceNum++;

    rreqPacket->setHopLimit(par("maxHopLimit"));
    rreqPacket->setSeqNum(sequenceNum);

    rreqPacket->setOriginatorAddr(getSelfIPAddress());
    rreqPacket->setDestAddr(destAddr);

    if (rreqPacket->getOriginatorAddr().getType() == L3Address::IPv4)
        rreqPacket->setAddrLen(3);
    else if (rreqPacket->getOriginatorAddr().getType() == L3Address::IPv6)
        rreqPacket->setAddrLen(15);
    else if (rreqPacket->getOriginatorAddr().getType() == L3Address::MAC)
        rreqPacket->setAddrLen(5);

    // The Hop Count field is set to zero.
    rreqPacket->setHopCount(0);

/// HERE is propagate the metric.
// metric data
    auto metric2 = new LoadNgMetric2Option();
    metric2->setValue(0);
    rreqPacket->getTlvOptionsForUpdate().appendTlvOption(metric2);


    // Before broadcasting the RREQ, the originating node buffers the RREQ
    // ID and the Originator IP address (its own address) of the RREQ for
    // PATH_DISCOVERY_TIME.
    // In this way, when the node receives the packet again from its neighbors,
    // it will not reprocess and re-forward the packet.

    RreqIdentifier rreqIdentifier(getSelfIPAddress(), rreqId);
    rreqsArrivalTime[rreqIdentifier] = simTime();
    if (rreqPacket->getAddrLen() == 3)
        rreqPacket->setChunkLength(B(30));
    else if (rreqPacket->getAddrLen() == 15)
        rreqPacket->setChunkLength(B(54));
    else if (rreqPacket->getAddrLen() == 5)
        rreqPacket->setChunkLength(B(34));
    return rreqPacket;
}

const Ptr<Rrep> LoadNgBase::createRREPSp(const L3Address &dest, const SpecialPath &path)
{

    auto rrep = makeShared<Rrep>(); // TODO: "AODV-RREP");
    rrep->setPacketType(RREP);

    // When generating a RREP message, a node copies the Destination IP
    // Address and the Originator Sequence Number from the RREQ message into
    // the corresponding fields in the RREP message.

    rrep->setDestAddr(dest);
    rrep->setOriginatorAddr(this->getSelfIPAddress());
    rrep->setSeqNumDest(-1);

    sequenceNum++;
    rrep->setSeqNum(sequenceNum);
    rrep->setHopLimit(par("maxHopLimit"));

    if (rrep->getOriginatorAddr().getType() == L3Address::IPv4)
        rrep->setAddrLen(3);
    else if (rrep->getOriginatorAddr().getType() == L3Address::IPv6)
        rrep->setAddrLen(15);
    else if (rrep->getOriginatorAddr().getType() == L3Address::MAC)
        rrep->setAddrLen(5);
    else
        throw cRuntimeError("Error size incorrect");
    // OriginatorAddr = The IP address of the node which originated the RREQ
    // for which the route is supplied.
    rrep->setHopCount(0);

    // The destination node copies the value MY_ROUTE_TIMEOUT
    // into the Lifetime field of the RREP.
   // rrep->setLifeTime(myRouteTimeout);

// include metrict.
    // metric data
    auto metric2 = new LoadNgMetric2Option();
    metric2->setValue(0);
    rrep->getTlvOptionsForUpdate().appendTlvOption(metric2);

    if (rrep->getAddrLen() == 3)
        rrep->setChunkLength(B(34));
    else if (rrep->getAddrLen() == 15)
        rrep->setChunkLength(B(58));
    else if (rrep->getAddrLen() == 5)
        rrep->setChunkLength(B(38));

    rrep->setAccumulate(true);
    rrep->setAccumulateAddressArraySize(path.path.size());
    for (int i = 0; i < (int)path.path.size(); i++)
        rrep->setAccumulateAddress(i,path.path[i]);
    return rrep;
}


const Ptr<Rrep> LoadNgBase::createRREP(const Ptr<Rreq>& rreq)
{
    if (rreq->getDestAddr() != getSelfIPAddress())
        return Ptr<Rrep>();
    auto rrep = makeShared<Rrep>(); // TODO: "AODV-RREP");
    rrep->setPacketType(RREP);

    // When generating a RREP message, a node copies the Destination IP
    // Address and the Originator Sequence Number from the RREQ message into
    // the corresponding fields in the RREP message.

    rrep->setDestAddr(rreq->getOriginatorAddr());
    rrep->setOriginatorAddr(this->getSelfIPAddress());

    sequenceNum++;
    rrep->setSeqNum(sequenceNum);
    rrep->setHopLimit(par("maxHopLimit"));

    if (rrep->getOriginatorAddr().getType() == L3Address::IPv4)
        rrep->setAddrLen(3);
    else if (rrep->getOriginatorAddr().getType() == L3Address::IPv6)
        rrep->setAddrLen(15);
    else if (rrep->getOriginatorAddr().getType() == L3Address::MAC)
        rrep->setAddrLen(5);
    else
        throw cRuntimeError("Error size incorrect");
    // OriginatorAddr = The IP address of the node which originated the RREQ
    // for which the route is supplied.
    rrep->setHopCount(0);

    // The destination node copies the value MY_ROUTE_TIMEOUT
    // into the Lifetime field of the RREP.
   // rrep->setLifeTime(myRouteTimeout);

// include metrict.
    // metric data
    auto metric2 = new LoadNgMetric2Option();
    metric2->setValue(0);
    rrep->getTlvOptionsForUpdate().appendTlvOption(metric2);

    if (rrep->getAddrLen() == 3)
        rrep->setChunkLength(B(34));
    else if (rrep->getAddrLen() == 15)
        rrep->setChunkLength(B(58));
    else if (rrep->getAddrLen() == 5)
        rrep->setChunkLength(B(38));
    return rrep;
}

void LoadNgBase::handleRREP(const Ptr<Rrep>& rrep, const L3Address& sourceAddr, const MacAddress &macSenderAddress, SignalPowerInd * powerInd, SnirInd * snirInd)
{
    EV_INFO << "LoadNg Route Reply arrived with source addr: " << sourceAddr << " originator addr: " << rrep->getOriginatorAddr()
            << " destination addr: " << rrep->getDestAddr() << endl;

    int weak = 0;
    if (weakMetric) {
        int power = 511;
        if (powerInd) {
            power = powerInd->getPower().get();
            // this values are for uni disk.
            double max = 1.0 / 100.0;
            double min = 1.0 / 10000;
            double interval = max - min;
            double step = interval / 511;
            if (power > max)
                power = max;
            power = std::round(power / step);
            if (power > 511)
                power = 511;
        }

        int snir = 511;
        if (snirInd) {
            snir = snirInd->getMaximumSnir();

            double max = 1.0 / 100.0;
            double min = 1.0 / 10000;
            double interval = max - min;
            double step = interval / 511;
            if (snir > max)
                snir = max;
            snir = std::round(snir / step);
            if (snir > 511)
                snir = 511;
        }

        if (snir < par("RSSIThreshol").intValue()) {
            weak = 1;
            rrep->setWeakLinkMetric(rrep->getWeakLinkMetric()+1);
        }
    }

    if (!macSenderAddress.isUnspecified() && !sourceAddr.isUnspecified()) {
        auto itMac = macToIpAddress.find(macSenderAddress);
        if (itMac == macToIpAddress.end()) {
            macToIpAddress[macSenderAddress] = sourceAddr;
        }
    }

    if (!rrep->getSeqField()) // bad format
        return;

    if (rrep->getHopLimit() != 0) // actualize
        rrep->setHopLimit(rrep->getHopLimit()-1);

    auto itSeq = seqNumbers.find(rrep->getOriginatorAddr());

    if (itSeq != seqNumbers.end() && itSeq->second > rrep->getSeqNum()) // 11.1 No future processing
        return;

    if (itSeq == seqNumbers.end() || (itSeq != seqNumbers.end() && itSeq->second < rrep->getSeqNum())) {
            seqNumbers[rrep->getOriginatorAddr()] = rrep->getSeqNum();
    }

    int metricType = -1;
    unsigned int metric = 255;
    double metricNeg = 1;
    auto metricPos = rrep->getTlvOptionsForUpdate().findByType(METRIC);
    if (metricPos != -1) {
        auto metricTlv = check_and_cast<LoadNgMetricOption *>(rrep->getTlvOptionsForUpdate().getTlvOptionForUpdate(metricPos));
        if (metricTlv->getExtensionFlag())
            metricType = metricTlv->getExtension();
        if (metricTlv->getValueFlag())
            metric = metricTlv->getValue();
        if (metricType == HOPCOUNT) {
            if (metricType == HOPCOUNT) {
                if (metric < 255)
                    metric++;
                metricTlv->setValue(metric);
                metricNeg = 1;
            }
            else {
                // unknow metric set to hopcount
                metricTlv->setType(HOPCOUNT);
                metricTlv->setValue(255);
            }
            metricNeg = 1;
        }
    }


    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // HERE:::::::: ACTUALIZE METRIC
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // HERE; Metric to neighbor

    double metricToOrigin = 255;

    if (!actualizeMetric(rrep, metricNeg, metricToOrigin))
        metricToOrigin = 255;


    IRoute *previousHopRoute = routingTable->findBestMatchingRoute(sourceAddr);
    if (!previousHopRoute || previousHopRoute->getSource() != this) {
        // create without valid sequence number
        previousHopRoute = createRoute(sourceAddr, sourceAddr, 1, -1, true, simTime() + activeRouteTimeout, metricType, metricNeg, weak);
    }
    else {
        auto loadNgRouteData = check_and_cast<LoadNgRouteData *> (previousHopRoute->getProtocolData());
        updateRoutingTable(previousHopRoute, sourceAddr, 1, loadNgRouteData->getDestSeqNum(), true, simTime() + activeRouteTimeout, metricType, metric, weak);
    }

    // Next, the node then increments the hop count value in the RREP by one,
    // to account for the new hop through the intermediate node
    unsigned int newHopCount = rrep->getHopCount() + 1;
    rrep->setHopCount(newHopCount);

    IRoute *originRoute = routingTable->findBestMatchingRoute(rrep->getOriginatorAddr());
    LoadNgRouteData *orgRouteData = nullptr;

    int64_t seqNum = rrep->getSeqNum();

    if (originRoute && originRoute->getSource() == this) {    // already exists
        orgRouteData = check_and_cast<LoadNgRouteData *>(originRoute->getProtocolData());

        // 11.1.  Identifying Invalid RREQ or RREP Messages,
        int64_t oldSeqNum = orgRouteData->getDestSeqNum();
        if (oldSeqNum != -1 && seqNum < oldSeqNum) // invalid message
            return;

        orgRouteData->setIsBidirectiona(true);

        if (orgRouteData->getDestSeqNum() == -1) {
            updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric, rrep->getWeakLinkMetric());
        }
           // Upon comparison, the existing entry is updated only in the following circumstances:
        else if (seqNum > orgRouteData->getDestSeqNum()) {
            updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric, rrep->getWeakLinkMetric());
        }
        else {
            // (iii) the sequence numbers are the same, but the route is
            //       marked as inactive, or
            if (seqNum == orgRouteData->getDestSeqNum() && !orgRouteData->isActive()) {
                updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric, rrep->getWeakLinkMetric());
            }
            // (iv) the sequence numbers are the same, and the New Hop Count is
            //      smaller than the hop count in route table entry.
            else if (weakMetric) {

                if (seqNum == orgRouteData->getDestSeqNum() && rrep->getWeakLinkMetric() < (unsigned int)orgRouteData->getWeakMetric()) {
                    updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric, rrep->getWeakLinkMetric());
                }
                else if (seqNum == orgRouteData->getDestSeqNum() &&  rrep->getWeakLinkMetric() == (unsigned int)orgRouteData->getWeakMetric() && newHopCount < (unsigned int)originRoute->getMetric()) {
                    updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric, rrep->getWeakLinkMetric());
                }
            }
            else {
                if (seqNum == orgRouteData->getDestSeqNum() && newHopCount < (unsigned int)originRoute->getMetric()) {
                    updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric, rrep->getWeakLinkMetric());
                }
            }
        }
    }
    else {    // create forward route for the destination: this path will be used by the originator to send data packets
       // orgRouteData->setIsBidirectiona(true);
        originRoute = createRoute(rrep->getOriginatorAddr(), sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric, rrep->getWeakLinkMetric());
     }

    // check ack req, Tlv
    auto flagPos = rrep->getTlvOptionsForUpdate().findByType(FLAGS);
    if (flagPos != -1) {
        auto flag = check_and_cast<LoadNgAckRrepReq *>(rrep->getTlvOptionsForUpdate().getTlvOptionForUpdate(flagPos));
        if (flag->getValue() == REQACK) {
            auto rrepACK = createRREPACK();
            sendRREPACK(rrepACK, sourceAddr);
            // check if must delete ACK option
            delete rrep->getTlvOptionsForUpdate().dropTlvOption(flag);
            // rrep->setAckRequiredFlag(false);
        }
    }



    if (hasOngoingRouteDiscovery(rrep->getOriginatorAddr())) {
        EV_INFO << "The Route Reply has arrived for our Route Request to node " << rrep->getOriginatorAddr() << endl;
        completeRouteDiscovery(rrep->getOriginatorAddr());
    }


    // Then the forward route for this destination is created if it does not
    // already exist.

    IRoute *destRoute = routingTable->findBestMatchingRoute(rrep->getDestAddr());
    LoadNgRouteData *destRouteData = nullptr;
    if (this->getSelfIPAddress() == rrep->getDestAddr()) {
       // nothing to do
    }
    else if (destRoute && destRoute->getSource() == this) {    // already exists
        destRouteData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());
        if (destRouteData == nullptr)
            throw cRuntimeError("");
        // Upon comparison, the existing entry is updated only in the following circumstances:
        //updateRoutingTable(destRoute, destRoute->getNextHopAsGeneric(), destRoute->getMetric(), destRouteData->getDestSeqNum(), true, simTime() + activeRouteTimeout, destRouteData->getMetric(), destRouteData->getMetric());
        updateValidRouteLifeTime(rrep->getDestAddr(), simTime() + activeRouteTimeout);
        // When any node transmits a RREP, the precursor list for the
        // corresponding destination node is updated by adding to it
        // the next hop node to which the RREP is forwarded.

        if (rrep->getHopCount() > 0) {
            if (destRoute->getNextHopAsGeneric() == rrep->getOriginatorAddr()) {
                // send error
                sendRERRWhenNoRouteToForward(rrep->getDestAddr(), rrep->getOriginatorAddr());
                return;
                throw cRuntimeError("Never loop Origin %s, destination %s", rrep->getOriginatorAddr().str().c_str(), rrep->getDestAddr().str().c_str());
            }
            for (unsigned int i = 0; i < rrep->getAccumulateAddressArraySize(); i++) {
                if (rrep->getAccumulateAddress(i) == destRoute->getNextHopAsGeneric() ||
                        rrep->getAccumulateAddress(i) == this->getSelfIPAddress()) {
                    sendRERRWhenNoRouteToForward(rrep->getDestAddr(), rrep->getOriginatorAddr());
                    return;
                    throw cRuntimeError("RREP loop. Origin %s, destination %s", rrep->getOriginatorAddr().str().c_str(), rrep->getDestAddr().str().c_str());
                }
            }
            auto outgoingRREP = dynamicPtrCast<Rrep>(rrep->dupShared());
            outgoingRREP->appendAccumulateAddress(this->getSelfIPAddress());
            forwardRREP(outgoingRREP, destRoute->getNextHopAsGeneric());
        }
    }
    else {    // create forward route for the destination: this path will be used by the originator to send data packets
        /** received routing message is an RREP and no routing entry was found **/
        sendRERRWhenNoRouteToForward(rrep->getDestAddr(), rrep->getOriginatorAddr());
    }
}

void LoadNgBase::updateRoutingTable(IRoute *route, const L3Address& nextHop, unsigned int hopCount, int64_t destSeqNum, bool isActive, simtime_t lifeTime, int metricType, const double & metric, const int &weak)
{
    EV_DETAIL << "Updating existing route: " << route << endl;

    route->setNextHop(nextHop);
    route->setMetric(hopCount);
    LoadNgRouteData *routingData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
    ASSERT(routingData != nullptr);

    routingData->setLifeTime(lifeTime);
    routingData->setDestSeqNum(destSeqNum);
    routingData->setIsActive(isActive);
    routingData->setActualizeTime();
    routingData->setWeakMetric(weak);


    if (metricType != -1) {
        routingData->setMetricType(metricType);
        routingData->setMetric(metric);
    }
    routingData->setHopCount(hopCount);

    EV_DETAIL << "Route updated: " << route << endl;

    scheduleExpungeRoutes();
}

void LoadNgBase::sendLoadNgPacket(const Ptr<LoadNgControlPacket>& packet, const L3Address& destAddr, unsigned int timeToLive, double delay)
{
    ASSERT(timeToLive != 0);

    if (packet->getHopLimits()) {
        auto rreq = dynamicPtrCast<const Rreq>(packet);
        if (rreq && rreq->getHopLimit() == 0) {
            EV_WARN << "Hop limit 0. Canceling sending Packet Rreq" << endl;
            return;
        }
        else {
            auto rrep = dynamicPtrCast<const Rrep>(packet);
            if (rrep && rrep->getHopLimit() == 0) {
                EV_WARN << "Hop limit 0. Canceling sending Packet Rrep" << endl;
                return;
            }
            else {
                auto rerr = dynamicPtrCast<const Rerr>(packet);
                if (rerr && rerr->getHopLimit() == 0) {
                    EV_WARN << "Hop limit 0. Canceling sending Packet Rerr" << endl;
                    return;
                }
            }
        }
    }
    // TODO: Implement: support for multiple interfaces
    NetworkInterface *ifEntry = interfaceTable->findInterfaceByName("wlan0");

    auto className = packet->getClassName();
    Packet *udpPacket = new Packet(!strncmp("inet::", className, 6) ? className + 6 : className);
    udpPacket->insertAtFront(packet);
    // TODO: was udpPacket->copyTags(*packet);
    udpPacket->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::manet);
    udpPacket->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(addressType->getNetworkProtocol());
    udpPacket->addTagIfAbsent<InterfaceReq>()->setInterfaceId(ifEntry->getInterfaceId());
    auto addresses = udpPacket->addTagIfAbsent<L3AddressReq>();
    addresses->setSrcAddress(getSelfIPAddress());
    addresses->setDestAddress(destAddr);
    udpPacket->addTagIfAbsent<HopLimitReq>()->setHopLimit(timeToLive);

    if (destAddr.isBroadcast())
        lastBroadcastTime = simTime();
    if (delay == 0)
        send(udpPacket, "ipOut");
    else {
        auto *timer = new PacketHolderMessage("LoadNg-send-jitter", KIND_DELAYEDSEND);
        timer->setOwnedPacket(udpPacket);
        pendingSend.insert(timer);
        scheduleAt(simTime()+delay, timer);
    }
}

bool LoadNgBase::actualizeMetric(const Ptr<LoadNgControlPacketWithTlv> &pkt, const double &val, double &newMetric) {
    auto metricPos = pkt->getTlvOptionsForUpdate().findByType(METRIC2);
    int metricType = -1;
    if (metricPos != -1) {
        auto metricTlv = check_and_cast<LoadNgMetric2Option *>(
                pkt->getTlvOptionsForUpdate().getTlvOptionForUpdate(
                        metricPos));
        if (metricTlv->getExtensionFlag())
            metricType = metricTlv->getExtension();
        if (!metricTlv->getValueFlag())
            return false;
        if (metricType == HOPCOUNT) {
            return false;
        } else if (metricType == DIMENSIONLESS ){
            // HERE ACTUALIZE THE METRIC
            metricTlv->setValue(metricTlv->getValue() + val);
            newMetric = metricTlv->getValue();
            return true;
        }
    }

    return false;
}


bool LoadNgBase::setMetric(const Ptr<LoadNgControlPacketWithTlv> &pkt, const double & val) {
    auto metricPos = pkt->getTlvOptionsForUpdate().findByType(METRIC2);
    int metricType = -1;
    if (metricPos  != -1) {
        auto metricTlv = check_and_cast<LoadNgMetric2Option *>(pkt->getTlvOptionsForUpdate().getTlvOptionForUpdate(metricPos));
        if (metricTlv->getExtensionFlag())
            metricType = metricTlv->getExtension();
        if (!metricTlv->getValueFlag())
            return false;
        if (metricType == HOPCOUNT) {
           return false;
        } else if (metricType == DIMENSIONLESS ){
            // HERE ACTUALIZE THE METRIC
            metricTlv->setValue(val);
            return true;
        }
    }
    return false;
}


bool LoadNgBase::getMetric(const Ptr<LoadNgControlPacketWithTlv> &pkt, double &newMetric) {
    auto metricPos = pkt->getTlvOptionsForUpdate().findByType(METRIC2);
    int metricType = -1;
    if (metricPos != -1) {
        auto metricTlv = check_and_cast<LoadNgMetric2Option *>(pkt->getTlvOptionsForUpdate().getTlvOptionForUpdate(metricPos));
        if (metricTlv->getExtensionFlag())
            metricType = metricTlv->getExtension();
        if (!metricTlv->getValueFlag())
            return false;
        if (metricType == HOPCOUNT) {
            return false;
        } else if (metricType == DIMENSIONLESS ){
            // HERE ACTUALIZE THE METRIC
            newMetric = metricTlv->getValue();
            return true;
        }
    }

    return false;
}

void LoadNgBase::actualizeCost(const Ptr<Rreq>& rreq, LoadNgRouteData * loadNgRouteData) {
    loadNgRouteData->setEnergy(rreq->getEnergy());
    loadNgRouteData->setRecPower(rreq->getRecPower());
    loadNgRouteData->setNumNeig(rreq->getNumNeig());
    loadNgRouteData->setDelay(rreq->getDelay());
    loadNgRouteData->setSnir(rreq->getSnir());
    loadNgRouteData->setEtx(rreq->getEtx());
}

void LoadNgBase::handleRREQ(const Ptr<Rreq>& rreq, const L3Address& sourceAddr, unsigned int timeToLive, const MacAddress &macSenderAddress, SignalPowerInd * powerInd, SnirInd * snirInd)
{
    EV_INFO << "LoadNgBasic Route Request arrived with source addr: " << sourceAddr << " originator addr: " << rreq->getOriginatorAddr()
            << " destination addr: " << rreq->getDestAddr() << " Hop limit :" << rreq->getHopLimit() << endl;

    // A node ignores all RREQs received from any node in its blacklist set.

    int weak = 0;
    if (weakMetric) {
        int power = 511;
        if (powerInd) {
            power = powerInd->getPower().get();
            // this values are for uni disk.
            double max = 1.0 / 100.0;
            double min = 1.0 / 10000;
            double interval = max - min;
            double step = interval / 511;
            if (power > max)
                power = max;
            power = std::round(power / step);
            if (power > 511)
                power = 511;
        }

        int snir = 511;
        if (snirInd) {
            snir = snirInd->getMaximumSnir();

            double max = 1.0 / 100.0;
            double min = 1.0 / 10000;
            double interval = max - min;
            double step = interval / 511;
            if (snir > max)
                snir = max;
            snir = std::round(snir / step);
            if (snir > 511)
                snir = 511;
        }

        if (snir < par("RSSIThreshol").intValue()) {
            weak = 1;
            rreq->setWeakLinkMetric(rreq->getWeakLinkMetric()+1);
        }
    }

    if (!macSenderAddress.isUnspecified() && !sourceAddr.isUnspecified()) {
        auto itMac = macToIpAddress.find(macSenderAddress);
        if (itMac == macToIpAddress.end()) {
            macToIpAddress[macSenderAddress] = sourceAddr;
        }
    }

    auto blackListIt = blacklist.find(sourceAddr);
    if (blackListIt != blacklist.end() && !rreq->getAccumulate()) {
        EV_INFO << "The sender node " << sourceAddr << " is in our blacklist. Ignoring the Route Request" << endl;
        return;
    }

    int64_t oldSeqNum = -1;

    auto itSeq = seqNumbers.find(rreq->getOriginatorAddr());
    if (itSeq != seqNumbers.end())
        oldSeqNum = itSeq->second;

    if (itSeq != seqNumbers.end() && itSeq->second > rreq->getSeqNum())
        return;

    if (itSeq == seqNumbers.end() || (itSeq != seqNumbers.end() && itSeq->second < rreq->getSeqNum())) {
            seqNumbers[rreq->getOriginatorAddr()] = rreq->getSeqNum();
    }


    // When a node receives a RREQ, it first creates or updates a route to
    // the previous hop without a valid sequence number (see section 6.2).

    int metricType = -1;
    double metricNeg = 1;

    auto metricPos = rreq->getTlvOptionsForUpdate().findByType(METRIC);

    if (metricPos != -1) {
        auto metricTlv = check_and_cast<LoadNgMetricOption *>(rreq->getTlvOptionsForUpdate().getTlvOptionForUpdate(metricPos));
        if (metricTlv->getExtensionFlag())
            metricType = metricTlv->getExtension();
        uint8_t metric = 255;
        if (metricTlv->getValueFlag())
            metric = metricTlv->getValue();
        if (metricType == HOPCOUNT) {
            if (metric < 255)
                metric++;
            metricTlv->setValue(metric);
            metricNeg = 1;
        }
        else {
            // unknow metricToNeig set to hopcount
            metricTlv->setType(HOPCOUNT);
            metricTlv->setValue(255);
        }
    }


    // HERE:::::::: ACTUALIZE METRIC
    // HERE it is necessary to set the value of VAL,

    double val = 1;
    double metricToOrigin = 255;

    if (!actualizeMetric(rreq, val, metricToOrigin))
        metricToOrigin = 255;


    if (rreq->getHopLimit() != 0) // actualize
        rreq->setHopLimit(rreq->getHopLimit()-1);

    IRoute *previousHopRoute = routingTable->findBestMatchingRoute(sourceAddr);

    // extract the metric to actualize the cost to the next hop
    if ((!previousHopRoute || previousHopRoute->getSource() != this) && !rreq->getAccumulate()) {
        // create without valid sequence number
        previousHopRoute = createRoute(sourceAddr, sourceAddr, 1, -1, true, simTime() + activeRouteTimeout, metricType, metricNeg, weak);
    }
    else {
        if (previousHopRoute) {
            auto loadNgRouteData = check_and_cast<LoadNgRouteData *> (previousHopRoute->getProtocolData());
            loadNgRouteData->setUsingDijkstra(false);
            updateRoutingTable(previousHopRoute, sourceAddr, 1, loadNgRouteData->getDestSeqNum(), true, simTime() + activeRouteTimeout, metricType, metricNeg, weak);
        }
    }

    // First, it first increments the hop count value in the RREQ by one, to
    // account for the new hop through the intermediate node.

    // check 11.1. (1)
    if (rreq->getOriginatorAddr() == getSelfIPAddress())
        return;

    rreq->setHopCount(rreq->getHopCount() + 1);

    // Then the node searches for a reverse route to the Originator IP Address (see
    // section 6.2), using longest-prefix matching.

    IRoute *reverseRoute = routingTable->findBestMatchingRoute(rreq->getOriginatorAddr());

    // If need be, the route is created, or updated using the Originator Sequence Number from the
    // RREQ in its routing table.
    //
    // When the reverse route is created or updated, the following actions on
    // the route are also carried out:
    //
    //   1. the Originator Sequence Number from the RREQ is compared to the
    //      corresponding destination sequence number in the route table entry
    //      and copied if greater than the existing value there
    //
    //   2. the valid sequence number field is set to true;
    //
    //   3. the next hop in the routing table becomes the node from which the
    //      RREQ was received (it is obtained from the source IP address in
    //      the IP header and is often not equal to the Originator IP Address
    //      field in the RREQ message);
    //
    //   4. the hop count is copied from the Hop Count in the RREQ message;
    //
    //   Whenever a RREQ message is received, the Lifetime of the reverse
    //   route entry for the Originator IP address is set to be the maximum of
    //   (ExistingLifetime, MinimalLifetime), where
    //
    //   MinimalLifetime = (current time + 2*NET_TRAVERSAL_TIME - 2*HopCount*NODE_TRAVERSAL_TIME).

    unsigned int hopCount = rreq->getHopCount();
    simtime_t minimalLifeTime = simTime() + 2 * netTraversalTime - 2 * hopCount * nodeTraversalTime;
    simtime_t newLifeTime = std::max(simTime(), minimalLifeTime);
    int rreqSeqNum = rreq->getSeqNum();

    auto prevNodeOrigin = sourceAddr;
    bool newRrep = true;


    if (!reverseRoute || reverseRoute->getSource() != this) {    // create
        // This reverse route will be needed if the node receives a RREP back to the
        // node that originated the RREQ (identified by the Originator IP Address).
        reverseRoute = createRoute(rreq->getOriginatorAddr(), prevNodeOrigin, hopCount, rreqSeqNum, true, newLifeTime,  metricType, metricToOrigin, rreq->getWeakLinkMetric());
        LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(reverseRoute->getProtocolData());
        actualizeCost(rreq, routeData);
    }
    else {
        if (reverseRoute) {
            LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(reverseRoute->getProtocolData());
            int routeSeqNum = routeData->getDestSeqNum();

            if (oldSeqNum < routeSeqNum)
                oldSeqNum = routeSeqNum;

            int newSeqNum = std::max(routeSeqNum, rreqSeqNum);
            int newHopCount = rreq->getHopCount(); // Note: already incremented by 1.
            int routeHopCount = reverseRoute->getMetric();

            // The route is only updated if the new sequence number is either
            //
            //   (i)       higher than the destination sequence number in the route
            //             table, or
            //
            //   (ii)      the sequence numbers are equal, but the hop count (of the
            //             new information) plus one, is smaller than the existing hop
            //             count in the routing table, or
            //
            //   (iii)     the sequence number is unknown.

            if (weakMetric) {
                if (rreqSeqNum > routeSeqNum) {
                    actualizeCost(rreq, routeData);
                    updateRoutingTable(reverseRoute, prevNodeOrigin, hopCount, newSeqNum, true, newLifeTime, metricType, metricToOrigin, rreq->getWeakLinkMetric());
                }
                else if (rreqSeqNum == routeSeqNum && rreq->getWeakLinkMetric() < (unsigned int)routeData->getWeakMetric()) {
                    updateRoutingTable(reverseRoute, prevNodeOrigin, hopCount, newSeqNum, true, newLifeTime, metricType, metricToOrigin, rreq->getWeakLinkMetric());
                    newRrep = false;
                }
                else if (rreqSeqNum == routeSeqNum &&  rreq->getWeakLinkMetric() == (unsigned int)routeData->getWeakMetric() && newHopCount < routeHopCount) {
                    updateRoutingTable(reverseRoute, prevNodeOrigin, hopCount, newSeqNum, true, newLifeTime, metricType, metricToOrigin, rreq->getWeakLinkMetric());
                    newRrep = false;
                }
            }
            else {
                if (rreqSeqNum > routeSeqNum  || (rreqSeqNum == routeSeqNum && newHopCount < routeHopCount)) {
                    if (rreqSeqNum == routeSeqNum) // no propagate.
                        newRrep = false;
                    actualizeCost(rreq, routeData);
                    updateRoutingTable(reverseRoute, prevNodeOrigin, hopCount, newSeqNum, true, newLifeTime, metricType, metricToOrigin, rreq->getWeakLinkMetric());
                }
                else
                    newRrep = false;
            }
        }
    }
    // A node generates a RREP if either:
    //
    // (i)       it is itself the destination, or
    //

    // check (i)
    if (rreq->getDestAddr() == getSelfIPAddress() && newRrep) {
        EV_INFO << "I am the destination node for which the route was requested" << endl;
        // create RREP
        auto rrep = createRREP(rreq);
        rrep->setHopLimit(par("maxHopLimit"));
        //LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(reverseRoute->getProtocolData());
        //rrep->setRecPower(routeData->getRecPower());
        //rrep->setEnergy(routeData->getEnergy());
        //rrep->setDelay(routeData->getDelay());
        //rrep->setSnir(routeData->getSnir());
        //rrep->setEtx(routeData->getEtx());
        // send to the originator
        sendRREP(rrep, rreq->getOriginatorAddr(), 255, -1);
        return;    // discard RREQ, in this case, we do not forward it.
    }
    else if (oldSeqNum == rreq->getSeqNum())
        return; // don't propagate,


    // If a node does not generate a RREP (following the processing rules in
    // section 6.6), and if the incoming IP header has TTL larger than 1,
    // the node updates and broadcasts the RREQ to address 255.255.255.255
    // on each of its configured interfaces (see section 6.14).  To update
    // the RREQ, the TTL or hop limit field in the outgoing IP header is
    // decreased by one, and the Hop Count field in the RREQ message is
    // incremented by one, to account for the new hop through the
    // intermediate node. (!) Lastly, the Destination Sequence number for the
    // requested destination is set to the maximum of the corresponding
    // value received in the RREQ message, and the destination sequence
    // value currently maintained by the node for the requested destination.
    // However, the forwarding node MUST NOT modify its maintained value for
    // the destination sequence number, even if the value received in the
    // incoming RREQ is larger than the value currently maintained by the
    // forwarding node.

    if (simTime() > rebootTime + deletePeriod || rebootTime == 0) {
        auto outgoingRREQ = dynamicPtrCast<Rreq>(rreq->dupShared());
        forwardRREQ(outgoingRREQ);
    }
    else
        EV_WARN << " LoadNg reboot has not completed yet" << endl;
}

IRoute *LoadNgBase::createRoute(const L3Address& destAddr, const L3Address& nextHop,
        unsigned int hopCount, int64_t destSeqNum,
        bool isActive, simtime_t lifeTime, int metricType, const double & metric, const int &weak)
{
    if (destAddr == this->getSelfIPAddress()) {
        throw cRuntimeError("never ");
    }
    // it->second.lastNotification = simTime();
    // create a new route
    // first check if the route exist
    IRoute *route = routingTable->findBestMatchingRoute(destAddr);
    if (route != nullptr) {
        if (route->getSource() == this)
            throw cRuntimeError("");
        else
            routingTable->deleteRoute(route);
    }
    do {
        route = routingTable->findBestMatchingRoute(destAddr);
        if (route) routingTable->deleteRoute(route);
    } while(route != nullptr);


    IRoute *newRoute = routingTable->createRoute();

    // adding generic fields
    newRoute->setDestination(destAddr);
    newRoute->setNextHop(nextHop);
    newRoute->setPrefixLength(addressType->getMaxPrefixLength());    // TODO:
    newRoute->setMetric(hopCount);
    NetworkInterface *ifEntry = interfaceTable->findInterfaceByName("wlan0");    // TODO: IMPLEMENT: multiple interfaces
    if (ifEntry)
        newRoute->setInterface(ifEntry);
    newRoute->setSourceType(IRoute::MANET2);
    newRoute->setSource(this);

    // A route towards a destination that has a routing table entry
    // that is marked as valid.  Only active routes can be used to
    // forward data packets.

    // adding protocol-specific fields
    LoadNgRouteData *newProtocolData = new LoadNgRouteData();
    newProtocolData->setIsActive(isActive);
    newProtocolData->setDestSeqNum(destSeqNum);
    newProtocolData->setLifeTime(lifeTime);
    newProtocolData->setMetricType(metricType);
    newProtocolData->setMetric(metric);
    newProtocolData->setHopCount(hopCount);
    newProtocolData->setActualizeTime();
    newProtocolData->setWeakMetric(weak);

    newRoute->setProtocolData(newProtocolData);

    EV_DETAIL << "Adding new route " << newRoute << endl;
    routingTable->addRoute(newRoute);

    scheduleExpungeRoutes();
    return newRoute;
}

#if 0
void LoadNgBase::actualizeBreakRoutes() {
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *route = routingTable->getRoute(i);
        LoadNgRouteData *routeData = dynamic_cast<LoadNgRouteData *>(route->getProtocolData());
        if (routeData && routeData->isActive() && route->getNextHopAsGeneric() == unreachableAddr) {
            if (routeData->hasValidDestNum())
                routeData->setDestSeqNum(routeData->getDestSeqNum() + 1);
            EV_DETAIL << "Marking route to " << route->getDestinationAsGeneric() << " as inactive" << endl;
            routeData->setIsActive(false);
            routeData->setLifeTime(simTime() + deletePeriod);
            scheduleExpungeRoutes();
            UnreachableNode node;
            node.addr = route->getDestinationAsGeneric();
            node.seqNum = routeData->getDestSeqNum();
            unreachableNodes.push_back(node);
        }
    }
}
#endif

void LoadNgBase::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method("receiveChangeNotification");

    if (signalID == linkBrokenSignal) {

        Packet *datagram = check_and_cast<Packet *>(obj);
        EV_DETAIL << "Received link break signal" << datagram << endl;

        const auto& networkHeader = findNetworkProtocolHeader(datagram);
        if (networkHeader != nullptr) {
            L3Address unreachableAddr = networkHeader->getDestinationAddress();
            L3Address sourceAddress = networkHeader->getSourceAddress();
            if (unreachableAddr.getAddressType() == addressType) {
                // A node initiates processing for a RERR message in three situations:
                //
                //   (i)     if it detects a link break for the next hop of an active
                //           route in its routing table while transmitting data (and
                //           route repair, if attempted, was unsuccessful), or

                // TODO: Implement: local repair
                // check if the node is accessible using Dijkstra

                IRoute *route = routingTable->findBestMatchingRoute(unreachableAddr);

                if (route && route->getSource() == this) {
                    handleLinkBreakSendRERR(route->getNextHopAsGeneric(), sourceAddress, unreachableAddr);
                }
            }
        }
    }
}

void LoadNgBase::handleLinkBreakSendRERR(const L3Address& unreachableAddr, const L3Address& source, const L3Address& destination)
{


    // If local address Destination or source ignore
    if (routingTable->isLocalAddress(source) || routingTable->isLocalAddress(destination))
        return;

    // For case (i), the node first makes a list of unreachable destinations
    // consisting of the unreachable neighbor and any additional
    // destinations (or subnets, see section 7) in the local routing table
    // that use the unreachable neighbor as the next hop.

    // Just before transmitting the RERR, certain updates are made on the
    // routing table that may affect the destination sequence numbers for
    // the unreachable destinations.  For each one of these destinations,
    // the corresponding routing table entry is updated as follows:
    //
    // 1. The destination sequence number of this routing entry, if it
    //    exists and is valid, is incremented for cases (i) and (ii) above,
    //    and copied from the incoming RERR in case (iii) above.
    //
    // 2. The entry is invalidated by marking the route entry as invalid
    //
    // 3. The Lifetime field is updated to current time plus DELETE_PERIOD.
    //    Before this time, the entry SHOULD NOT be deleted.


    IRoute *unreachableRoute = routingTable->findBestMatchingRoute(unreachableAddr);

    if (!unreachableRoute || unreachableRoute->getSource() != this)
        return;

    std::vector<UnreachableNode> unreachableNodes;
    LoadNgRouteData *unreachableRouteData = check_and_cast<LoadNgRouteData *>(unreachableRoute->getProtocolData());

    if (unreachableRouteData->isActive()) {
        UnreachableNode node;
        node.addr = unreachableAddr;
        node.seqNum = unreachableRouteData->getDestSeqNum();
        unreachableNodes.push_back(node);
    }

    // For case (i), the node first makes a list of unreachable destinations
    // consisting of the unreachable neighbor and any additional destinations
    // (or subnets, see section 7) in the local routing table that use the
    // unreachable neighbor as the next hop.

    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *route = routingTable->getRoute(i);

        LoadNgRouteData *routeData = dynamic_cast<LoadNgRouteData *>(route->getProtocolData());
        if (routeData && routeData->isActive() && route->getNextHopAsGeneric() == unreachableAddr) {

            EV_DETAIL << "Marking route to " << route->getDestinationAsGeneric() << " as inactive" << endl;

            routeData->setIsActive(false);
            routeData->setLifeTime(simTime() + deletePeriod);
            scheduleExpungeRoutes();

            UnreachableNode node;
            node.addr = route->getDestinationAsGeneric();
            node.seqNum = routeData->getDestSeqNum();
            unreachableNodes.push_back(node);
        }
    }

    // The neighboring node(s) that should receive the RERR are all those
    // that belong to a precursor list of at least one of the unreachable
    // destination(s) in the newly created RERR.  In case there is only one
    // unique neighbor that needs to receive the RERR, the RERR SHOULD be
    // unicast toward that neighbor.  Otherwise the RERR is typically sent
    // to the local broadcast address (Destination IP == 255.255.255.255,
    // TTL == 1) with the unreachable destinations, and their corresponding
    // destination sequence numbers, included in the packet.

    if (rerrCount >= rerrRatelimit) {
        EV_WARN << "A node should not generate more than RERR_RATELIMIT RERR messages per second. Canceling sending RERR" << endl;
        return;
    }

    if (unreachableNodes.empty())
        return;

    // extract the route to the previous node
    auto nextAddress = addressType->getBroadcastAddress();
    IRoute *route = routingTable->findBestMatchingRoute(source);
    int ttl = 1;
    if (route && route->getSource() == this) {
        LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
        if (routeData->isActive()) {
            nextAddress = route->getNextHopAsGeneric();
            ttl = route->getMetric();
        }
    }

    if (nextAddress.isBroadcast()) // Only unicast
         return;

    auto rerr = createRERR(unreachableNodes);
    rerr->setOriginatorAddr(this->getSelfIPAddress());
    rerr->setDestAddr(source);

    rerrCount++;

    EV_INFO << " Route Error message to " << nextAddress  << endl;
    sendLoadNgPacket(rerr, nextAddress, ttl, 0);
}

const Ptr<Rerr> LoadNgBase::createRERR(const std::vector<UnreachableNode>& unreachableNodes)
{
    auto rerr = makeShared<Rerr>(); // TODO: "AODV-RERR");
    unsigned int destCount = unreachableNodes.size();

    rerr->setPacketType(RERR);
    rerr->setUnreachableNodesArraySize(destCount);
    rerr->setHopLimit(par("maxHopLimit"));

    for (unsigned int i = 0; i < destCount; i++) {
        UnreachableNode node;
        node.addr = unreachableNodes[i].addr;
        node.seqNum = unreachableNodes[i].seqNum;
        rerr->setUnreachableNodes(i, node);
    }

    rerr->setChunkLength(B(4 + 4 * 2 * destCount));
    return rerr;
}

void LoadNgBase::handleRERR(const Ptr<const Rerr>& rerr, const L3Address& sourceAddr)
{
    EV_INFO << "AODV Route Error arrived with source addr: " << sourceAddr << endl;

    // A node initiates processing for a RERR message in three situations:
    // (iii)   if it receives a RERR from a neighbor for one or more
    //         active routes.
    unsigned int unreachableArraySize = rerr->getUnreachableNodesArraySize();
    std::vector<UnreachableNode> unreachableNeighbors;

    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *route = routingTable->getRoute(i);
        LoadNgRouteData *routeData = route ? dynamic_cast<LoadNgRouteData *>(route->getProtocolData()) : nullptr;

        if (!routeData)
            continue;

        // For case (iii), the list should consist of those destinations in the RERR
        // for which there exists a corresponding entry in the local routing
        // table that has the transmitter of the received RERR as the next hop.

        if (routeData->isActive() && route->getNextHopAsGeneric() == sourceAddr) {
            for (unsigned int j = 0; j < unreachableArraySize; j++) {
                if (route->getDestinationAsGeneric() == rerr->getUnreachableNodes(j).addr) {
                    // 1. The destination sequence number of this routing entry, if it
                    // exists and is valid, is incremented for cases (i) and (ii) above,
                    // ! and copied from the incoming RERR in case (iii) above.

                    auto it = seqNumbers.find(route->getDestinationAsGeneric());
                    if (it != seqNumbers.end() && rerr->getUnreachableNodes(j).seqNum < it->second)
                        continue; // ignore older seq num

                    routeData->setDestSeqNum(rerr->getUnreachableNodes(j).seqNum);
                    routeData->setIsActive(false);    // it means invalid, see 3. AODV Terminology p.3. in RFC 3561
                    routeData->setLifeTime(simTime() + deletePeriod);

                    // The RERR should contain those destinations that are part of
                    // the created list of unreachable destinations and have a non-empty
                    // precursor list.

                    //if (routeData->getPrecursorList().size() > 0) {
                    //    UnreachableNode node;
                    //    node.addr = route->getDestinationAsGeneric();
                    //    node.seqNum = routeData->getDestSeqNum();
                    //    unreachableNeighbors.push_back(node);
                    //}
                    // Delete the link from the list.

                    scheduleExpungeRoutes();
                }
            }
        }
    }

    if (rerrCount >= rerrRatelimit) {
        EV_WARN << "A node should not generate more than RERR_RATELIMIT RERR messages per second. Canceling sending RERR" << endl;
        return;
    }

    if (routingTable->isLocalAddress(rerr->getDestAddr())) // No propagate more
        return;

    if (unreachableNeighbors.size() > 0
            && (simTime() > rebootTime + deletePeriod || rebootTime == 0)) {
        EV_INFO << "Sending RERR to inform our neighbors about link breaks."
                       << endl;
        if (rerr->getHopLimit() > 1) {


            // extract the route to the previous node
            auto nextAddress = addressType->getBroadcastAddress();
            IRoute *route = routingTable->findBestMatchingRoute(rerr->getDestAddr());
            int ttl = 1;
            if (route && route->getSource() == this) {
                LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(
                        route->getProtocolData());
                if (routeData->isActive()) {
                    nextAddress = route->getNextHopAsGeneric();
                    ttl = route->getMetric();
                }
            }

            if (nextAddress.isBroadcast()) // Only unicast
                return;
            if (!rerr->getDestAddr().isBroadcast()
                    && routingTable->isLocalAddress(rerr->getDestAddr()))
                return;

            auto newRERR = createRERR(unreachableNeighbors);
            newRERR->setOriginatorAddr(rerr->getOriginatorAddr());
            newRERR->setDestAddr(rerr->getDestAddr());

            if (rerr->getHopLimit() != 0)
                newRERR->setHopLimit(rerr->getHopLimit() - 1);

            sendLoadNgPacket(newRERR, nextAddress, ttl, 0);
            rerrCount++;
        }
    }
}

void LoadNgBase::handleStartOperation(LifecycleOperation *operation)
{
    rebootTime = simTime();

    // RFC 5148:
    // Jitter SHOULD be applied by reducing this delay by a random amount, so that
    // the delay between consecutive transmissions of messages of the same type is
    // equal to (MESSAGE_INTERVAL - jitter), where jitter is the random value.
    scheduleAt(simTime() + 1, counterTimer);
}

void LoadNgBase::handleStopOperation(LifecycleOperation *operation)
{
    clearState();
}

void LoadNgBase::handleCrashOperation(LifecycleOperation *operation)
{
    clearState();
}

void LoadNgBase::clearState()
{
    rerrCount = rreqCount = rreqId = sequenceNum = 0;
    addressToRreqRetries.clear();
    for (auto & elem : waitForRREPTimers)
        cancelAndDelete(elem.second);

    // FIXME: Drop the queued datagrams.
    //for (auto it = targetAddressToDelayedPackets.begin(); it != targetAddressToDelayedPackets.end(); it++)
    //    networkProtocol->dropQueuedDatagram(const_cast<const Packet *>(it->second));

    targetAddressToDelayedPackets.clear();

    waitForRREPTimers.clear();
    rreqsArrivalTime.clear();

    if (expungeTimer)
        cancelEvent(expungeTimer);
    if (counterTimer)
        cancelEvent(counterTimer);
    if (blacklistTimer)
        cancelEvent(blacklistTimer);
    for (const auto &elem : pendingAckTimers)
        cancelAndDelete(elem.second);
    pendingAckTimers.clear();
    //if (rrepAckTimer)
    //    cancelEvent(rrepAckTimer);
}

void LoadNgBase::handleWaitForRREP(WaitForRrep *rrepTimer)
{
    EV_INFO << "We didn't get any Route Reply within RREP timeout" << endl;
    L3Address destAddr = rrepTimer->getDestAddr();

    ASSERT(addressToRreqRetries.find(destAddr) != addressToRreqRetries.end());
    if (addressToRreqRetries[destAddr] == rreqRetries) {
        cancelRouteDiscovery(destAddr);
        EV_WARN << "Re-discovery attempts for node " << destAddr << " reached RREQ_RETRIES= " << rreqRetries << " limit. Stop sending RREQ." << endl;
        return;
    }

    auto rreq = createRREQ(destAddr);

    // the node MAY try again to discover a route by broadcasting another
    // RREQ, up to a maximum of RREQ_RETRIES times at the maximum TTL value.
    if (rrepTimer->getLastTTL() == netDiameter) // netDiameter is the maximum TTL value
        addressToRreqRetries[destAddr]++;

    if (addressToRreqRetries[destAddr] == rreqRetries && threeMessagesMode) {
        // Try 3 messages
        rreq->setAccumulate(true);
    }

    sendRREQ(rreq, addressType->getBroadcastAddress(), 0);
}

void LoadNgBase::forwardRREP(const Ptr<Rrep>& rrep, const L3Address& destAddr)
{
    EV_INFO << "Forwarding the Route Reply to the node " << rrep->getOriginatorAddr() << " which originated the Route Request" << endl;

    if (rrep->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RREP" << endl;
        return;
    }

    IRoute *destRoute = routingTable->findBestMatchingRoute(destAddr);
    if (destRoute == nullptr)
        throw cRuntimeError("Dest route not found");
    //LoadNgRouteData *destRouteData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());

    if (destAddr.isUnicast() && par("generateRrepAck")) {
        auto ackReq = new LoadNgAckRrepReq();
        rrep->getTlvOptionsForUpdate().appendTlvOption(ackReq);
        auto it = pendingAckTimers.find(destRoute->getNextHopAsGeneric());
        if (it != pendingAckTimers.end()) {
            if (it->second->isScheduled())
                cancelEvent(it->second);
            scheduleAt(simTime() + nextHopWait, it->second);
        }
        else {
            auto msg = new RrepAckTimer();
            msg->setNextHopAddress(destRoute->getNextHopAsGeneric());
            pendingAckTimers[destRoute->getNextHopAsGeneric()] = msg;
            scheduleAt(simTime() + nextHopWait, msg);
        }
    }

/*
    char myAddress[30];
    char dest[30];
    strcpy(myAddress, this->getSelfIPAddress().str().c_str());
    strcpy(dest, destAddr.str().c_str());
    */
/*
    if (!isNeigBiDir && (!destRouteData->getIsBidirectiona() && destRoute->getNextHopAsGeneric() == destAddr)) {
        // It is possible that a RREP transmission may fail, especially if the
        // RREQ transmission triggering the RREP occurs over a unidirectional
        // link.
        auto ackReq = new LoadNgAckRrepReq();
        rrep->getTlvOptionsForUpdate().appendTlvOption(ackReq);
        //rrep->setAckRequiredFlag(true);
        // when a node detects that its transmission of a RREP message has failed,
        // it remembers the next-hop of the failed RREP in a "blacklist" set.
        failedNextHop = destAddr;
        if (rrepAckTimer->isScheduled())
            cancelEvent(rrepAckTimer);
        scheduleAt(simTime() + nextHopWait, rrepAckTimer);
    }
*/
    if (destAddr.isUnicast())
        sendLoadNgPacket(rrep, destAddr, 100, 0);
    else
        sendLoadNgPacket(rrep, destAddr, 100, *jitterPar);
}

void LoadNgBase::forwardRREQ(const Ptr<Rreq>& rreq)
{
    EV_INFO << "Forwarding the Route Request message with TTL= " << rreq->getHopLimit() << endl;
    if (rreq->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RREQ" << endl;
        return;
    }
    auto next =  routingTable->findBestMatchingRoute(rreq->getDestAddr());
    LoadNgRouteData *destRouteData = nullptr;
    if (next && next->getSource() == this) {
        destRouteData = check_and_cast<LoadNgRouteData *>(next->getProtocolData());
        if (!destRouteData->isActive())
            next = nullptr;      }
    if (par("smartRREQ").boolValue() && next)
        sendLoadNgPacket(rreq, next->getNextHopAsGeneric(), rreq->getHopLimit(), 0);
    else
        sendLoadNgPacket(rreq, addressType->getBroadcastAddress(), rreq->getHopLimit(), *jitterPar);
}

void LoadNgBase::completeRouteDiscoveryDelayed(const L3Address& target)
{
    EV_DETAIL << "Completing route discovery, originator " << getSelfIPAddress() << ", target " << target << endl;
    ASSERT(hasOngoingRouteDiscovery(target));


    auto route = routingTable->findBestMatchingRoute(target);
    if (route == nullptr)
        throw cRuntimeError("Error no valid route");

    LoadNgRouteData *routeDestData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());

    if (!routeDestData->isActive())
        throw cRuntimeError("Error no valid route");

    auto msgDelay = new SmallDelayPacket();
    msgDelay->setDestAddress(target);
    scheduleAt(simTime()+0.0001,msgDelay);
    auto waitRREPIter = waitForRREPTimers.find(target);
    ASSERT(waitRREPIter != waitForRREPTimers.end());
    cancelAndDelete(waitRREPIter->second);
    waitForRREPTimers.erase(waitRREPIter);
}


void LoadNgBase::completeRouteDiscovery(const L3Address& target)
{
    EV_DETAIL << "Completing route discovery, originator " << getSelfIPAddress() << ", target " << target << endl;
    ASSERT(hasOngoingRouteDiscovery(target));

    auto lt = targetAddressToDelayedPackets.lower_bound(target);
    auto ut = targetAddressToDelayedPackets.upper_bound(target);

    auto route = routingTable->findBestMatchingRoute(target);
    if (route == nullptr)
        throw cRuntimeError("Error no valid route");

    LoadNgRouteData *routeDestData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());

    if (!routeDestData->isActive())
        throw cRuntimeError("Error no valid route");

    // reinject the delayed datagrams
    for (auto it = lt; it != ut; it++) {
        Packet *datagram = it->second;
        const auto& networkHeader = getNetworkProtocolHeader(datagram);
        EV_DETAIL << "Sending queued datagram: source " << networkHeader->getSourceAddress() << ", destination " << networkHeader->getDestinationAddress() << endl;
        networkProtocol->reinjectQueuedDatagram(const_cast<const Packet *>(datagram));
    }

    // clear the multimap
    targetAddressToDelayedPackets.erase(lt, ut);

    // we have a route for the destination, thus we must cancel the WaitForRREPTimer events
    auto waitRREPIter = waitForRREPTimers.find(target);
    ASSERT(waitRREPIter != waitForRREPTimers.end());
    cancelAndDelete(waitRREPIter->second);
    waitForRREPTimers.erase(waitRREPIter);
}

void LoadNgBase::sendGRREP(const Ptr<Rrep>& grrep, const L3Address& destAddr, unsigned int timeToLive)
{
    EV_INFO << "Sending gratuitous Route Reply to " << destAddr << endl;

    IRoute *destRoute = routingTable->findBestMatchingRoute(destAddr);
    const L3Address& nextHop = destRoute->getNextHopAsGeneric();
    if (grrep->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RREQ" << endl;
        return;
    }

    sendLoadNgPacket(grrep, nextHop, timeToLive, 0);
}



void LoadNgBase::expungeRoutes()
{
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *route = routingTable->getRoute(i);
        if (route->getSource() == this) {
            LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
            ASSERT(routeData != nullptr);
            if (routeData->getLifeTime() <= simTime()) {
                if (routeData->isActive()) {
                    EV_DETAIL << "Route to " << route->getDestinationAsGeneric() << " expired and set to inactive. It will be deleted after DELETE_PERIOD time" << endl;
                    // An expired routing table entry SHOULD NOT be expunged before
                    // (current_time + DELETE_PERIOD) (see section 6.11).  Otherwise, the
                    // soft state corresponding to the route (e.g., last known hop count)
                    // will be lost.
                    routeData->setIsActive(false);
                    routeData->setLifeTime(simTime() + deletePeriod);
                }
                else {
                    // Any routing table entry waiting for a RREP SHOULD NOT be expunged
                    // before (current_time + 2 * NET_TRAVERSAL_TIME).
                    if (hasOngoingRouteDiscovery(route->getDestinationAsGeneric())) {
                        EV_DETAIL << "Route to " << route->getDestinationAsGeneric() << " expired and is inactive, but we are waiting for a RREP to this destination, so we extend its lifetime with 2 * NET_TRAVERSAL_TIME" << endl;
                        routeData->setLifeTime(simTime() + 2 * netTraversalTime);
                    }
                    else {
                        EV_DETAIL << "Route to " << route->getDestinationAsGeneric() << " expired and is inactive and we are not expecting any RREP to this destination, so we delete this route" << endl;
                        routingTable->deleteRoute(route);
                    }
                }
            }
        }
    }
    scheduleExpungeRoutes();
}

void LoadNgBase::scheduleExpungeRoutes()
{
    simtime_t nextExpungeTime = SimTime::getMaxTime();
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *route = routingTable->getRoute(i);

        if (route->getSource() == this) {
            LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
            ASSERT(routeData != nullptr);

            if (routeData->getLifeTime() < nextExpungeTime)
                nextExpungeTime = routeData->getLifeTime();
        }
    }
    if (nextExpungeTime == SimTime::getMaxTime()) {
        if (expungeTimer->isScheduled())
            cancelEvent(expungeTimer);
    }
    else {
        if (!expungeTimer->isScheduled())
            scheduleAt(nextExpungeTime, expungeTimer);
        else {
            if (expungeTimer->getArrivalTime() != nextExpungeTime) {
                cancelEvent(expungeTimer);
                scheduleAt(nextExpungeTime, expungeTimer);
            }
        }
    }
}

INetfilter::IHook::Result LoadNgBase::datagramForwardHook(Packet *datagram)
{
    // TODO: Implement: Actions After Reboot
    // If the node receives a data packet for some other destination, it SHOULD
    // broadcast a RERR as described in subsection 6.11 and MUST reset the waiting
    // timer to expire after current time plus DELETE_PERIOD.

    Enter_Method("datagramForwardHook");
    const auto& networkHeader = getNetworkProtocolHeader(datagram);
    const L3Address& destAddr = networkHeader->getDestinationAddress();
    const L3Address& sourceAddr = networkHeader->getSourceAddress();
    IRoute *ipSource = routingTable->findBestMatchingRoute(sourceAddr);

    if (destAddr.isBroadcast() || routingTable->isLocalAddress(destAddr) || destAddr.isMulticast()) {
        if (routingTable->isLocalAddress(destAddr) && ipSource && ipSource->getSource() == this)
            updateValidRouteLifeTime(ipSource->getNextHopAsGeneric(), simTime() + activeRouteTimeout);

        return ACCEPT;
    }

    // TODO: IMPLEMENT: check if the datagram is a data packet or we take control packets as data packets

    IRoute *routeDest = routingTable->findBestMatchingRoute(destAddr);
    LoadNgRouteData *routeDestData = routeDest ? dynamic_cast<LoadNgRouteData *>(routeDest->getProtocolData()) : nullptr;

    // Each time a route is used to forward a data packet, its Active Route
    // Lifetime field of the source, destination and the next hop on the
    // path to the destination is updated to be no less than the current
    // time plus ACTIVE_ROUTE_TIMEOUT

    updateValidRouteLifeTime(sourceAddr, simTime() + activeRouteTimeout);
    updateValidRouteLifeTime(destAddr, simTime() + activeRouteTimeout);

    if (routeDest && routeDest->getSource() == this)
        updateValidRouteLifeTime(routeDest->getNextHopAsGeneric(), simTime() + activeRouteTimeout);

    // Since the route between each originator and destination pair is expected
    // to be symmetric, the Active Route Lifetime for the previous hop, along the
    // reverse path back to the IP source, is also updated to be no less than the
    // current time plus ACTIVE_ROUTE_TIMEOUT.

    if (ipSource && ipSource->getSource() == this)
        updateValidRouteLifeTime(ipSource->getNextHopAsGeneric(), simTime() + activeRouteTimeout);

    EV_INFO << "We can't forward datagram because we have no active route for " << destAddr << endl;
    if (routeDest && routeDestData && !routeDestData->isActive()) {    // exists but is not active
        // A node initiates processing for a RERR message in three situations:
        // (ii)      if it gets a data packet destined to a node for which it
        //           does not have an active route and is not repairing (if
        //           using local repair)

        // TODO: check if it is not repairing (if using local repair)

        // 1. The destination sequence number of this routing entry, if it
        // exists and is valid, is incremented for cases (i) and (ii) above,
        // and copied from the incoming RERR in case (iii) above.

        if (routeDestData->hasValidDestNum())
            routeDestData->setDestSeqNum(routeDestData->getDestSeqNum() + 1);

        // 2. The entry is invalidated by marking the route entry as invalid <- it is invalid

        // 3. The Lifetime field is updated to current time plus DELETE_PERIOD.
        //    Before this time, the entry SHOULD NOT be deleted.
        routeDestData->setLifeTime(simTime() + deletePeriod);

        if (!routingTable->isLocalAddress(sourceAddr))
            sendRERRWhenNoRouteToForward(destAddr, sourceAddr);
    }
    else if (!routeDest || routeDest->getSource() != this) // doesn't exist at all
        if (!routingTable->isLocalAddress(sourceAddr))
            sendRERRWhenNoRouteToForward(destAddr, sourceAddr);

    return ACCEPT;
}

void LoadNgBase::sendRERRWhenNoRouteToForward(const L3Address& unreachableAddr, const L3Address &destAddr)
{
    if (rerrCount >= rerrRatelimit) {
        EV_WARN << "A node should not generate more than RERR_RATELIMIT RERR messages per second. Canceling sending RERR" << endl;
        return;
    }

    std::vector<UnreachableNode> unreachableNodes;
    UnreachableNode node;
    node.addr = unreachableAddr;

    IRoute *unreachableRoute = routingTable->findBestMatchingRoute(unreachableAddr);
    LoadNgRouteData *unreachableRouteData = unreachableRoute ? dynamic_cast<LoadNgRouteData *>(unreachableRoute->getProtocolData()) : nullptr;

    if (unreachableRouteData && unreachableRouteData->hasValidDestNum())
        node.seqNum = unreachableRouteData->getDestSeqNum();
    else
        node.seqNum = 0;

    unreachableNodes.push_back(node);

    // extract the route to the previous node
    auto nextAddress = addressType->getBroadcastAddress();
    IRoute *route = routingTable->findBestMatchingRoute(destAddr);
    int ttl = 1;
     if (route && route->getSource() == this) {
         LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
         if (routeData->isActive()) {
             nextAddress = route->getNextHopAsGeneric();
             ttl = route->getMetric();
         }
     }
     if (nextAddress.isBroadcast()) // Only unicast
         return;

     auto rerr = createRERR(unreachableNodes);
     rerr->setOriginatorAddr(this->getSelfIPAddress());
     rerr->setDestAddr(destAddr);


    rerrCount++;
    EV_INFO << "Broadcasting Route Error message with TTL=1" << endl;
    sendLoadNgPacket(rerr, nextAddress, ttl, 0);    // TODO: unicast if there exists a route to the source
}

void LoadNgBase::cancelRouteDiscovery(const L3Address& destAddr)
{
    ASSERT(hasOngoingRouteDiscovery(destAddr));
    auto lt = targetAddressToDelayedPackets.lower_bound(destAddr);
    auto ut = targetAddressToDelayedPackets.upper_bound(destAddr);
    for (auto it = lt; it != ut; it++)
        networkProtocol->dropQueuedDatagram(const_cast<const Packet *>(it->second));

    targetAddressToDelayedPackets.erase(lt, ut);

    auto waitRREPIter = waitForRREPTimers.find(destAddr);
    ASSERT(waitRREPIter != waitForRREPTimers.end());
    cancelAndDelete(waitRREPIter->second);
    waitForRREPTimers.erase(waitRREPIter);
}

bool LoadNgBase::updateValidRouteLifeTime(const L3Address& destAddr, simtime_t lifetime)
{
    IRoute *route = routingTable->findBestMatchingRoute(destAddr);
    if (route && route->getSource() == this) {
        LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
        if (routeData->isActive()) {
            simtime_t newLifeTime = std::max(routeData->getLifeTime(), lifetime);
            EV_DETAIL << "Updating " << route << " lifetime to " << newLifeTime << endl;
            routeData->setLifeTime(newLifeTime);
            return true;
        }
    }
    return false;
}

const Ptr<RrepAck> LoadNgBase::createRREPACK()
{
    auto rrepACK = makeShared<RrepAck>(); // TODO: "AODV-RREPACK");
    rrepACK->setPacketType(RREPACK);
    rrepACK->setChunkLength(B(2));
    return rrepACK;
}

void LoadNgBase::sendRREPACK(const Ptr<RrepAck>& rrepACK, const L3Address& destAddr)
{
    EV_INFO << "Sending Route Reply ACK to " << destAddr << endl;
    sendLoadNgPacket(rrepACK, destAddr, 100, 0);
}

void LoadNgBase::handleRREPACK(const Ptr<const RrepAck>& rrepACK, const L3Address& neighborAddr)
{
    // Note that the RREP-ACK packet does not contain any information about
    // which RREP it is acknowledging.  The time at which the RREP-ACK is
    // received will likely come just after the time when the RREP was sent
    // with the 'A' bit.
    auto it = pendingAckTimers.find(neighborAddr);
    if (it != pendingAckTimers.end()) {
        if (it->second->isScheduled()) {
            IRoute *route = routingTable->findBestMatchingRoute(neighborAddr);
            if (route && route->getSource() == this) {
                EV_DETAIL << "Marking route " << route << " as active" << endl;
                LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
                routeData->setIsActive(true);
                routeData->setIsBidirectiona(true);
                cancelEvent(it->second);
            }
        }
        delete it->second;
        pendingAckTimers.erase(it);
    }
/*
    if (rrepAckTimer->isScheduled()) {
        EV_INFO << "RREP-ACK arrived from " << neighborAddr << endl;

        IRoute *route = routingTable->findBestMatchingRoute(neighborAddr);
        if (route && route->getSource() == this) {
            EV_DETAIL << "Marking route " << route << " as active" << endl;
            LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
            routeData->setIsActive(true);
            routeData->setIsBidirectiona(true);
            cancelEvent(rrepAckTimer);
        }
    }
    */
}

void LoadNgBase::handleRREPACKTimer(const L3Address & addr)
{
    // when a node detects that its transmission of a RREP message has failed,
    // it remembers the next-hop of the failed RREP in a "blacklist" set.

    EV_INFO << "RREP-ACK didn't arrived within timeout. Adding " << addr << " to the blacklist" << endl;

    blacklist[addr] = simTime() + blacklistTimeout;    // lifetime

    auto it = pendingAckTimers.find(addr);
    if (it != pendingAckTimers.end()) {
        delete it->second;
        pendingAckTimers.erase(it);
    }

    if (!blacklistTimer->isScheduled())
        scheduleAt(simTime() + blacklistTimeout, blacklistTimer);
}

void LoadNgBase::handleBlackListTimer()
{
    simtime_t nextTime = SimTime::getMaxTime();

    for (auto it = blacklist.begin(); it != blacklist.end(); ) {
        auto current = it++;

        // Nodes are removed from the blacklist set after a BLACKLIST_TIMEOUT period
        if (current->second <= simTime()) {
            EV_DETAIL << "Blacklist lifetime has expired for " << current->first << " removing it from the blacklisted addresses" << endl;
            blacklist.erase(current);
        }
        else if (nextTime > current->second)
            nextTime = current->second;
    }

    if (nextTime != SimTime::getMaxTime())
        scheduleAt(nextTime, blacklistTimer);
}

LoadNgBase::~LoadNgBase()
{

    for (auto & e : pendingSend) {
        cancelAndDelete(e);
    }
    cancelAndDelete(recomputeTimer);
    pendingSend.clear();

    clearState();

    delete expungeTimer;
    delete counterTimer;
    //delete rrepAckTimer;
    delete blacklistTimer;
}

}
}
} // namespace inet

