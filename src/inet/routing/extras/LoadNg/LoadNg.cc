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
#include "inet/routing/extras/LoadNg/LoadNg.h"
#include "inet/routing/extras/LoadNg/DeepFirstForwardTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/common/packet/dissector/PacketDissector.h"

// DONE: actualize routes using hello information,
// DONE: Fill the routing tables using the routes computes by Dijkstra
// DONE: Modify the link layer to force that a percentage of links could be only unidirectional
// DONE: Compute k-shortest paths for using with DFF
// DONE: Measure ETX, DONE: include fields

// TODO: Extract alternative routes from Dijkstra if the principal fails.
// TODO: Recalculate Dijkstra using the transmission errors: DONE PARTIAL,
// TODO: Calculate the route to the sink using Hellos, TODO: Propagate distance to root, DONE: define TLV for this
// TODO: measure link quality metrics and to use them for routing.
// TODO: Modify the link layer that quality measures could arrive to upper layers
// TODO: Modify the link layer to force a predetermine lost packets in predetermined links.
// TODO: Add DFF tag to the packets stolen from network layer.
// TODO: Solve unidirectional problem, find a route when not bidirectional path exist, double RREQ.
// TODO: the protocol LoadNg must handle the packets in this case, the final destination address must be included in a header and the Ip address must be broadcast.



namespace inet {
namespace inetmanet {

simsignal_t LoadNg::recomputeSignal = registerSignal("LoadNgRecomputeSignal");


Define_Module(LoadNg);


inline std::ostream& operator<<(std::ostream& out, const NeigborElement& d)
{
    double metric = d.metricToNeig;
    out << "Bidir="<< d.isBidirectional << " Pending " << d.pendingConfirmation << " Metric= "<< metric << " Neig list ";
    for (auto elem : d.listNeigbours)
    {
        int metric = elem.second.metric;
        out << "add = " << elem.first << " bidir = " <<  elem.second.isBidirectional << "Pending " << elem.second.pendingConfirmation
        << " metric = "<< metric << " <> ";
    }
    return out;
}


void LoadNg::initialize(int stage)
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
        routingTable.reference(this, "routingTableModule", true);
        interfaceTable.reference(this, "interfaceTableModule", true);
        networkProtocol.reference(this, "networkProtocolModule", true);

        aodvUDPPort = par("udpPort");
        useHelloMessages = par("useHelloMessages");
        activeRouteTimeout = par("activeRouteTimeout");
        helloInterval = par("helloInterval");
        allowedHelloLoss = par("allowedHelloLoss");
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
        periodicJitter = &par("periodicJitter");

        myRouteTimeout = par("myRouteTimeout");
        deletePeriod = par("deletePeriod");
        blacklistTimeout = par("blacklistTimeout");
        netTraversalTime = par("netTraversalTime");
        nextHopWait = par("nextHopWait");
        pathDiscoveryTime = par("pathDiscoveryTime");
        expungeTimer = new cMessage("ExpungeTimer");
        counterTimer = new cMessage("CounterTimer");
        rrepAckTimer = new cMessage("RrepAckTimer");
        blacklistTimer = new cMessage("BlackListTimer");
        if (useHelloMessages)
            helloMsgTimer = new cMessage("HelloMsgTimer");

        measureEtx = par("measureEtx");

        minHelloInterval = par("minHelloInterval");
        maxHelloInterval = par("maxHelloInterval");

        threeMessagesMode = par("threeMessagesMode");

        WATCH_MAP(neighbors);
    }
    else if (stage == INITSTAGE_ROUTING_PROTOCOLS) {
        registerProtocol(Protocol::manet, gate("ipOut"), gate("ipIn"));
        networkProtocol->registerHook(0, this);
        host->subscribe(linkBrokenSignal, this);
        if (par("useDijkstra")) {
            dijkstra = new Dijkstra();
            dijkstra->setRoot(this->getSelfIPAddress());
            if (activeFFD) {
                dijkstraKs = new  DijkstraKshortest(5);
                dijkstraKs->initMinAndMax();
                dijkstraKs->setRoot(this->getSelfIPAddress());

            }
        }
        if (useCommonData) {
            nodesData[this->getSelfIPAddress()] = &neighbors;
            dijkstraTotal = new Dijkstra();
            dijkstraTotal->setRoot(this->getSelfIPAddress());
            recomputeTimer = new cMessage("RecomputeTimer");
            getSimulation()->getSystemModule()->subscribe(recomputeSignal, this);
        }
        cModule *en = host->getSubmodule("energyStorage");
        if (en != nullptr)
            energyStorage = check_and_cast<power::IEpEnergyStorage *>(host->getSubmodule("energyStorage"));
    }
}

void LoadNg::actualizeDelayed(Packet *pkt) {
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

void LoadNg::handleMessageWhenUp(cMessage *msg)
{
    checkNeigList(msg);
    if (msg->isSelfMessage()) {
        if (msg->isScheduled())
            cancelEvent(msg);
        if (recomputeTimer == msg) {
            regenerateLinkMap();
            return;
        }
        if (auto waitForRrep = dynamic_cast<WaitForRrep *>(msg))
            handleWaitForRREP(waitForRrep);
        else if (msg == helloMsgTimer)
            sendHelloMessagesIfNeeded();
        else if (msg == expungeTimer)
            expungeRoutes();
        else if (msg == counterTimer) {
            rreqCount = rerrCount = 0;
            scheduleAt(simTime() + 1, counterTimer);
        }
        else if (msg == rrepAckTimer)
            handleRREPACKTimer();
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
                networkProtocol->reinjectQueuedDatagram(datagram);
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
        auto signalPowerInd = packet->findTag<SignalPowerInd>();
        auto snirInd = packet->findTag<SnirInd>();

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

                case HELLO:
                    handleHelloMessage(dynamicPtrCast<const Hello>(ctrlPacket), signalPowerInd, snirInd, macSenderAddr);
                    break;

                default:
                    throw cRuntimeError("AODV Control Packet arrived with undefined packet type: %d", ctrlPacket->getPacketType());
            }
            delete udpPacket;
        }
    }
}

INetfilter::IHook::Result LoadNg::ensureRouteForDatagram(Packet *datagram)
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
                if (dijkstra && dijkstra->getRoute(sourceAddr, pathNode)) {
                    LoadNgRouteData *routeData = routeOrigin ? dynamic_cast<LoadNgRouteData *>(routeOrigin->getProtocolData()) : nullptr;
                    if (routeData && routeData->getUsingDijkstra()) {
                        if (pathNode[1] != routeOrigin->getNextHopAsGeneric()) {// rebuild data bases
                           /* double t1 = simTime().dbl();
                            double t2 = routeData->actualized.dbl();
                            auto itNeig = neighbors.find(pathNode[1]);
                            double t3 = itNeig->second.lastNotification.dbl();
                            auto itNeig2 = neighbors.find(routeOrigin->getNextHopAsGeneric());
                            double t4 = itNeig2->second.lastNotification.dbl();*/
                            rebuildDijkstraWihtHellos();
                            /*double t2b = routeData->actualized.dbl();
                            dijkstra->getRoute(sourceAddr, pathNode);
                            itNeig = neighbors.find(pathNode[1]);
                            double t3b = itNeig->second.lastNotification.dbl();
                            printf("");*/
                        }
                    }
                }

                auto macSender = datagram->getTag<MacAddressInd>()->getSrcAddress();
                auto it = macToIpAddress.find(macSender);
                if (it != macToIpAddress.end()) {
                    auto senderAddr = it->second;
                    std::vector<L3Address> pathNode;
                    if (dijkstra && dijkstra->getRoute(sourceAddr, pathNode)) {

                        // adapt the route
                        if (pathNode[1] != routeOrigin->getNextHopAsGeneric()) {
                            routeOrigin->setNextHop(pathNode[1]);
                        }
                    }
                    else {
                        auto itNeig = neighbors.find(senderAddr);
                        if (itNeig != neighbors.end() && itNeig->second.isBidirectional && !itNeig->second.pendingConfirmation) {
                            if (routeOrigin->getNextHopAsGeneric() != senderAddr) {
                                routeOrigin->setNextHop(senderAddr);
                            }
                        }
                    }
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

LoadNg::LoadNg()
{
}

bool LoadNg::hasOngoingRouteDiscovery(const L3Address& target)
{
    return waitForRREPTimers.find(target) != waitForRREPTimers.end();
}

void LoadNg::startRouteDiscovery(const L3Address& target, unsigned timeToLive)
{
    EV_INFO << "Starting route discovery with originator " << getSelfIPAddress() << " and destination " << target << endl;
    ASSERT(!hasOngoingRouteDiscovery(target));
    auto rreq = createRREQ(target);
    addressToRreqRetries[target] = 0;
    if (par("Force3Messages").boolValue())
        rreq->setAccumulate(true);
    sendRREQ(rreq, addressType->getBroadcastAddress(), timeToLive);
}

L3Address LoadNg::getSelfIPAddress() const
{
    return routingTable->getRouterIdAsGeneric();
}

void LoadNg::delayDatagram(Packet *datagram)
{
    const auto& networkHeader = getNetworkProtocolHeader(datagram);
    EV_DETAIL << "Queuing datagram, source " << networkHeader->getSourceAddress() << ", destination " << networkHeader->getDestinationAddress() << endl;
    const L3Address& target = networkHeader->getDestinationAddress();
    targetAddressToDelayedPackets.insert(std::pair<L3Address, Packet *>(target, datagram));
}

void LoadNg::sendRREQ(const Ptr<Rreq>& rreq, const L3Address& destAddr, unsigned int timeToLive)
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

void LoadNg::sendRREP(const Ptr<Rrep>& rrep, const L3Address& destAddr, unsigned int timeToLive, const double &cost)
{
    EV_INFO << "Sending Route Reply to " << destAddr << endl;

    // When any node transmits a RREP, the precursor list for the
    // corresponding destination node is updated by adding to it
    // the next hop node to which the RREP is forwarded.

    if (rrep->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RREP" << endl;
        return;
    }

    if (rrep->getAccumulate()) {
        // special packet, send
        // first check the origin
        L3Address nextHop;
        L3Address destAddress = rrep->getDestAddr();
        if (rrep->getAccumulateAddressArraySize() == 0)
                        nextHop = destAddress;
        else {
            if (routingTable->isLocalAddress(rrep->getOriginatorAddr())) {
                // Fist
                nextHop = rrep->getAccumulateAddress(0);
            }
            else {
                // Extract next hop
                for (unsigned int i = 0; i < rrep->getAccumulateAddressArraySize(); i++) {
                    if (routingTable->isLocalAddress(rrep->getOriginatorAddr())) {
                        if (i == rrep->getAccumulateAddressArraySize() - 1)
                            nextHop = destAddress;
                        else
                            nextHop = rrep->getAccumulateAddress(i+1);
                    }
                }
            }
        }
        // modify the routing table and send
        IRoute *destRoute = routingTable->findBestMatchingRoute(destAddr);
        IRoute *nextHopRoute = routingTable->findBestMatchingRoute(nextHop);
        int hops = rrep->getAccumulateAddressArraySize() + 1;

        // extract the metric to actualize the cost to the next hop
        if (!destRoute || destRoute->getSource() != this) {
            // create without valid sequence number
            destRoute = createRoute(destAddr, nextHop, hops, rrep->getSeqNumDest(), true, simTime() + activeRouteTimeout, HOPCOUNT, hops);
        }
        else {
            if (destRoute) {
               updateRoutingTable(destRoute, nextHop, hops, rrep->getSeqNumDest(), true, simTime() + activeRouteTimeout, HOPCOUNT, hops);
            }
        }

        if (!nextHopRoute || nextHopRoute->getSource() != this) {
                   // create without valid sequence number
            nextHopRoute = createRoute(destAddr, nextHop, 1, -1, true, simTime() + activeRouteTimeout, HOPCOUNT, hops);
        }
        sendLoadNgPacket(rrep, nextHop, timeToLive, 0);
        return;
    }

    IRoute *destRoute = routingTable->findBestMatchingRoute(destAddr);
    const L3Address& nextHop = destRoute->getNextHopAsGeneric();
    //LoadNgRouteData *destRouteData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());

    IRoute *nextHopRoute = routingTable->findBestMatchingRoute(nextHop);
    LoadNgRouteData *destRouteDataNextHop = check_and_cast<LoadNgRouteData *>(nextHopRoute->getProtocolData());


    // The node we received the Route Request for is our neighbor,
    // it is probably an unidirectional link
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


    sendLoadNgPacket(rrep, nextHop, timeToLive, 0);
}

const Ptr<Rreq> LoadNg::createRREQ(const L3Address& destAddr)
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

const Ptr<Rrep> LoadNg::createRREPSp(const L3Address &dest, const SpecialPath &path)
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


const Ptr<Rrep> LoadNg::createRREP(const Ptr<Rreq>& rreq, IRoute *destRoute, IRoute *originatorRoute, const L3Address& lastHopAddr)
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

void LoadNg::handleRREP(const Ptr<Rrep>& rrep, const L3Address& sourceAddr, const MacAddress &macSenderAddress)
{
    EV_INFO << "AODV Route Reply arrived with source addr: " << sourceAddr << " originator addr: " << rrep->getOriginatorAddr()
            << " destination addr: " << rrep->getDestAddr() << endl;

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

    if (itSeq != seqNumbers.end() && itSeq->second > rrep->getSeqNum())
        return;

    if (itSeq == seqNumbers.end() || (itSeq != seqNumbers.end() && itSeq->second < rrep->getSeqNum())) {
            seqNumbers[rrep->getOriginatorAddr()] = rrep->getSeqNum();
    }

    // Special RREP packet, generate with the double search RREQ
    if (rrep->getAccumulate()) {
        // special packet, send
        // first check the origin
        L3Address nextHop;
        L3Address destAddress = rrep->getDestAddr();
        int hops = rrep->getAccumulateAddressArraySize() + 1;
        if (!routingTable->isLocalAddress(destAddress)) {
            if (rrep->getAccumulateAddressArraySize() == 0)
                nextHop = destAddress;
            else {
                if (routingTable->isLocalAddress(rrep->getOriginatorAddr())) {
                    // Fist
                    nextHop = rrep->getAccumulateAddress(0);
                }
                else {
                    // Extract next hop
                    for (unsigned int i = 0; i < rrep->getAccumulateAddressArraySize(); i++) {
                        if (routingTable->isLocalAddress(rrep->getAccumulateAddress(i))) {
                            if (i == rrep->getAccumulateAddressArraySize() - 1)
                                nextHop = destAddress;
                            else
                                nextHop = rrep->getAccumulateAddress(i + 1);
                            hops -= (i+1);
                            break;
                        }
                    }
                }
            }
        }
        if (!routingTable->isLocalAddress(destAddress)) {
            // modify the routing table and send
            IRoute *destRoute = routingTable->findBestMatchingRoute(destAddress);

            if (nextHop.isUnspecified())
                throw cRuntimeError("Next hop address unspecified");

            // extract the metric to actualize the cost to the next hop
            if (!destRoute || destRoute->getSource() != this) {
                // create without valid sequence number
                destRoute = createRoute(destAddress, nextHop, hops, rrep->getSeqNumDest(), true, simTime() + activeRouteTimeout, HOPCOUNT, hops);
            }
            else {
                auto seqNum = rrep->getSeqNumDest();
                auto routeData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());
                auto regSeqNum = routeData->getDestSeqNum();
                if (seqNum !=-1 && seqNum < regSeqNum)
                    throw cRuntimeError("Check seq numbers");
                if (destRoute) {
                    updateRoutingTable(destRoute, nextHop, hops, rrep->getSeqNumDest(), true, simTime() + activeRouteTimeout, HOPCOUNT, hops);
                }
            }
        }
        else {
            if(rrep->getIncludeRoute()) {
                SpecialPath path;
                path.lifetime = simTime() + 2 * netTraversalTime - 2 * (rrep->getRouteArraySize()+1) * nodeTraversalTime;
                for (unsigned int i = 0; i < rrep->getRouteArraySize(); i ++) {
                    path.path.push_back(rrep->getRoute(i));
                }
                specialPaths[rrep->getOriginatorAddr()] = path;
            }
        }
        if (rrep->getHopLimit() > 0 && this->getSelfIPAddress() != rrep->getDestAddr()) {
            auto outgoingRREP = dynamicPtrCast<Rrep>(rrep->dupShared());
            forwardRREP(outgoingRREP, nextHop, 100);
        }
        if (hasOngoingRouteDiscovery(rrep->getOriginatorAddr())) {
              EV_INFO << "The Route Reply has arrived for our Route Request to node " << rrep->getOriginatorAddr() << endl;
              completeRouteDiscovery(rrep->getOriginatorAddr());
        }
        return;
    }


    // the hello is now a sparate type.
   /*if (rrep->getDestAddr().isUnspecified()) {
        EV_INFO << "This Route Reply is a Hello Message" << endl;
        handleHelloMessage(rrep);
        return;
    }
    */

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

    auto itNeig = neighbors.find(sourceAddr);
    if (useHelloMessages && (itNeig == neighbors.end() || !itNeig->second.isBidirectional || itNeig->second.pendingConfirmation)) {
        EV_INFO << "The sender node " << sourceAddr << " is not in the neighbors list or is not bi-directional. Ignoring the Route Request" << endl;
        // throw cRuntimeError("");
    }


    // HERE; Metric to neighbor

    double metricToOrigin = 255;

    if (useHelloMessages && measureEtx)
          metricNeg = itNeig->second.metricToNeig;

    if (!actualizeMetric(rrep, metricNeg, metricToOrigin))
        metricToOrigin = 255;


    IRoute *previousHopRoute = routingTable->findBestMatchingRoute(sourceAddr);
    if (!previousHopRoute || previousHopRoute->getSource() != this) {
        // create without valid sequence number
        previousHopRoute = createRoute(sourceAddr, sourceAddr, 1, -1, true, simTime() + activeRouteTimeout, metricType, metricNeg);
    }
    else {
        auto loadNgRouteData = check_and_cast<LoadNgRouteData *> (previousHopRoute->getProtocolData());
        updateRoutingTable(previousHopRoute, sourceAddr, 1, loadNgRouteData->getDestSeqNum(), true, simTime() + activeRouteTimeout, metricType, metric);
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
            updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric);
        }
           // Upon comparison, the existing entry is updated only in the following circumstances:
        else if (seqNum > orgRouteData->getDestSeqNum()) {
            updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric);

        }
        else {
            // (iii) the sequence numbers are the same, but the route is
            //       marked as inactive, or
            if (seqNum == orgRouteData->getDestSeqNum() && !orgRouteData->isActive()) {
                updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric);
            }
            // (iv) the sequence numbers are the same, and the New Hop Count is
            //      smaller than the hop count in route table entry.
            else if (seqNum == orgRouteData->getDestSeqNum() && metric < (unsigned int)orgRouteData->getMetric()) {
                updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric);
            }
            else if (seqNum == orgRouteData->getDestSeqNum() && metric == (unsigned int)orgRouteData->getMetric() && newHopCount < (unsigned int)originRoute->getMetric()) {
                updateRoutingTable(originRoute, sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric);
            }
        }
    }
    else {    // create forward route for the destination: this path will be used by the originator to send data packets
       // orgRouteData->setIsBidirectiona(true);
        originRoute = createRoute(rrep->getOriginatorAddr(), sourceAddr, newHopCount, seqNum, true, simTime() + activeRouteTimeout, metricType, metric);
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
            throw cRuntimeError("destRouteData == nullptr");
        // Upon comparison, the existing entry is updated only in the following circumstances:
        //updateRoutingTable(destRoute, destRoute->getNextHopAsGeneric(), destRoute->getMetric(), destRouteData->getDestSeqNum(), true, simTime() + activeRouteTimeout, destRouteData->getMetric(), destRouteData->getMetric());
        updateValidRouteLifeTime(rrep->getDestAddr(), simTime() + activeRouteTimeout);
        // When any node transmits a RREP, the precursor list for the
        // corresponding destination node is updated by adding to it
        // the next hop node to which the RREP is forwarded.

        if (rrep->getHopCount() > 0) {
            if (destRoute->getNextHopAsGeneric() == rrep->getOriginatorAddr())
                throw cRuntimeError("Never loop");
            for (unsigned int i = 0; i < rrep->getAccumulateAddressArraySize(); i++) {
                if (rrep->getAccumulateAddress(i) == destRoute->getNextHopAsGeneric() ||
                        rrep->getAccumulateAddress(i) == this->getSelfIPAddress()) {
                    throw cRuntimeError("Never loop");
                }
            }
            auto outgoingRREP = dynamicPtrCast<Rrep>(rrep->dupShared());
            outgoingRREP->appendAccumulateAddress(this->getSelfIPAddress());
            forwardRREP(outgoingRREP, destRoute->getNextHopAsGeneric(), 100);
        }
    }
    else {    // create forward route for the destination: this path will be used by the originator to send data packets
        /** received routing message is an RREP and no routing entry was found **/
        sendRERRWhenNoRouteToForward(rrep->getDestAddr(), rrep->getOriginatorAddr());
    }
}

void LoadNg::updateRoutingTable(IRoute *route, const L3Address& nextHop, unsigned int hopCount, int64_t destSeqNum, bool isActive, simtime_t lifeTime, int metricType, const double & metric)
{
    EV_DETAIL << "Updating existing route: " << route << endl;

    auto it = neighbors.find(nextHop);
    if (it == neighbors.end()) {
        // include in the list
        NeigborElement elem;
//        elem.lastNotification = simTime();
        if (nextHop == route->getDestinationAsGeneric()) {
            elem.seqNumber = destSeqNum;
        }
        neighbors[nextHop] = elem;
    }
    else {
//        it->second.lastNotification = simTime();
        if (nextHop == route->getDestinationAsGeneric()) {
            it->second.seqNumber = destSeqNum;
         }
    }


    route->setNextHop(nextHop);
    route->setMetric(hopCount);
    LoadNgRouteData *routingData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
    ASSERT(routingData != nullptr);

    routingData->setLifeTime(lifeTime);
    routingData->setDestSeqNum(destSeqNum);
    routingData->setIsActive(isActive);
    routingData->setActualizeTime();


    if (metricType != -1) {
        routingData->setMetricType(metricType);
        routingData->setMetric(metric);
    }
    routingData->setHopCount(hopCount);

    EV_DETAIL << "Route updated: " << route << endl;

    scheduleExpungeRoutes();
}

void LoadNg::sendLoadNgPacket(const Ptr<LoadNgControlPacket>& packet, const L3Address& destAddr, unsigned int timeToLive, double delay)
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

bool LoadNg::actualizeMetric(const Ptr<LoadNgControlPacketWithTlv> &pkt, const double &val, double &newMetric) {
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


bool LoadNg::setMetric(const Ptr<LoadNgControlPacketWithTlv> &pkt, const double & val) {
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


bool LoadNg::getMetric(const Ptr<LoadNgControlPacketWithTlv> &pkt, double &newMetric) {
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

bool LoadNg::checkCost(const Ptr<Rreq>& rreq, LoadNgRouteData * loadNgRouteData) {

    if (rreq->getEtx() != NaN) {
        if (loadNgRouteData->getEtx() == NaN || loadNgRouteData->getEtx() >  rreq->getEtx()) {
            return true;
        }
    }

    if (rreq->getEnergy() != NaN) {
        if (loadNgRouteData->getEnergy() == NaN || loadNgRouteData->getEnergy() <  rreq->getEnergy()) {
            return true;
        }
    }

    if (rreq->getRecPower() != NaN) {
        if (loadNgRouteData->getRecPower() == NaN || loadNgRouteData->getRecPower() <  rreq->getRecPower()) {
            return true;
        }
    }



    if (rreq->getDelay() != NaN) {
        if (loadNgRouteData->getDelay() == NaN || loadNgRouteData->getDelay() >  rreq->getDelay()) {
            return true;
        }
    }
    if (rreq->getSnir() != NaN) {
        if (loadNgRouteData->getSnir() == NaN || loadNgRouteData->getSnir() <  rreq->getSnir()) {
            return true;
        }
    }
    return false;
}

void LoadNg::actualizeCost(const Ptr<Rreq>& rreq, LoadNgRouteData * loadNgRouteData) {
    loadNgRouteData->setEnergy(rreq->getEnergy());
    loadNgRouteData->setRecPower(rreq->getRecPower());
    loadNgRouteData->setNumNeig(rreq->getNumNeig());
    loadNgRouteData->setDelay(rreq->getDelay());
    loadNgRouteData->setSnir(rreq->getSnir());
    loadNgRouteData->setEtx(rreq->getEtx());
}

void LoadNg::actualizeCostUsingStatus(const NodeStatus *status, LoadNgRouteData * loadNgRouteData) {
    loadNgRouteData->setEnergy(status->energy);
    loadNgRouteData->setRecPower(status->recPower);
    loadNgRouteData->setNumNeig(status->numNeig);
    loadNgRouteData->setDelay(status->delay.dbl());
    loadNgRouteData->setSnir(status->snir);
    loadNgRouteData->setEtx(status->metric);
}

void LoadNg::actualizeCostUsingNeig(const NeigborElement *status, LoadNgRouteData * loadNgRouteData) {
    loadNgRouteData->setEnergy(status->energy);
    loadNgRouteData->setRecPower(status->powerList.back());
    loadNgRouteData->setNumNeig(status->listNeigbours.size());
    loadNgRouteData->setDelay(status->delayCost);
    loadNgRouteData->setSnir(status->snirCost);
    loadNgRouteData->setEtx(status->metricToNeig);
}


void LoadNg::handleRREQ(const Ptr<Rreq>& rreq, const L3Address& sourceAddr, unsigned int timeToLive, const MacAddress &macSenderAddress)
{
    EV_INFO << "AODV Route Request arrived with source addr: " << sourceAddr << " originator addr: " << rreq->getOriginatorAddr()
            << " destination addr: " << rreq->getDestAddr() << endl;

    // A node ignores all RREQs received from any node in its blacklist set.

    if (!macSenderAddress.isUnspecified() && !sourceAddr.isUnspecified()) {
        auto itMac = macToIpAddress.find(macSenderAddress);
        if (itMac == macToIpAddress.end()) {
            macToIpAddress[macSenderAddress] = sourceAddr;
        }
    }


    auto itNeig = neighbors.find(sourceAddr);
    if (useHelloMessages && (itNeig == neighbors.end() || !itNeig->second.isBidirectional || itNeig->second.pendingConfirmation) && !rreq->getAccumulate()) {
        EV_INFO << "The sender node " << sourceAddr << " is not in the neighbors list or is not bi-directional. Ignoring the Route Request" << endl;
        return;
    }

    auto blackListIt = blacklist.find(sourceAddr);
    if (blackListIt != blacklist.end() && !rreq->getAccumulate()) {
        EV_INFO << "The sender node " << sourceAddr << " is in our blacklist. Ignoring the Route Request" << endl;
        return;
    }

    auto itSeq = seqNumbers.find(rreq->getOriginatorAddr());

    if (itSeq != seqNumbers.end() && itSeq->second > rreq->getSeqNum())
        return;

    if (itSeq == seqNumbers.end() || (itSeq != seqNumbers.end() && itSeq->second < rreq->getSeqNum())) {
            seqNumbers[rreq->getOriginatorAddr()] = rreq->getSeqNum();
    }


    // When a node receives a RREQ, it first creates or updates a route to
    // the previous hop without a valid sequence number (see section 6.2).

    int metricType = -1;
    double metricNeg = 1;

    // HERE; Metric to neighbor
    if (useHelloMessages && measureEtx)
        metricNeg = itNeig->second.metricToNeig;


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

    double val = metricNeg;
    double metricToOrigin = 255;
    double etx = NaN;
    double delay = NaN;
    double snir = NaN;
    double recpower = NaN;
    double energy = NaN;
    int numNeigh = std::numeric_limits<int>::quiet_NaN();


    if (itNeig != neighbors.end()) {
        if (!itNeig->second.powerList.empty())
            recpower = itNeig->second.powerList.back();

        if (!itNeig->second.delayList.empty()) {
            simtime_t val;
            for (auto e : itNeig->second.delayList)
                val += e;
            val /= itNeig->second.delayList.size();
            delay = val.dbl();
        }

        if (!itNeig->second.snirList.empty()) {
            double val = 0;
            for (auto e : itNeig->second.snirList)
                val += e;
            val /= itNeig->second.snirList.size();
            snir = val;
        }

        etx = itNeig->second.metricToNeig;
        val = itNeig->second.metricToNeig;
        energy = itNeig->second.energy;
        numNeigh = itNeig->second.listNeigbours.size();
    }

    if (!actualizeMetric(rreq, val, metricToOrigin))
        metricToOrigin = 255;

    if (etx != NaN) {
        if (rreq->getEtx() == NaN)
            rreq->setEtx(etx);
        else
            rreq->setEtx(etx + rreq->getEtx());
    }

    if (delay != NaN) {
        if (rreq->getDelay() == NaN)
            rreq->setDelay(delay);
        else
            rreq->setDelay(delay + rreq->getDelay());
    }

    if (recpower != NaN) {
        if (rreq->getRecPower() == NaN)
            rreq->setRecPower(recpower);
        else if (rreq->getRecPower() > recpower)
            rreq->setRecPower(recpower);
    }

    if (energy != NaN) {
        if (rreq->getEnergy() == NaN)
            rreq->setEnergy(energy);
        else if (rreq->getEnergy() > energy)
            rreq->setEnergy(energy);
    }

    if (snir != NaN) {
        if (rreq->getSnir() == NaN)
            rreq->setSnir(snir);
        else if (rreq->getSnir() > snir)
            rreq->setSnir(snir);
    }

    if (numNeigh != NaN) {
        if (rreq->getNumNeig() == NaN)
            rreq->setNumNeig(numNeigh);
        else if (rreq->getNumNeig() > numNeigh)
            rreq->setNumNeig(numNeigh);
    }
    // End actualize metrics. Now they sould select the best alternative.


    if (rreq->getHopLimit() != 0) // actualize
        rreq->setHopLimit(rreq->getHopLimit()-1);

    IRoute *previousHopRoute = routingTable->findBestMatchingRoute(sourceAddr);

    // extract the metric to actualize the cost to the next hop
    if ((!previousHopRoute || previousHopRoute->getSource() != this) && !rreq->getAccumulate()) {
        // create without valid sequence number
        previousHopRoute = createRoute(sourceAddr, sourceAddr, 1, -1, true, simTime() + activeRouteTimeout, metricType, metricNeg);
    }
    else {
        if (previousHopRoute) {
            if (dijkstra && par("dijkstraAndRreq")) {
                std::vector<L3Address> pathNode;
                if (dijkstra->getRoute(sourceAddr, pathNode)) {
                    if (sourceAddr != pathNode[1]) { // be carefull, possible change
                        auto cost = dijkstra->getCost(sourceAddr);
                        if (cost > metricToOrigin) {
                            auto loadNgRouteData = check_and_cast<LoadNgRouteData *> (previousHopRoute->getProtocolData());
                            loadNgRouteData->setUsingDijkstra(false);
                            updateRoutingTable(previousHopRoute, sourceAddr, 1, loadNgRouteData->getDestSeqNum(), true, simTime() + activeRouteTimeout, metricType, metricNeg);
                        }
                    }
                }
            }
            else {
                auto loadNgRouteData = check_and_cast<LoadNgRouteData *> (previousHopRoute->getProtocolData());
                loadNgRouteData->setUsingDijkstra(false);
                updateRoutingTable(previousHopRoute, sourceAddr, 1, loadNgRouteData->getDestSeqNum(), true, simTime() + activeRouteTimeout, metricType, metricNeg);
            }
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

    int64_t oldSeqNum = -1;
    double oldMetric = 255;
    auto prevNodeOrigin = sourceAddr;

    if (dijkstra && par("dijkstraAndRreq")) {
        // check if exist a route to the origin
        std::vector<L3Address> pathNode;
        if (dijkstra->getRoute(rreq->getOriginatorAddr(), pathNode)) {
            auto cost = dijkstra->getCost(rreq->getOriginatorAddr());
            if (cost < metricToOrigin) {
                setMetric(rreq, cost);
                if (pathNode.size()<=1)
                    throw cRuntimeError("Path error next hop not found");
                prevNodeOrigin = pathNode[1];
                metricToOrigin = cost;
                rreq->setHopCount(pathNode.size()-1);
            }
        }
    }

    if ((!reverseRoute || reverseRoute->getSource() != this) && !rreq->getAccumulate()) {    // create
        // This reverse route will be needed if the node receives a RREP back to the
        // node that originated the RREQ (identified by the Originator IP Address).
        reverseRoute = createRoute(rreq->getOriginatorAddr(), prevNodeOrigin, hopCount, rreqSeqNum, true, newLifeTime,  metricType, metricToOrigin);
        LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(reverseRoute->getProtocolData());
        actualizeCost(rreq, routeData);
    }
    else {
        if (reverseRoute) {
            LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(reverseRoute->getProtocolData());
            int routeSeqNum = routeData->getDestSeqNum();
            oldMetric = routeData->getMetric();
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

            if (rreqSeqNum > routeSeqNum || (rreqSeqNum == routeSeqNum && metricToOrigin < oldMetric)
                    || (rreqSeqNum == routeSeqNum && metricToOrigin == oldMetric && newHopCount < routeHopCount) || checkCost(rreq, routeData)) {
                actualizeCost(rreq, routeData);
                routeData->setUsingDijkstra(false);
                updateRoutingTable(reverseRoute, prevNodeOrigin, hopCount, newSeqNum, true, newLifeTime, metricType, metricToOrigin);
            }
        }
    }

    // check 11.1. (2)
    if (oldSeqNum > rreq->getSeqNum())
        return;


    // A node generates a RREP if either:
    //
    // (i)       it is itself the destination, or
    //

    // check (i)
    if (rreq->getDestAddr() == getSelfIPAddress() && (oldSeqNum < rreq->getSeqNum() ||
            (oldSeqNum == rreq->getSeqNum() && oldMetric > metricToOrigin && par("multipleRrep").boolValue()))) {
        EV_INFO << "I am the destination node for which the route was requested" << endl;


        if (rreq->getAccumulate() && !rreq->getIncludeRoute()) {

            auto rreqR = createRREQ(rreq->getOriginatorAddr());
            rreqR->setAccumulate(true);
            rreqR->setIncludeRoute(true);
            rreqR->setAccumulateAddressArraySize(rreq->getAccumulateAddressArraySize());
            for (unsigned int i = 0; i < rreq->getAccumulateAddressArraySize(); i ++){
                rreqR->setAccumulateAddress(i, rreq->getAccumulateAddress(i));
            }
            rreqR->setChunkLength(rreq->getChunkLength()+B(rreq->getAccumulateAddressArraySize()));
            rreqR->setHopLimit(netDiameter);
            forwardRREQ(rreqR, timeToLive);
            //sendRREQ(rreqR, addressType->getBroadcastAddress(), 0);
        }
        else if (rreq->getAccumulate() && rreq->getIncludeRoute()) {
            // Special RREP
            auto rrep = createRREP(rreq, nullptr, reverseRoute, sourceAddr);
            rrep->setAccumulate(true);
            rrep->setIncludeRoute(true);
            rrep->setHopLimit(par("maxHopLimit"));
            rrep->setAccumulateAddressArraySize(rreq->getAccumulateAddressArraySize());
            for (unsigned int i = 0; i < rreq->getAccumulateAddressArraySize(); i ++){
                rrep->setAccumulateAddress(i, rreq->getAccumulateAddress(i));
            }
            rrep->setRouteArraySize(rreq->getRouteArraySize());
            for (unsigned int i = 0; i < rreq->getRouteArraySize(); i ++){
                rrep->setRoute(i, rreq->getRoute(i));
            }
            rrep->setChunkLength(rreq->getChunkLength()+B(rreq->getAccumulateAddressArraySize())+B(rreq->getRouteArraySize()));
            rrep->setSeqNumDest(rreq->getSeqNum());
            sendRREP(rrep, rreq->getOriginatorAddr(), 255, -1);

            //Store the data  in memory, introduce a small delay before send the stored packets.
            SpecialPath path;
            path.lifetime = simTime() + 2 * netTraversalTime - 2 * (rreq->getAccumulateAddressArraySize()+1) * nodeTraversalTime;
            for (unsigned int i = 0; i < rreq->getAccumulateAddressArraySize(); i ++){
                path.path.push_back(rrep->getAccumulateAddress(i));
            }

            specialPaths[rreq->getOriginatorAddr()] = path;
            if (hasOngoingRouteDiscovery(rreq->getOriginatorAddr())) {
                EV_INFO << "The Route Reply has arrived for our Route Request to node " << rreq->getOriginatorAddr() << endl;
                completeRouteDiscoveryDelayed(rreq->getOriginatorAddr());
            }
        }
        else {
            // create RREP
            auto rrep = createRREP(rreq, nullptr, reverseRoute, sourceAddr);
            rrep->setHopLimit(par("maxHopLimit"));
            //LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(reverseRoute->getProtocolData());
            //rrep->setRecPower(routeData->getRecPower());
            //rrep->setEnergy(routeData->getEnergy());
            //rrep->setDelay(routeData->getDelay());
            //rrep->setSnir(routeData->getSnir());
            //rrep->setEtx(routeData->getEtx());
            // send to the originator
            sendRREP(rrep, rreq->getOriginatorAddr(), 255, -1);
        }

        return;    // discard RREQ, in this case, we do not forward it.
    }
    else if (oldSeqNum == rreq->getSeqNum() && oldMetric > metricToOrigin) {
        // Propagate if the new path is better?, now it don't propagate it.
        // Search the packet in the delayed list to actualize it
        // No, it won't work
        return;
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
        forwardRREQ(outgoingRREQ, timeToLive);
    }
    else
        EV_WARN << " LoadNg reboot has not completed yet" << endl;
}

IRoute *LoadNg::createRoute(const L3Address& destAddr, const L3Address& nextHop,
        unsigned int hopCount, int64_t destSeqNum,
        bool isActive, simtime_t lifeTime, int metricType, const double & metric)
{
    if (destAddr == this->getSelfIPAddress()) {
        throw cRuntimeError("never ");
    }
    auto it = neighbors.find(nextHop);
    if (it == neighbors.end()) {
        // include in the list
        NeigborElement elem;
        // elem.lastNotification = simTime();
        if (nextHop == destAddr)
            elem.seqNumber = destSeqNum;
        neighbors[nextHop] = elem;
    }
    else {
        // it->second.lastNotification = simTime();
        if (nextHop == destAddr)
            it->second.seqNumber = destSeqNum;
    }

    // create a new route
    // first check if the route exist
    IRoute *route = routingTable->findBestMatchingRoute(destAddr);
    if (route != nullptr) {
        if (route->getSource() == this)
            throw cRuntimeError("route->getSource() == this");
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

    newRoute->setProtocolData(newProtocolData);

    EV_DETAIL << "Adding new route " << newRoute << endl;
    routingTable->addRoute(newRoute);

    scheduleExpungeRoutes();
    return newRoute;
}

#if 0
void LoadNg::actualizeBreakRoutes() {
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

void LoadNg::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method("receiveChangeNotification");
    if (signalID == recomputeSignal) {
        if (!recomputeTimer->isScheduled()) {
            scheduleAt(simTime(), recomputeTimer);
        }
    }
    else if (signalID == linkBrokenSignal) {

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
                if (dijkstra) {
                    if (route && route->getSource() == this) {
                        // set the neig to error
                        auto it = neighbors.find(route->getNextHopAsGeneric());
                        if (it != neighbors.end()) {
                            it->second.pendingHello = true;
                            it->second.pendingConfirmation = true;
                        }
                        // invalidate the next hop route.

                        rebuildDijkstraWihtHellos();
                        std::vector<L3Address> pathNode;
                        if (dijkstra->getRoute(unreachableAddr, pathNode)) {
                            auto it = neighbors.find(pathNode[1]);
                            if (it != neighbors.end() && it->second.isBidirectional && !it->second.pendingConfirmation) {
                                // alternative route
                                if (route->getNextHopAsGeneric() != pathNode[1])
                                    throw cRuntimeError("Check run dijkstra");
                                // re-inject the packet
                                // Now remove all the headers that are before the network header
                                PacketDissector::PduTreeBuilder pduTreeBuilder;
                                auto packetProtocolTag = datagram->findTag<PacketProtocolTag>();
                                auto protocol = packetProtocolTag != nullptr ? packetProtocolTag->getProtocol() : nullptr;
                                PacketDissector packetDissector(ProtocolDissectorRegistry::getInstance(), pduTreeBuilder);
                                packetDissector.dissectPacket(const_cast<Packet *>(datagram), protocol);
                                auto& protocolDataUnit = pduTreeBuilder.getTopLevelPdu();
                                bool isIpv4 = false;
                                bool sendError = false;
                                for (const auto& chunk : protocolDataUnit->getChunks()) {
                                    if (auto childLevel = dynamicPtrCast<const PacketDissector::ProtocolDataUnit>(chunk)) {
                                        for (const auto& chunkAux : childLevel->getChunks()) {
                                            if (chunkAux->getChunkType() == Chunk::CT_SEQUENCE) {
                                                for (const auto& elementChunk : staticPtrCast<const SequenceChunk>(chunkAux)->getChunks()) {
                                                    if (dynamic_cast<const Ipv4Header *>(elementChunk.get())) {
                                                        isIpv4 = true;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    else if (chunk->getChunkType()== Chunk::CT_SEQUENCE) {
                                        for (const auto& elementChunk : staticPtrCast<const SequenceChunk>(chunk)->getChunks()) {
                                            if (dynamic_cast<const Ipv4Header *>(elementChunk.get())) {
                                                isIpv4 = true;
                                            }
                                        }
                                    }
                                }
                                if (!isIpv4)
                                    return; // nothing more to do

                                if (sendError) {
                                    if (route && route->getSource() == this) {
                                        handleLinkBreakSendRERR(route->getNextHopAsGeneric(), sourceAddress, unreachableAddr);
                                    }
                                    return;
                                }

                                // create a copy of this packet, but only with the network headers and below.
                                auto pkt = new Packet(datagram->getName());
                                pkt->copyTags(*datagram);

                                for (const auto& chunk : protocolDataUnit->getChunks()) {
                                    if (auto childLevel = dynamicPtrCast<const PacketDissector::ProtocolDataUnit>(chunk)) {
                                        for (const auto& chunkAux : childLevel->getChunks()) {
                                            if (chunkAux->getChunkType() == Chunk::CT_SEQUENCE) {
                                                // remove previous headers to ipv4
                                                bool removed = false;
                                                for (const auto& elementChunk : staticPtrCast<const SequenceChunk>(chunkAux)->getChunks()) {
                                                    if (elementChunk == networkHeader) {
                                                        removed = true;
                                                        auto header = networkHeader->dupShared();
                                                        header->addTagIfAbsent<RetransmitTag>()->setRetransmitNode(this->getSelfIPAddress());
                                                        insertNetworkProtocolHeader(pkt, Protocol::ipv4, dynamicPtrCast<NetworkHeaderBase>(header));
                                                    }
                                                    else if (removed)
                                                        pkt->insertAtBack(elementChunk->dupShared());
                                                }
                                            }
                                        }
                                    }
                                    else if (chunk->getChunkType() == Chunk::CT_SEQUENCE) {
                                        if (staticPtrCast<const SequenceChunk>(chunk)->getChunks().front() != networkHeader) {
                                            delete pkt;
                                            return;
                                        }
                                        for (const auto& elementChunk : staticPtrCast<const SequenceChunk>(chunk)->getChunks()) {
                                            if (elementChunk == networkHeader) {
                                                insertNetworkProtocolHeader(pkt, Protocol::ipv4, dynamicPtrCast<NetworkHeaderBase>(networkHeader->dupShared()));
                                            }
                                            else
                                                pkt->insertAtBack(elementChunk->dupShared());
                                        }
                                    }
                                }
                                pkt->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::ipv4);
                                NetworkInterface *ifEntry = interfaceTable->findInterfaceByName("wlan0");
                                pkt->addTagIfAbsent<InterfaceInd>()->setInterfaceId(ifEntry->getInterfaceId());
                                networkProtocol->enqueueRoutingHook(pkt, IHook::Type::PREROUTING);
                                networkProtocol->reinjectQueuedDatagram(pkt);
                                return;
                            }
                        }
                    }
                }
                if (route && route->getSource() == this) {
                    handleLinkBreakSendRERR(route->getNextHopAsGeneric(), sourceAddress, unreachableAddr);
                }
            }
        }
    }
}

void LoadNg::handleLinkBreakSendRERR(const L3Address& unreachableAddr, const L3Address& source, const L3Address& destination)
{
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
   // int ttl = 1;
    if (route && route->getSource() == this) {
         LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
         if (routeData->isActive()) {
             nextAddress = route->getNextHopAsGeneric();
    //         ttl = route->getMetric();
         }
     }
     if (nextAddress.isBroadcast()) // Only unicast
         return;

     auto rerr = createRERR(unreachableNodes);
     rerr->setOriginatorAddr(this->getSelfIPAddress());
     rerr->setDestAddr(source);

    rerrCount++;

    // broadcast
    EV_INFO << "Broadcasting Route Error message with TTL=1" << endl;
    sendLoadNgPacket(rerr, addressType->getBroadcastAddress(), 1, *jitterPar);
}

const Ptr<Rerr> LoadNg::createRERR(const std::vector<UnreachableNode>& unreachableNodes)
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

void LoadNg::handleRERR(const Ptr<const Rerr>& rerr, const L3Address& sourceAddr)
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
            //int ttl = 1;
            if (route && route->getSource() == this) {
                LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(
                        route->getProtocolData());
                if (routeData->isActive()) {
                    nextAddress = route->getNextHopAsGeneric();
             //       ttl = route->getMetric();
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

            sendLoadNgPacket(newRERR, nextAddress, 1, 0);
            rerrCount++;
        }
    }
}

void LoadNg::handleStartOperation(LifecycleOperation *operation)
{
    rebootTime = simTime();

    // RFC 5148:
    // Jitter SHOULD be applied by reducing this delay by a random amount, so that
    // the delay between consecutive transmissions of messages of the same type is
    // equal to (MESSAGE_INTERVAL - jitter), where jitter is the random value.
    if (useHelloMessages)
        scheduleAt(simTime() + helloInterval - *periodicJitter, helloMsgTimer);
    if (measureEtx) {
        helloInterval = maxHelloInterval = minHelloInterval;
        //
        if (helloMsgTimer->isScheduled())
            cancelEvent(helloMsgTimer);
       // sendHelloMessagesIfNeeded();
        scheduleAt(simTime() + *periodicJitter, helloMsgTimer);
    }
    scheduleAt(simTime() + 1, counterTimer);
}

void LoadNg::handleStopOperation(LifecycleOperation *operation)
{
    clearState();
}

void LoadNg::handleCrashOperation(LifecycleOperation *operation)
{
    clearState();
}

void LoadNg::clearState()
{
    rerrCount = rreqCount = rreqId = sequenceNum = 0;
    addressToRreqRetries.clear();
    for (auto & elem : waitForRREPTimers)
        cancelAndDelete(elem.second);

    // FIXME: Drop the queued datagrams.
    //for (auto it = targetAddressToDelayedPackets.begin(); it != targetAddressToDelayedPackets.end(); it++)
    //    networkProtocol->dropQueuedDatagram(it->second);

    targetAddressToDelayedPackets.clear();

    waitForRREPTimers.clear();
    rreqsArrivalTime.clear();

    if (useHelloMessages)
        cancelEvent(helloMsgTimer);
    if (expungeTimer)
        cancelEvent(expungeTimer);
    if (counterTimer)
        cancelEvent(counterTimer);
    if (blacklistTimer)
        cancelEvent(blacklistTimer);
    if (rrepAckTimer)
        cancelEvent(rrepAckTimer);
}

void LoadNg::handleWaitForRREP(WaitForRrep *rrepTimer)
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

void LoadNg::forwardRREP(const Ptr<Rrep>& rrep, const L3Address& destAddr, unsigned int timeToLive)
{
    EV_INFO << "Forwarding the Route Reply to the node " << rrep->getOriginatorAddr() << " which originated the Route Request" << endl;

    if (rrep->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RREP" << endl;
        return;
    }

    IRoute *destRoute = routingTable->findBestMatchingRoute(destAddr);
    LoadNgRouteData *destRouteData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());

/*
    char myAddress[30];
    char dest[30];
    strcpy(myAddress, this->getSelfIPAddress().str().c_str());
    strcpy(dest, destAddr.str().c_str());
    */

    bool isNeigBiDir = false;
    auto it = neighbors.find(destAddr);
    if (it != neighbors.end())
        isNeigBiDir = it->second.isBidirectional;

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
    if (destAddr.isUnicast())
        sendLoadNgPacket(rrep, destAddr, 100, 0);
    else
        sendLoadNgPacket(rrep, destAddr, 100, *jitterPar);
}

void LoadNg::forwardRREQ(const Ptr<Rreq>& rreq, unsigned int timeToLive)
{
    EV_INFO << "Forwarding the Route Request message with TTL= " << timeToLive << endl;
    // TODO: unidirectional rreq
    auto next = routingTable->getNextHopForDestination(rreq->getDestAddr());
    if (!next.isUnspecified()) {
        // it is possible unicast RREQ, propagate at least one more hop
        if (rreq->getHopLimit() == 0) {
            rreq->setHopLimit(1);
        }
    }

    if (rreq->getHopLimit() == 0) {
        EV_WARN << "Hop limit 0. Canceling sending RREQ" << endl;
        return;
    }

    if (rreq->getOriginatorAddr() != this->getSelfIPAddress()) {
        if (rreq->getAccumulate() && !rreq->getIncludeRoute()) {
            rreq->setAccumulateAddressArraySize(rreq->getAccumulateAddressArraySize() + 1);
            rreq->setAccumulateAddress(rreq->getAccumulateAddressArraySize() - 1, this->getSelfIPAddress());
            rreq->setChunkLength(rreq->getChunkLength() + B(1));
        }
        else if (rreq->getAccumulate() && rreq->getIncludeRoute()) {
            rreq->setRouteArraySize(rreq->getRouteArraySize() + 1);
            rreq->setRoute(rreq->getRouteArraySize() - 1, this->getSelfIPAddress());
            rreq->setChunkLength(rreq->getChunkLength() + B(1));
        }
    }

    if (next.isUnspecified())
        sendLoadNgPacket(rreq, addressType->getBroadcastAddress(), 1, *jitterPar);
    else
        sendLoadNgPacket(rreq, next, rreq->getHopLimit(), 0);
}

void LoadNg::completeRouteDiscoveryDelayed(const L3Address& target)
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


void LoadNg::completeRouteDiscovery(const L3Address& target)
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
        networkProtocol->reinjectQueuedDatagram(datagram);
    }

    // clear the multimap
    targetAddressToDelayedPackets.erase(lt, ut);

    // we have a route for the destination, thus we must cancel the WaitForRREPTimer events
    auto waitRREPIter = waitForRREPTimers.find(target);
    ASSERT(waitRREPIter != waitForRREPTimers.end());
    cancelAndDelete(waitRREPIter->second);
    waitForRREPTimers.erase(waitRREPIter);
}

void LoadNg::sendGRREP(const Ptr<Rrep>& grrep, const L3Address& destAddr, unsigned int timeToLive)
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

void LoadNg::regenerateLinkMap() {
    if (!useCommonData)
        return;
    simtime_t interval = simTime() - lastRecomputation;
    if (interval < minRecompueTime) {
        simtime_t next = lastRecomputation + minRecompueTime;
        if (recomputeTimer->isScheduled()) {
           if (recomputeTimer->getArrivalTime() == next)  return;
        }
        else {
            cancelEvent(recomputeTimer);
        }
        scheduleAt(next, recomputeTimer);
        return;
    }

    lastRecomputation = simTime();

    dijkstraTotal->clearAll();

    for (auto elem : nodesData) {
        auto origin = elem.first;
        for (auto elem2 : *(elem.second)) {
            auto dest = elem2.first;
            if (!elem2.second.isBidirectional)
                continue;
            if (elem2.second.pendingHello)
                continue;
            if (elem2.second.pendingConfirmation)
                continue;

            //double power = NaN;
            //double delay = NaN;
            double cost = NaN;

            //if (!elem2.second.powerList.empty())
            //    power = elem2.second.powerList.back();

            if (!elem2.second.delayList.empty()) {
                simtime_t val;
                for (auto e : elem2.second.delayList)
                    val += e;
                val /= elem2.second.delayList.size();
                //delay = val.dbl();
            }
            cost = elem2.second.metricToNeig;
            dijkstraTotal->addEdge(origin, dest, cost, 1);
        }
    }

    if (simTime() < 3) {
        if (dijkstraTotal->rootNodeIncluded())
            dijkstraTotal->run();
        else
            dijkstraTotal->cleanRoutes();
    }
    else
        dijkstraTotal->run();

    std::map<L3Address, std::vector<L3Address>> paths;
    dijkstraTotal->getRoutes(paths);
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *destRoute = routingTable->getRoute(i);
        //const L3Address& nextHop = destRoute->getNextHopAsGeneric();
        auto itDest = paths.find(destRoute->getDestinationAsGeneric());
        if (itDest != paths.end()) {
            // actualize
            auto neigh = itDest->second[1];
            auto itAux = neighbors.find(neigh);
            auto itNeigList = nodesData.find(neigh);
            if (itNeigList == nodesData.end())
                throw cRuntimeError("Node not found");

            if (itAux == neighbors.end())
                throw cRuntimeError("Path error found");
            LoadNgRouteData *routingData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());
            simtime_t newLifeTime = routingData->getLifeTime();
            if (newLifeTime < itAux->second.lifeTime + par("allowedHelloLoss") * helloInterval)
                newLifeTime = itAux->second.lifeTime + par("allowedHelloLoss") * helloInterval;
            double cost = dijkstra->getCost(itDest->first);
            if (neigh == itDest->first) {
                updateRoutingTable(destRoute, itDest->first, 1, itAux->second.seqNumber, true, newLifeTime, DIMENSIONLESS, cost);
            }
            else {

                auto dest = itDest->second[2];
                auto itAux2 = itNeigList->second->find(dest);
                if (itAux2 == itNeigList->second->end())
                    throw cRuntimeError("Path error found");
                updateRoutingTable(destRoute, neigh, itDest->second.size() - 1, itAux2->second.seqNumber, true, newLifeTime, DIMENSIONLESS, cost);
            }
            paths.erase(itDest);
        }
    }
    for (const auto &elem : paths) {
         auto neigh = elem.second[1];
         auto itAux = neighbors.find(neigh);
         if (itAux == neighbors.end())
             throw cRuntimeError("Path error found");

         auto itNeigList = nodesData.find(neigh);
         auto newLifeTime = itAux->second.lifeTime + par("allowedHelloLoss") * helloInterval;
         double cost = dijkstra->getCost(elem.first);
         if (neigh == elem.first) {
             createRoute(elem.first, elem.first, 1, itAux->second.seqNumber, true, newLifeTime, DIMENSIONLESS, cost );
         }
         else {
             if (elem.second.size() <= 2)
                 throw cRuntimeError("Incorrect size");

             auto destination = elem.second[2];
             auto itAux2 = itNeigList->second->find(destination);
             if (itAux2 == itNeigList->second->end())
                 throw cRuntimeError("Path error found");

             createRoute(elem.first, neigh, elem.second.size() - 1, itAux2->second.seqNumber, true, newLifeTime, HOPCOUNT, cost);
         }
     }
}

void LoadNg::rebuildDijkstraWihtHellos()
{
    if (dijkstra == nullptr)
        return;
    dijkstra->clearAll();
    for (auto it = neighbors.begin(); it != neighbors.end(); ) {
        // this should be in function of the mobility,
        if (simTime() - it->second.lastNotification >= helloInterval * par("allowedHelloLoss")) {
            neighbors.erase(it++);
            continue;
        }
       if (it->second.isBidirectional && !it->second.pendingConfirmation) {
           dijkstra->addEdge(this->getSelfIPAddress(), it->first, it->second.metricToNeig, 0);
           dijkstra->addEdge(it->first, this->getSelfIPAddress(), it->second.metricToNeig, 0);
       }
       for (const auto &elem2 : it->second.listNeigbours) {
           if (elem2.second.isBidirectional && !elem2.second.pendingConfirmation) {
               dijkstra->addEdge(it->first, elem2.first, elem2.second.metric, 0);
               dijkstra->addEdge(elem2.first, it->first, elem2.second.metric, 0);
           }
       }
       ++it;
    }
    runDijkstra();
}


void LoadNg::runDijkstra()
{
    dijkstra->cleanRoutes();
    if (!dijkstra->nodeExist(this->getSelfIPAddress()))
        return;
    dijkstra->run();
    std::map<L3Address, std::vector<L3Address>> paths;
    dijkstra->getRoutes(paths);

    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *destRoute = routingTable->getRoute(i);

        if (destRoute->getSource() != this)
            continue; // ignore this

        LoadNgRouteData *routingData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());

        //const L3Address& nextHop = destRoute->getNextHopAsGeneric();
        auto itDest = paths.find(destRoute->getDestinationAsGeneric());
        if (itDest != paths.end()) {
            // actualize
            auto neigh = itDest->second[1];
            auto itAux = neighbors.find(neigh);
            if (itAux == neighbors.end())
                throw cRuntimeError("Path error found");

            routingData->setUsingDijkstra(true);
            simtime_t newLifeTime = routingData->getLifeTime();
            if (newLifeTime < itAux->second.lifeTime + par("allowedHelloLoss") * helloInterval)
                newLifeTime = itAux->second.lifeTime + par("allowedHelloLoss") * helloInterval;

            double cost = dijkstra->getCost(itDest->first);
            if (neigh == itDest->first) {
                updateRoutingTable(destRoute, itDest->first, 1, itAux->second.seqNumber, true, newLifeTime, DIMENSIONLESS, cost);
            }
            else {
                if (itDest->second.size() <= 2)
                    throw cRuntimeError("Incorrect size");
                auto dest = itDest->second[2];
                auto itAux2 = itAux->second.listNeigbours.find(dest);
                if (itAux2 == itAux->second.listNeigbours.end()) {
                    // A node has reported a neighbor but other not, inconsistency
                    // Mark nodes
                    auto it1 = neighbors.find(itDest->second[1]);
                    auto it2 = neighbors.find(itDest->second[2]);
                    if (it1 == neighbors.end() || it2 == neighbors.end())
                        throw cRuntimeError("Path error found Link, nodes  %s and %s should be both neiggbors",
                                                        itDest->second[1].str().c_str(),itDest->second[2].str().c_str());
                    auto it1Neigh = it1->second.listNeigbours.find(it2->first);
                    auto it2Neigh = it2->second.listNeigbours.find(it1->first);

                    if ((it1Neigh != it1->second.listNeigbours.end() && it2Neigh == it2->second.listNeigbours.end()) ||
                            (it1Neigh == it1->second.listNeigbours.end() && it2Neigh != it2->second.listNeigbours.end())){
                        if (it1Neigh != it1->second.listNeigbours.end())
                            it1->second.listNeigbours.erase(it1Neigh);
                        if (it2Neigh != it2->second.listNeigbours.end())
                            it2->second.listNeigbours.erase(it2Neigh);
                        rebuildDijkstraWihtHellos();
                        return;
                    }
                    else {
                        // error
                        throw cRuntimeError("Path error found Link, edge from node %s to node %s doesn't exist",
                                                        itDest->second[1].str().c_str(),itDest->second[2].str().c_str());
                    }
                    throw cRuntimeError("Path error found Link node %s to node %s defined but node %s is not a neighbor of %s",
                            itDest->second[1].str().c_str(),itDest->second[2].str().c_str(), itAux->first.str().c_str(), dest.str().c_str());
                }

                updateRoutingTable(destRoute, neigh, itDest->second.size() - 1, itAux2->second.seqNum, true, newLifeTime, DIMENSIONLESS, cost);
            }
            paths.erase(itDest);
        }
        else if (routingData->getUsingDijkstra()) {
            routingData->setIsActive(false);
        }
        else {
            auto itDest = neighbors.find(destRoute->getDestinationAsGeneric());
            if (itDest == neighbors.end() || itDest->second.lifeTime >= simTime())
                routingData->setIsActive(false);
        }
    }

    for (const auto &elem : paths) {
        auto neigh = elem.second[1];
        auto itAux = neighbors.find(neigh);
        if (itAux == neighbors.end())
            throw cRuntimeError("Path error found");
        auto newLifeTime = itAux->second.lifeTime + par("allowedHelloLoss") * helloInterval;
        double cost = dijkstra->getCost(elem.first);
        if (neigh == elem.first) {
            auto destRoute = createRoute(elem.first, elem.first, 1, itAux->second.seqNumber, true, newLifeTime, DIMENSIONLESS, cost );
            LoadNgRouteData *routingData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());
            routingData->setUsingDijkstra(true);
        }
        else {
            if (elem.second.size() <= 2)
                throw cRuntimeError("Incorrect size");

            auto destination = elem.second[2];
            auto itAux2 = itAux->second.listNeigbours.find(destination);
            if (itAux2 == itAux->second.listNeigbours.end())
                throw cRuntimeError("Path error found");
            auto destRoute = createRoute(elem.first, neigh, elem.second.size() - 1, itAux2->second.seqNum, true, newLifeTime, HOPCOUNT, cost);
            LoadNgRouteData *routingData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());
            routingData->setUsingDijkstra(true);
        }
    }
}

void LoadNg::runDijkstraKs()
{

    throw cRuntimeError("runDijkstraKs unsuported yet");

    dijkstraKs->run();
    //std::map<L3Address, std::vector<std::vector<L3Address>>>paths;

    dijkstraKs->getAllRoutes(alternativePaths);
//    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
//        IRoute *destRoute = routingTable->getRoute(i);
//        //const L3Address& nextHop = destRoute->getNextHopAsGeneric();
//        auto itDest = paths.find(destRoute->getDestinationAsGeneric());
//        if (itDest != paths.end()) {
//            // actualize
//            auto neigh = itDest->second[1];
//            auto itAux = neighbors.find(neigh);
//            if (itAux == neighbors.end())
//                throw cRuntimeError("Path error found");
//            LoadNgRouteData *routingData = check_and_cast<LoadNgRouteData *>(destRoute->getProtocolData());
//            simtime_t newLifeTime = routingData->getLifeTime();
//            if (newLifeTime < itAux->second.lifeTime + 3 * minHelloInterval)
//                newLifeTime = itAux->second.lifeTime + 3 * minHelloInterval;
//
//            if (neigh == itDest->first) {
//                updateRoutingTable(destRoute, itDest->first, 1, itAux->second.seqNumber, true, newLifeTime, HOPCOUNT, 1 );
//            }
//            else {
//                auto itAux2 = itAux->second.listNeigbours.find(neigh);
//                if (itAux2 == itAux->second.listNeigbours.end())
//                    throw cRuntimeError("Path error found");
//                updateRoutingTable(destRoute, neigh, 2, itAux2->second.seqNum, true, newLifeTime, HOPCOUNT, 2 );
//            }
//            paths.erase(itDest);
//        }
//        else {
//            // remove
//            routingTable->removeRoute(destRoute);
//            // Purge all routes with the same next hop?
//        }
//    }
//    for (const auto &elem : paths) {
//
//        auto neigh = elem.second[1];
//        auto itAux = neighbors.find(neigh);
//        if (itAux == neighbors.end())
//            throw cRuntimeError("Path error found");
//
//        auto newLifeTime = itAux->second.lifeTime + 3 * minHelloInterval;
//        if (neigh == elem.first) {
//            createRoute(elem.first, elem.first, 1, itAux->second.seqNumber, true, newLifeTime, HOPCOUNT, 1 );
//        }
//        else {
//            auto itAux2 = itAux->second.listNeigbours.find(neigh);
//            if (itAux2 == itAux->second.listNeigbours.end())
//                throw cRuntimeError("Path error found");
//            createRoute(elem.first, neigh, 2, itAux2->second.seqNum, true, newLifeTime, HOPCOUNT, 2 );
//        }
//    }
}

void LoadNg::checkNeigList(cMessage *timer)
{
    bool recompute = false;
    for (auto it = neighbors.begin(); it != neighbors.end();) {
        if (simTime() > it->second.lifeTime + par("allowedHelloLoss") * helloInterval) {
            std::vector<IRoute *> routesToDelete;
            for (int i = 0; i< routingTable->getNumRoutes(); i++) {
                IRoute * route = routingTable->getRoute(i);
                if (route->getNextHopAsGeneric() == it->first) {
                    routesToDelete.push_back(route);
                }
            }
            while (!routesToDelete.empty()) {
                routingTable->deleteRoute(routesToDelete.back());
                routesToDelete.pop_back();
            }
            // delete it
            recompute = true;
            neighbors.erase(it++);
        }
        else {
            if (simTime() > it->second.lifeTime + 2 * minHelloInterval) {
                helloInterval = minHelloInterval;
                it->second.pendingConfirmation = true;
                if (timer != helloMsgTimer) {
                    if (!helloMsgTimer->isScheduled()) {
                        // schedule immediately
                        scheduleAt(simTime(), helloMsgTimer);
                    }
                    else {
                        simtime_t schduled = helloMsgTimer->getSendingTime();
                        simtime_t arrival = helloMsgTimer->getArrivalTime();
                        simtime_t interval = arrival - schduled;
                        if (interval > minHelloInterval) {
                            // Schedule immediately
                            cancelEvent(helloMsgTimer);
                            scheduleAt(simTime(), helloMsgTimer);
                        }
                    }
                }
            }
            ++it;
        }
    }
    if (recompute && dijkstra) {
        emit(recomputeSignal, this);
        rebuildDijkstraWihtHellos();
    }
    if (recompute && dijkstraKs) {
        runDijkstraKs();
    }

}


const Ptr<Hello> LoadNg::createHelloMessage()
{
    // called a Hello message, with the RREP
    // message fields set as follows:
    //
    //    Destination IP Address         The node's IP address.
    //
    //    Destination Sequence Number    The node's latest sequence number.
    //
    //    Hop Count                      0
    //
    //    Lifetime                       ALLOWED_HELLO_LOSS *HELLO_INTERVAL

    auto helloMessage = makeShared<Hello>(); // TODO: "AODV-HelloMsg");
    helloMessage->setPacketType(HELLO);
    helloMessage->setSeqNum(sequenceNum);
    helloMessage->setOriginatorAddr(getSelfIPAddress());
    helloMessage->setHopCount(0);
    helloMessage->setHelloIdentifier(helloIdentifier++);
    helloMessage->setHopLimit(1);
    helloMessage->setChunkLength(B(34));

    helloMessage->setCreationTime(simTime());
    if (energyStorage)
        helloMessage->setEnergy(energyStorage->getResidualEnergyCapacity().get());

    // include neighbors list


    if (!(neighbors.empty())) {
        int s = (neighbors.size()-1)*1 + std::ceil(double(neighbors.size()*2)/8.0) + 6;
        if (this->getSelfIPAddress().getType() == L3Address::IPv4)
            s += 4;
        else if (this->getSelfIPAddress().getType() == L3Address::IPv6)
            s += 16;
        else if (this->getSelfIPAddress().getType() == L3Address::MAC)
            s += 6;
        else
            throw cRuntimeError("Unknown address type");

        if (s < 47)
            helloMessage->setNeighAddrsArraySize(neighbors.size());
        else {
            s = 6 + (30-1)*1 + std::ceil(double(30*2)/8.0);
            helloMessage->setNeighAddrsArraySize(30);
            if (this->getSelfIPAddress().getType() == L3Address::IPv4)
                s += 4;
            else if (this->getSelfIPAddress().getType() == L3Address::IPv6)
                s += 16;
            else if (this->getSelfIPAddress().getType() == L3Address::MAC)
                s += 6;
        }
        unsigned int k = 0;
        for (auto &elem : neighbors){
            NeigborData nData;
            nData.setAddr(elem.first);
            nData.setIsBidir(elem.second.isBidirectional);
            nData.setPendingConfirmation(elem.second.pendingConfirmation);
            nData.setSeqNum(elem.second.seqNumber);
            nData.setEnergy(NaN);
            nData.setRecPower(NaN);
            nData.setSnir(NaN);

            if (!elem.second.snirList.empty()) {
                double val = 0;
                for (auto e : elem.second.snirList)
                    val += e;
                val /= elem.second.snirList.size();
                nData.setSnir(val);
            }

            if (!elem.second.powerList.empty())
                nData.setRecPower(elem.second.powerList.back());

            if (!elem.second.delayList.empty()) {
                simtime_t val;
                for (auto e : elem.second.delayList)
                    val += e;
                val /= elem.second.delayList.size();
                nData.setDelay(val);
            }


            // HERE inform to the neighbors about the metricToNeig.
            if (measureEtx) {
                // delete old entries.
                while (!elem.second.helloTime.empty() && (simTime() - elem.second.helloTime.front()) > numHellosEtx * minHelloInterval)
                    elem.second.helloTime.pop_front();
                nData.setNumHelloRec(elem.second.helloTime.size());

                if (elem.second.helloTime.size() == 0 || elem.second.numHelloRec == 0)
                    elem.second.metricToNeig = 255;
                else {
                    double val = (double)(elem.second.numHelloRec * elem.second.helloTime.size())/(double)(numHellosEtx*numHellosEtx);
                    if (val > 1) val = 1;
                    val = 1/val;
                    if (val > 255) elem.second.metricToNeig = 255;
                    else elem.second.metricToNeig = val;
                }
            }

            //
            nData.setMetric(elem.second.metricToNeig);
            helloMessage->setNeighAddrs(k, nData);

            k++;
            if (k >= helloMessage->getNeighAddrsArraySize())
                break;
        }
        // size
        helloMessage->setChunkLength(helloMessage->getChunkLength() + B(s));
    }
   // include metrict.
    /*
    auto metric = new LoadNgMetricOption();
    metric->setValue(0);
    helloMessage->getTlvOptionsForUpdate().appendTlvOption(metric);

    */

    return helloMessage;
}

void LoadNg::sendHelloMessagesIfNeeded()
{
    ASSERT(useHelloMessages);
    // Every HELLO_INTERVAL milliseconds, the node checks whether it has
    // sent a broadcast (e.g., a RREQ or an appropriate layer 2 message)
    // within the last HELLO_INTERVAL.  If it has not, it MAY broadcast
    // a RREP with TTL = 1

    if (measureEtx) {
        // new to scheduleHello
        simtime_t nextHello = simTime() + helloInterval - *periodicJitter;
        auto helloMessage = createHelloMessage();
        helloMessage->setLifetime(nextHello);
        sendLoadNgPacket(helloMessage, addressType->getBroadcastAddress(), 1, 0);
        scheduleAt(nextHello, helloMsgTimer);
        return;
    }

    if (!par("useIntelligentHello").boolValue()) {
        // send hello
        simtime_t nextHello = simTime() + helloInterval - *periodicJitter;
        scheduleAt(nextHello, helloMsgTimer);
        auto helloMessage = createHelloMessage();
        helloMessage->setLifetime(nextHello);
        sendLoadNgPacket(helloMessage, addressType->getBroadcastAddress(), 1, 0);
        scheduleAt(nextHello, helloMsgTimer);
        return;
    }

    // A node SHOULD only use hello messages if it is part of an
    // active route.
    bool hasActiveRoute = false;

    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *route = routingTable->getRoute(i);
        if (route->getSource() == this) {
            LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
            if (routeData->isActive()) {
                hasActiveRoute = true;
                break;
            }
        }
    }

    simtime_t nextHello = simTime() + helloInterval - *periodicJitter;
    if (hasActiveRoute && (lastBroadcastTime == 0 || simTime() - lastBroadcastTime > helloInterval)) {
        EV_INFO << "It is hello time, broadcasting Hello Messages with TTL=1" << endl;
        // sequenceNum++;
        auto helloMessage = createHelloMessage();
        helloMessage->setLifetime(nextHello);
        sendLoadNgPacket(helloMessage, addressType->getBroadcastAddress(), 1, 0);
    }

    scheduleAt(nextHello, helloMsgTimer);
}

#if 0
void LoadNg::handleHelloMessage(const Ptr<const Hello>& helloMessage, SignalPowerInd * powerInd, SnirInd * snirInd)
{
    double power = NaN;
    double snir = NaN;
    if (powerInd)
        power = powerInd->getPower().get();
    if (snirInd)
        snir = snirInd->getMaximumSnir();

    auto delay = simTime() - helloMessage->getCreationTime();
    const L3Address& helloOriginatorAddr = helloMessage->getOriginatorAddr();
    auto it = neighbors.find(helloOriginatorAddr);
    bool changeTimerNotification = false;
    if (it != neighbors.end()) {
        // refresh
        if (helloMessage->getSeqNum() < it->second.seqNumber)
            return;
    }
    else {
        // add
        NeigborElement elem;
        elem.isBidirectional = false;
        neighbors[helloOriginatorAddr] = elem;
        it = neighbors.find(helloOriginatorAddr);
        changeTimerNotification = true;
    }

    it->second.lifeTime = helloMessage->getLifetime();
    it->second.lastNotification = simTime();
    it->second.pendingConfirmation = false;
    it->second.pendingHello = false;
    it->second.seqNumber = helloMessage->getSeqNum();
    if (power != NaN)
        it->second.powerList.push_back(power);
    if (snir != NaN)
        it->second.snirList.push_back(snir);
    it->second.delayList.push_back(delay);
    while (it->second.powerList.size() > 10)
        it->second.powerList.pop_front();
    while (it->second.snirList.size() > 10)
        it->second.snirList.pop_front();
    while (it->second.delayList.size() > 10)
        it->second.delayList.pop_front();

    if (measureEtx) {
        it->second.helloTime.push_back(simTime());
        // delete old entries.
        while (!it->second.helloTime.empty() && (simTime() - it->second.helloTime.front()) > numHellosEtx * minHelloInterval)
            it->second.helloTime.pop_front();
    }

    int metricType = -1;
    unsigned int metric = 255;
    unsigned int metricNeg = 255;
    auto metricPos = helloMessage->getTlvOptions().findByType(METRIC);

    if (metricPos != -1) {
        auto metricTlv = check_and_cast<const LoadNgMetricOption *>(helloMessage->getTlvOptions().getTlvOption(metricPos));
        if (metricTlv->getExtensionFlag())
            metricType = metricTlv->getExtension();
        if (metricTlv->getValueFlag())
            metric = metricTlv->getValue();
        if (metricType == HOPCOUNT) {
            if (metric < 255)
                metric++;
            metricNeg = 1;
        }
        else {
            // unknow metricToNeig set to hopcount
            metric = 255;
        }
    }

    std::map<L3Address, NodeStatus> nodeStatus;
    for (int k = 0 ; k < helloMessage->getNeighAddrsArraySize(); k++) {
        auto nData = helloMessage->getNeighAddrs(k);
        NodeStatus status;
        status.isBidirectional = nData.isBidir();
        status.pendingConfirmation = nData.getPendingConfirmation();
        status.seqNum = nData.getSeqNum();
        status.metric = nData.getMetric();
        status.numHelloRec = nData.getNumHelloRec();
        status.delay = nData.getDelay();
        status.power = nData.getPower();
        status.snir = nData.getSnir();

        nodeStatus[nData.getAddr()] = status;
    }

    // TODO: In addEdge replace 1.0 value for the appropriate value of the metrics.

    // First check nodes that are in the list, remove nodes that are not in the new list, and actualize the others
    // Check if the list of neighbors of this node has change
    for (auto itAux = it->second.listNeigbours.begin(); itAux != it->second.listNeigbours.end(); ) {
        //
        if (this->getSelfIPAddress() == itAux->first) {
            ++itAux;
            continue;
        }
        auto itStatus =  nodeStatus.find(itAux->first);
        if (itStatus ==  nodeStatus.end()) {
            if (itAux->second.isBidirectional && dijkstra) {
                dijkstra->deleteEdge(itAux->first, it->first);
                dijkstra->deleteEdge(it->first, itAux->first);
            }
            if (itAux->second.isBidirectional && dijkstraKs) {
                dijkstraKs->deleteEdge(itAux->first, it->first);
                dijkstraKs->deleteEdge(it->first, itAux->first);
            }
            it->second.listNeigbours.erase(itAux++);
        }
        else {
            // now check change of status
            Dijkstra::Edge *edge1 = nullptr;
            Dijkstra::Edge *edge2 = nullptr;
            if (dijkstra) {
                edge1 = const_cast<Dijkstra::Edge *>(dijkstra->getEdge(itStatus->first, itAux->first));
                edge2 = const_cast<Dijkstra::Edge *>(dijkstra->getEdge(itAux->first, itStatus->first));
            }

            double val = itStatus->second.metric;

            if (itStatus->second.isBidirectional && !itStatus->second.pendingConfirmation &&
                    (edge1 == nullptr || edge1->cost != val)) {
                // Add or modify values
                if (dijkstra) {
                    // This should be modified to include the aggregate cost and the ohter cost
                    dijkstra->addEdge(itStatus->first, itAux->first, val, 0);
                    dijkstra->addEdge(itAux->first, itStatus->first, val, 0);
                }
                if (dijkstraKs) {
                    dijkstraKs->addEdge(itStatus->first, itAux->first, val);
                    dijkstraKs->addEdge(itAux->first, itStatus->first, val);
                }
            }
            else if ((!itStatus->second.isBidirectional
                    || itStatus->second.pendingConfirmation)
                    && edge1 != nullptr) {
                if (dijkstra) {
                    dijkstra->deleteEdge(itStatus->first, itAux->first);
                    dijkstra->deleteEdge(itAux->first, itStatus->first);
                }
                if (dijkstraKs) {
                    dijkstraKs->deleteEdge(itStatus->first, itAux->first);
                    dijkstraKs->deleteEdge(itAux->first, itStatus->first);
                }
            }

            itAux->second = itStatus->second;
            ++itAux;

            // check if common neighbor
            auto itAux = it->second.listNeigbours.find(itStatus->first);
            if (itAux != it->second.listNeigbours.end()) {
                // check changes.
                // check if this node is also a common neighbor.
                auto addr = itAux->first;
                auto itNeigAux = neighbors.find(addr);
                if (itNeigAux != neighbors.end() && itNeigAux->second.seqNumber < itAux->second.seqNum) {
                    itNeigAux->second.pendingConfirmation = true;
                    changeTimerNotification = true;
                }
            }
            nodeStatus.erase(itStatus);
        }
    }

    auto itCheckBiDir = nodeStatus.find(this->getSelfIPAddress());
    if (itCheckBiDir == nodeStatus.end()) {
        if (it->second.isBidirectional ) {
            changeTimerNotification = true;
            if (dijkstra) {
                dijkstra->deleteEdge( this->getSelfIPAddress(), it->first);
                dijkstra->deleteEdge(it->first,  this->getSelfIPAddress());
            }
            it->second.isBidirectional = false;
            it->second.metricToNeig = 255;
        }
    }
    else {
        // Compute ETX if is needed. HERE IT IS POSSIBLE TO COMPUTE THE METRICS

        it->second.numHelloRec = itCheckBiDir->second.numHelloRec;
        if (measureEtx) {
            if (itCheckBiDir->second.numHelloRec == 0)
                it->second.metricToNeig = 255;
            else {
                double val = 0;
                if (simTime() >= numHellosEtx * minHelloInterval) {
                    val = (double)(itCheckBiDir->second.numHelloRec * it->second.helloTime.size());
                    val /= (double)(numHellosEtx*numHellosEtx);
                }
                else {
                    int hellos = simTime()/minHelloInterval;
                    if (hellos != 0) {
                        val =  (double) (itCheckBiDir->second.numHelloRec * it->second.helloTime.size());
                        val /= (double) (hellos * hellos);
                    }
                    else
                        val = 0.01;
                }
                if (val > 1) val = 1;
                val = 1/val;
                if (val > 255)
                    it->second.metricToNeig = 255;
                else
                    it->second.metricToNeig = val;
            }
        }
            // now should actualize the routing table
        // bi-dir
        if (it->second.isBidirectional)
            it->second.pendingConfirmation = false;
        if (!it->second.isBidirectional) {
            changeTimerNotification = true;
            double cost = it->second.metricToNeig;
            if (cost == 100 && simTime() > 10)
                printf("");
            if (dijkstra) {
                dijkstra->addEdge( this->getSelfIPAddress(), it->first, cost, 0);
                dijkstra->addEdge(it->first,  this->getSelfIPAddress(), cost, 0);
            }
            if (dijkstraKs) {
                dijkstraKs->addEdge( this->getSelfIPAddress(), it->first, cost);
                dijkstraKs->addEdge(it->first,  this->getSelfIPAddress(), cost);
            }
            it->second.isBidirectional = true;
            it->second.pendingConfirmation = false;
        }


        if (itCheckBiDir->second.pendingConfirmation) // the other node needs confirmation
            changeTimerNotification = true;

    }

    // Add new elements
    // Here it is necessary to include the metrics in dijkstra, change the value 1.0 for the correct value
    for (auto elem : nodeStatus) {
        if (elem.first == this->getSelfIPAddress())
            continue;
        // include in the list
        it->second.listNeigbours[elem.first] = elem.second;
        double cost = elem.second.metric;
        if (elem.second.isBidirectional && !elem.second.pendingConfirmation) {
            // configuration change
            if (dijkstra) {
                dijkstra->addEdge(elem.first, it->first, cost, 0);
                dijkstra->addEdge(it->first, elem.first, cost, 0);
            }
            if (dijkstraKs) {
                dijkstraKs->addEdge(elem.first, it->first, 1.0);
                dijkstraKs->addEdge(it->first, elem.first, 1.0);
            }
        }
        else {
            if (dijkstra) {
                dijkstra->deleteEdge(elem.first, it->first);
                dijkstra->deleteEdge(it->first, elem.first);
            }
        }
    }


    if (it->second.isBidirectional && !it->second.pendingConfirmation) {
        // if the link is biDirectional, delete from blacklist
        auto blackListIt = blacklist.find(it->first);
        if (blackListIt != blacklist.end())
            blacklist.erase(blackListIt);
    }
    else {
        blacklist[it->first] = simTime() + blacklistTimeout;
    }

    if (dijkstra && dijkstra->isRouteMapEmpty()) {
        runDijkstra();
    }
    if (dijkstraKs && dijkstraKs->isRouteMapEmpty()) {
        runDijkstraKs();
    }

    int numRoutes = routingTable->getNumRoutes();
    if (dijkstraTotal && (dijkstraTotal->getNumRoutes() < nodesData.size() - 1 || dijkstraTotal->getNumRoutes() != numRoutes)) {
        emit(recomputeSignal, this);
    }

    // integrity check
    auto itAuxCheck = it->second.listNeigbours.find(this->getSelfIPAddress());
    if (itAuxCheck == it->second.listNeigbours.end() && it->second.isBidirectional) {

    }



    // Whenever a node receives a Hello message from a neighbor, the node
    // SHOULD make sure that it has an active route to the neighbor, and
    // create one if necessary.  If a route already exists, then the
    // Lifetime for the route should be increased, if necessary, to be at
    // least ALLOWED_HELLO_LOSS * HELLO_INTERVAL.  The route to the
    // neighbor, if it exists, MUST subsequently contain the latest
    // Destination Sequence Number from the Hello message.  The current node
    // can now begin using this route to forward data packets.  Routes that
    // are created by hello messages and not used by any other active routes
    // will have empty precursor lists and would not trigger a RERR message
    // if the neighbor moves away and a neighbor timeout occurs.
    IRoute *routeHelloOriginator = routingTable->findBestMatchingRoute(helloOriginatorAddr);

    unsigned int latestDestSeqNum = helloMessage->getSeqNum();
    simtime_t newLifeTime = helloMessage->getLifetime() + par("allowedHelloLoss") * minHelloInterval;

    if (!routeHelloOriginator || routeHelloOriginator->getSource() != this)
        routeHelloOriginator = createRoute(helloOriginatorAddr, helloOriginatorAddr, 1, latestDestSeqNum, true, newLifeTime, metricType, metricNeg );
    else {
        LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(routeHelloOriginator->getProtocolData());
        simtime_t lifeTime = routeData->getLifeTime();
        updateRoutingTable(routeHelloOriginator, helloOriginatorAddr, 1, latestDestSeqNum, true, std::max(lifeTime, newLifeTime), metricType, metricNeg);
    }
    /*
    LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(routeHelloOriginator->getProtocolData());
    routeData->setPower(rreq->getPower());
    routeData->setDelay(rreq->getDelay());
    routeData->setSnir(rreq->getSnir());
    routeData->setEtx(rreq->getEtx());
    */


    // Now, it is necessary to refresh two hops routes that have this node like next hop
    //
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IRoute *route = routingTable->getRoute(i);
        if (route->getDestinationAsGeneric() != route->getNextHopAsGeneric() &&
                route->getNextHopAsGeneric() == helloOriginatorAddr) {

            if (dijkstra) {
                std::vector<NodeId> pathNode;
                if (dijkstra->getRoute(route->getDestinationAsGeneric(), pathNode)) {
                    if (pathNode[1] == helloOriginatorAddr) {
                        auto itAux = it->second.listNeigbours.find(route->getDestinationAsGeneric());
                        // possible two or more hops
                        //if (itAux == it->second.listNeigbours.end())
                        //    throw cRuntimeError("");
                        double cost = dijkstra->getCost(route->getDestinationAsGeneric());
                        auto seqNum = itAux->second.seqNum;
                        updateRoutingTable(route, helloOriginatorAddr, 1, seqNum, true, newLifeTime, metricType, cost);
                    }
                }
            }
        }
    }

    // and finally, include the rest of two hop nodes
    for (auto elem: it->second.listNeigbours) {
        if (elem.first == this->getSelfIPAddress())
            continue;
        if (elem.second.isBidirectional || elem.second.pendingConfirmation)
            continue;
        auto route = routingTable->findBestMatchingRoute(elem.first);
        if (route == nullptr) {
            double cost = 2;

            if (dijkstra) cost = dijkstra->getCost(elem.first);
            createRoute(elem.first, helloOriginatorAddr, 2, elem.second.seqNum, true, newLifeTime, metricType, cost );
        }
    }

    // add the information of the hello to the routing table
    if (changeTimerNotification) {
        helloInterval = minHelloInterval;
        if (!helloMsgTimer->isScheduled()) {
            // schedule immediately
            scheduleAt(simTime(), helloMsgTimer);
        }
        else {
            simtime_t schduled = helloMsgTimer->getSendingTime();
            simtime_t arrival = helloMsgTimer->getArrivalTime();
            simtime_t interval = arrival - schduled;
            if (interval > minHelloInterval) {
                // Schedule immediately
                cancelEvent(helloMsgTimer);
                scheduleAt(simTime(), helloMsgTimer);
            }
        }
    }
    else {
        helloInterval += 2.0;
        if (helloInterval > maxHelloInterval) {
            helloInterval = maxHelloInterval;
        }
    }

    // change bi dir
    LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(routeHelloOriginator->getProtocolData());

    routeData->setIsBidirectiona(it->second.isBidirectional);


    // TODO: This feature has not implemented yet.
    // A node MAY determine connectivity by listening for packets from its
    // set of neighbors.  If, within the past DELETE_PERIOD, it has received
    // a Hello message from a neighbor, and then for that neighbor does not
    // receive any packets (Hello messages or otherwise) for more than
    // ALLOWED_HELLO_LOSS * HELLO_INTERVAL milliseconds, the node SHOULD
    // assume that the link to this neighbor is currently lost.  When this
    // happens, the node SHOULD proceed as in Section 6.11.

}
#endif

void LoadNg::handleHelloMessage(const Ptr<const Hello>& helloMessage, const Ptr<const SignalPowerInd> &powerInd, const Ptr<const SnirInd> &snirInd, const MacAddress &macSenderAddress)
{
    double power = NaN;
    double snir = NaN;
    if (powerInd)
        power = powerInd->getPower().get();
    if (snirInd)
        snir = snirInd->getMaximumSnir();

    auto delay = simTime() - helloMessage->getCreationTime();
    const L3Address& helloOriginatorAddr = helloMessage->getOriginatorAddr();

    if (!macSenderAddress.isUnspecified() && !helloOriginatorAddr.isUnspecified()) {
        auto itMac = macToIpAddress.find(macSenderAddress);
        if (itMac == macToIpAddress.end()) {
            macToIpAddress[macSenderAddress] = helloOriginatorAddr;
        }
    }

    auto it = neighbors.find(helloOriginatorAddr);
    bool changeTimerNotification = false;
    if (it != neighbors.end()) {
        // refresh
        if (helloMessage->getSeqNum() < it->second.seqNumber)
            return;
    }
    else {
        // add
        NeigborElement elem;
        elem.isBidirectional = false;
        elem.creationTime = simTime();
        neighbors[helloOriginatorAddr] = elem;
        it = neighbors.find(helloOriginatorAddr);
        changeTimerNotification = true;
    }

    it->second.lifeTime = helloMessage->getLifetime();
    it->second.lastNotification = simTime();
    it->second.pendingConfirmation = false;
    it->second.pendingHello = false;
    it->second.seqNumber = helloMessage->getSeqNum();
    it->second.energy = helloMessage->getEnergy();


    if (power != NaN)
        it->second.powerList.push_back(power);
    if (snir != NaN)
        it->second.snirList.push_back(snir);
    it->second.delayList.push_back(delay);
    while (it->second.powerList.size() > 10)
        it->second.powerList.pop_front();
    while (it->second.snirList.size() > 10)
        it->second.snirList.pop_front();
    while (it->second.delayList.size() > 10)
        it->second.delayList.pop_front();

    if (measureEtx) {
        it->second.helloTime.push_back(simTime());
        // delete old entries.
        while (!it->second.helloTime.empty() && (simTime() - it->second.helloTime.front()) > numHellosEtx * minHelloInterval)
            it->second.helloTime.pop_front();
    }


    if (!it->second.snirList.empty()) {
        double val = 0;
        for (auto e : it->second.snirList)
            val += e;
        val /= it->second.snirList.size();
        it->second.snirCost = val;
    }

    if (!it->second.powerList.empty())
        it->second.powerRecCost = it->second.powerList.back();

    if (!it->second.delayList.empty()) {
        simtime_t val;
        for (auto e : it->second.delayList)
            val += e;
        val /= it->second.delayList.size();
        it->second.delayCost = val.dbl();
    }


    int metricType = -1;
    unsigned int metric = 255;
    unsigned int metricNeg = 255;
    auto metricPos = helloMessage->getTlvOptions().findByType(METRIC);

    if (metricPos != -1) {
        auto metricTlv = check_and_cast<const LoadNgMetricOption *>(helloMessage->getTlvOptions().getTlvOption(metricPos));
        if (metricTlv->getExtensionFlag())
            metricType = metricTlv->getExtension();
        if (metricTlv->getValueFlag())
            metric = metricTlv->getValue();
        if (metricType == HOPCOUNT) {
            if (metric < 255)
                metric++;
            metricNeg = 1;
        }
        else {
            // unknow metricToNeig set to hopcount
            metric = 255;
        }
    }

    std::map<L3Address, NodeStatus> nodeStatus;
    it->second.listNeigbours.clear();
    for (unsigned int k = 0 ; k < helloMessage->getNeighAddrsArraySize(); k++) {
        auto nData = helloMessage->getNeighAddrs(k);
        NodeStatus status;

        auto itSeq = seqNumbers.find(nData.getAddr());
        if (itSeq != seqNumbers.end() && nData.getSeqNum() < itSeq->second)
            continue; // ignore it

        status.isBidirectional = nData.isBidir();
        status.pendingConfirmation = nData.getPendingConfirmation();
        status.seqNum = nData.getSeqNum();
        status.metric = nData.getMetric();
        status.numHelloRec = nData.getNumHelloRec();
        status.delay = nData.getDelay();
        status.energy = nData.getEnergy();
        status.recPower = nData.getRecPower();
        status.snir = nData.getSnir();

        if (itSeq == seqNumbers.end() || (itSeq != seqNumbers.end() && itSeq->second < nData.getSeqNum())) {
            seqNumbers[nData.getAddr()] = nData.getSeqNum();
        }

        nodeStatus[nData.getAddr()] = status;
        // check common neighbor
        auto itNeigAux = neighbors.find(nData.getAddr());
        if (itNeigAux != neighbors.end() && itNeigAux->second.seqNumber < nData.getSeqNum()) {
            itNeigAux->second.pendingConfirmation = true;
        }

        if (this->getSelfIPAddress() != nData.getAddr())
            it->second.listNeigbours[nData.getAddr()] = status;
    }

    // TODO: In addEdge replace 1.0 value for the appropriate value of the metrics.

    // First check nodes that are in the list, remove nodes that are not in the new list, and actualize the others
    // Check if the list of neighbors of this node has change

    auto itCheckBiDir = nodeStatus.find(this->getSelfIPAddress());
    if (itCheckBiDir == nodeStatus.end()) {
        if (it->second.isBidirectional ) {
            changeTimerNotification = true;
            it->second.isBidirectional = false;
            it->second.metricToNeig = 255;
        }
    }
    else {
        it->second.isBidirectional = true;
        // Compute ETX if is needed. HERE IT IS POSSIBLE TO COMPUTE THE METRICS
        it->second.numHelloRec = itCheckBiDir->second.numHelloRec;
        if (measureEtx) {
            if (itCheckBiDir->second.numHelloRec == 0)
                it->second.metricToNeig = 255;
            else {
                double val = 0;
                if (simTime() - it->second.creationTime >= numHellosEtx * minHelloInterval) {
                    val = (double)(itCheckBiDir->second.numHelloRec * it->second.helloTime.size());
                    val /= (double)(numHellosEtx*numHellosEtx);
                }
                else {
                    int hellos = (simTime() - it->second.creationTime)/minHelloInterval;
                    if (hellos != 0) {
                        val =  (double) (itCheckBiDir->second.numHelloRec * it->second.helloTime.size());
                        val /= (double) (hellos * hellos);
                    }
                    else
                        val = 0.01;
                }
                if (val > 1) val = 1;
                val = 1/val;
                if (val > 255)
                    it->second.metricToNeig = 255;
                else
                    it->second.metricToNeig = val;
            }
        }
            // now should actualize the routing table
        // bi-dir
        if (it->second.isBidirectional)
            it->second.pendingConfirmation = false;
    }


    if (it->second.isBidirectional && !it->second.pendingConfirmation) {
        // if the link is biDirectional, delete from blacklist
        auto blackListIt = blacklist.find(it->first);
        if (blackListIt != blacklist.end())
            blacklist.erase(blackListIt);
    }
    else {
        blacklist[it->first] = simTime() + blacklistTimeout;
    }

    // now include the information in dijkstra
    rebuildDijkstraWihtHellos();

    /*
    if (dijkstraKs && dijkstraKs->isRouteMapEmpty()) {
        runDijkstraKs();
    }

    */

    int numRoutes = routingTable->getNumRoutes();
    if (dijkstraTotal && (dijkstraTotal->getNumRoutes() < nodesData.size() - 1 || (int)dijkstraTotal->getNumRoutes() != numRoutes)) {
        emit(recomputeSignal, this);
    }

    // Whenever a node receives a Hello message from a neighbor, the node
    // SHOULD make sure that it has an active route to the neighbor, and
    // create one if necessary.  If a route already exists, then the
    // Lifetime for the route should be increased, if necessary, to be at
    // least ALLOWED_HELLO_LOSS * HELLO_INTERVAL.  The route to the
    // neighbor, if it exists, MUST subsequently contain the latest
    // Destination Sequence Number from the Hello message.  The current node
    // can now begin using this route to forward data packets.  Routes that
    // are created by hello messages and not used by any other active routes
    // will have empty precursor lists and would not trigger a RERR message
    // if the neighbor moves away and a neighbor timeout occurs.

    if (dijkstra) {
        // actualize routes
        Dijkstra::RoutesInfoList list;
        dijkstra->getRoutesInfoList(list);
        for (auto elem : list) {
            // add now
            IRoute *route = routingTable->findBestMatchingRoute(elem.destination);
            int64_t latestDestSeqNum = -1;
            auto it = neighbors.find(elem.nextHop);
            if (it == neighbors.end())
                throw cRuntimeError("neighbor not found in the list");

            if (elem.nextHop != elem.destination) {
                if (elem.hops == 2) {
                    auto it2 = it->second.listNeigbours.find(elem.destination);
                    if (it2 == it->second.listNeigbours.end())
                        throw cRuntimeError("neighbor not found in the list");
                    latestDestSeqNum = it2->second.seqNum;
                }
            }
            else
                latestDestSeqNum = it->second.seqNumber;
            simtime_t newLifeTime;
            if (elem.nextHop == helloOriginatorAddr) {
                newLifeTime = helloMessage->getLifetime() + par("allowedHelloLoss") * minHelloInterval;
            }

            if (!route || route->getSource() != this) {
                route = createRoute(elem.destination, elem.nextHop, elem.hops, latestDestSeqNum, true, newLifeTime, metricType, elem.cost );
                if (elem.hops == 1) {
                    LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
                    routeData->setIsBidirectiona(true);
                }
            }
            else {
                LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(route->getProtocolData());
                simtime_t lifeTime = routeData->getLifeTime();
                if (elem.hops == 1)
                    routeData->setIsBidirectiona(true);
                updateRoutingTable(route, elem.nextHop, elem.hops, latestDestSeqNum, true, std::max(lifeTime, newLifeTime), metricType, elem.cost);
            }
        }
    }
    else {
        if (it->second.pendingConfirmation == false && it->second.isBidirectional) {
            IRoute *routeHelloOriginator = routingTable->findBestMatchingRoute(helloOriginatorAddr);
            unsigned int latestDestSeqNum = helloMessage->getSeqNum();
            simtime_t newLifeTime = helloMessage->getLifetime() + par("allowedHelloLoss") * minHelloInterval;
            if (!routeHelloOriginator || routeHelloOriginator->getSource() != this) {
                routeHelloOriginator = createRoute(helloOriginatorAddr, helloOriginatorAddr, 1, latestDestSeqNum, true, newLifeTime, metricType, metricNeg );
                LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(routeHelloOriginator->getProtocolData());
                routeData->setIsBidirectiona(true);
            }
            else {
                LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(routeHelloOriginator->getProtocolData());
                simtime_t lifeTime = routeData->getLifeTime();
                routeData->setIsBidirectiona(true);
                updateRoutingTable(routeHelloOriginator, helloOriginatorAddr, 1, latestDestSeqNum, true, std::max(lifeTime, newLifeTime), metricType, metricNeg);
            }
        }
    }

    /*
    LoadNgRouteData *routeData = check_and_cast<LoadNgRouteData *>(routeHelloOriginator->getProtocolData());
    routeData->setPower(rreq->getPower());
    routeData->setDelay(rreq->getDelay());
    routeData->setSnir(rreq->getSnir());
    routeData->setEtx(rreq->getEtx());
    */

    changeTimerNotification = true;

    // add the information of the hello to the routing table
    if (changeTimerNotification) {
        helloInterval = minHelloInterval;
        if (!helloMsgTimer->isScheduled()) {
            // schedule immediately
            scheduleAt(simTime(), helloMsgTimer);
        }
        else {
            simtime_t schduled = helloMsgTimer->getSendingTime();
            simtime_t arrival = helloMsgTimer->getArrivalTime();
            simtime_t interval = arrival - schduled;
            if (interval > minHelloInterval) {
                // Schedule immediately
                cancelEvent(helloMsgTimer);
                scheduleAt(simTime(), helloMsgTimer);
            }
        }
    }
    else {
        helloInterval += 2.0;
        if (helloInterval > maxHelloInterval) {
            helloInterval = maxHelloInterval;
        }
    }
}

void LoadNg::expungeRoutes()
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

void LoadNg::scheduleExpungeRoutes()
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

INetfilter::IHook::Result LoadNg::datagramForwardHook(Packet *datagram)
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

void LoadNg::sendRERRWhenNoRouteToForward(const L3Address& unreachableAddr, const L3Address &destAddr)
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
    sendLoadNgPacket(rerr, nextAddress, ttl, *jitterPar);    // TODO: unicast if there exists a route to the source
}

void LoadNg::cancelRouteDiscovery(const L3Address& destAddr)
{
    ASSERT(hasOngoingRouteDiscovery(destAddr));
    auto lt = targetAddressToDelayedPackets.lower_bound(destAddr);
    auto ut = targetAddressToDelayedPackets.upper_bound(destAddr);
    for (auto it = lt; it != ut; it++)
        networkProtocol->dropQueuedDatagram(it->second);

    targetAddressToDelayedPackets.erase(lt, ut);

    auto waitRREPIter = waitForRREPTimers.find(destAddr);
    ASSERT(waitRREPIter != waitForRREPTimers.end());
    cancelAndDelete(waitRREPIter->second);
    waitForRREPTimers.erase(waitRREPIter);
}

bool LoadNg::updateValidRouteLifeTime(const L3Address& destAddr, simtime_t lifetime)
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

const Ptr<RrepAck> LoadNg::createRREPACK()
{
    auto rrepACK = makeShared<RrepAck>(); // TODO: "AODV-RREPACK");
    rrepACK->setPacketType(RREPACK);
    rrepACK->setChunkLength(B(2));
    return rrepACK;
}

void LoadNg::sendRREPACK(const Ptr<RrepAck>& rrepACK, const L3Address& destAddr)
{
    EV_INFO << "Sending Route Reply ACK to " << destAddr << endl;
    sendLoadNgPacket(rrepACK, destAddr, 100, 0);
}

void LoadNg::handleRREPACK(const Ptr<const RrepAck>& rrepACK, const L3Address& neighborAddr)
{
    // Note that the RREP-ACK packet does not contain any information about
    // which RREP it is acknowledging.  The time at which the RREP-ACK is
    // received will likely come just after the time when the RREP was sent
    // with the 'A' bit.
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
}

void LoadNg::handleRREPACKTimer()
{
    // when a node detects that its transmission of a RREP message has failed,
    // it remembers the next-hop of the failed RREP in a "blacklist" set.

    EV_INFO << "RREP-ACK didn't arrived within timeout. Adding " << failedNextHop << " to the blacklist" << endl;

    blacklist[failedNextHop] = simTime() + blacklistTimeout;    // lifetime

    if (!blacklistTimer->isScheduled())
        scheduleAt(simTime() + blacklistTimeout, blacklistTimer);
}

void LoadNg::handleBlackListTimer()
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

LoadNg::~LoadNg()
{

    for (auto & e : pendingSend) {
        cancelAndDelete(e);
    }
    cancelAndDelete(recomputeTimer);
    pendingSend.clear();

    neighbors.clear();
    if (dijkstra)
        delete dijkstra;
    if (dijkstraKs)
        delete dijkstraKs;
    clearState();
    delete helloMsgTimer;
    delete expungeTimer;
    delete counterTimer;
    delete rrepAckTimer;
    delete blacklistTimer;
}

} // namespace aodv
} // namespace inet

