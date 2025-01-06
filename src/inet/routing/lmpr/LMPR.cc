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

#include "LMPR.h"

namespace inet {

Define_Module(LMPR);

simsignal_t nodeInfoLenSignal = cComponent::registerSignal("nodeInfoLen");
simsignal_t LETSignal = cComponent::registerSignal("LET");
simsignal_t nextNodeChoiceLETSignal = cComponent::registerSignal("nextNodeChoiceLET");

LMPR::LMPR() {
    // TODO Auto-generated constructor stub

}

LMPR::~LMPR() {
    // TODO Auto-generated destructor stub
}

void LMPR::initialize(int stage) {
    NetworkProtocolBase::initialize(stage);

    if(stage == INITSTAGE_LOCAL) {
        m_seqNum = 0;
        m_OGM_seqNum = 0;

        // Routing
        mhOGMInterval = par("mhOGMInterval");
        neighborReliabilityTimeout = par("neighborReliabilityTimeout");
//        txRange = par("txRange");
        predictDuration = par("predictDuration");
        OGMReminder = new cMessage("OGMReminder");
        bcMaxEntries = par("bcMaxEntries");
        bcDelTime = par("bcDelTime");
        defaultTTL = par("defaultTTL");
        setAutoRange = par("setAutoRange");
        losMapError = par("losMapError");
        maxRangeForLET = par("maxRangeForLET");
        setAutoLETRange = par("setAutoLETRange");
    }
    else if(stage == INITSTAGE_NETWORK_LAYER) {
        ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        mobility = check_and_cast<IMobility*>(getContainingNode(this)->getSubmodule("mobility"));
        radioMedium = check_and_cast<physicallayer::IRadioMedium*>(getModuleByPath("Scenario.radioMedium"));
        arp = check_and_cast<GlobalArp*>(this->getParentModule()->getSubmodule("arp"));
//        arp = getModuleFromPar<GlobalArp>(par("arpModule"), this->getParentModule());
        cModule* topModule = getModuleByPath("Scenario");
        int numHosts = topModule->par("numHosts");
        int numRouters = topModule->par("numRouters");
        char mob_path_str[100];
        char ip_path_str[100];
        for(int i = 0; i < numHosts; i ++)
        {
            sprintf(mob_path_str, "Scenario.host[%d].mobility", i);
            sprintf(ip_path_str, "Scenario.host[%d].wlan[0]", i);
            ExtendedBonnMotionMobility* mob = check_and_cast<ExtendedBonnMotionMobility*>(
                    getModuleByPath(mob_path_str));
            NetworkInterface *ie = check_and_cast<NetworkInterface*>(
                    getModuleByPath(ip_path_str));
            Ipv4Address addr = ie->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
            _globalMob.push_back({addr, mob});
        }
        for (int i = 0; i < numRouters; i ++)
        {
            sprintf(mob_path_str, "Scenario.router[%d].mobility", i);
            sprintf(ip_path_str, "Scenario.router[%d].wlan[0]", i);
            ExtendedBonnMotionMobility* mob = check_and_cast<ExtendedBonnMotionMobility*>(
                    getModuleByPath(mob_path_str));
            NetworkInterface *ie = check_and_cast<NetworkInterface*>(
                    getModuleByPath(ip_path_str));
            Ipv4Address addr = ie->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
            _globalMob.push_back({addr, mob});
        }

        N = numHosts + numRouters;
        adjacencyMatrix.resize(N);
        for (int i = 0; i < N; i ++)
        {
            adjacencyMatrix[i].resize(N, 0.0);
        }

        pathLoss = check_and_cast<physicallayer::FactoryFading*>(getModuleByPath("Scenario.radioMedium.pathLoss"));

        start();
    }

    else
    {
        if(stage != 1)
            EV << stage << std::endl;
    }

}

void LMPR::start(){
    /* Search the 80211 interface */
    int num_80211 = 0;
    broadcastDelay = &par("broadcastDelay");
    NetworkInterface *ie;
    NetworkInterface *i_face;
    const char *name;
    for (int i = 0; i < ift->getNumInterfaces(); i++) {
        ie = ift->getInterface(i);
        name = ie->getInterfaceName();
        if (strstr(name, "wlan") != nullptr) {
            i_face = ie;
            num_80211++;
            interfaceId = i;
        }
    }

// One enabled network interface (in total)
    if (num_80211 == 1) {
        interface80211ptr = i_face;
    }
    else {
        throw cRuntimeError("Router has found %i 80211 interfaces", num_80211);
    }

    m_selfIpv4Address = interface80211ptr->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
    for(int i = 0; i < N; i ++)
    {
        if(_globalMob[i].first == m_selfIpv4Address)
        {
            m_selfNodeIdx = i;
            break;
        }
    }
//

//    scheduleAt(simTime() + uniform(0.0, par("maxJitter")), OGMReminder);
    scheduleAt(simTime() + uniform(0.0, par("mhOGMInterval")), OGMReminder);
}

void LMPR::stop() {

}
void LMPR::finish() {
//    storeMapDataToCSV();
}

void LMPR::handleSelfMessage(cMessage *msg) {
    if (msg == OGMReminder) {
        handleOGMReminder();
    }
}

/** @brief Handle messages from upper layer */
void LMPR::handleUpperPacket(Packet *packet)
{
    handleDataFromUpperLayer(packet);
}

/** @brief Handle messages from lower layer */
void LMPR::handleLowerPacket(Packet *packet)
{
//    addMapData(packet->dup());
//    if(strcmp(packet->getName(), "OGM") == 0)
    if(strstr(packet->getName(), "OGM") != nullptr)
    {
        auto incomingOGM = (staticPtrCast<OGM>(packet->peekData<OGM>()->dupShared()));
        handleOGM(incomingOGM, packet->getBitLength());
        delete packet;
    }
    else
    {
        auto packetCopy = packet->dup();
        handleDataFromLowerLayer(packetCopy);
        delete packet;
    }
}

//        auto tmp = this->getParentModule()->getParentModule()->getSubmodule("radioMedium");
//        if (tmp) {
//            radioMedium = check_and_cast<physicallayer::IRadioMedium*>(tmp);
//        }else{
//            throw;
//        }
//        arp.reference(this, "arpModule", true);

//    else if(stage == INITSTAGE_ROUTING_PROTOCOLS) {
//        registerProtocol(Protocol::manet, gate("ipOut"), gate("ipIn"));
//    }

//        rt = getModuleFromPar<IIpv4RoutingTable>(par("routingTableModule"),
//                this);
//        networkProtocol = getModuleFromPar<INetfilter>(
//                par("networkProtocolModule"), this);


//    else if (stage == INITSTAGE_NETWORK_INTERFACE_CONFIGURATION) {
//            for (int i = 0; i < interfaceTable->getNumInterfaces(); i++)
//                interfaceTable->getInterface(i)->setHasModulePathAddress(true);
//        }
//        else if (stage == INITSTAGE_NETWORK_LAYER) {
//            if (auto ie = interfaceTable->findFirstNonLoopbackInterface())
//                myNetwAddr = ie->getNetworkAddress();
//            else
//                throw cRuntimeError("No non-loopback interface found!");
//        }

//    CHK(interface80211ptr->getProtocolDataForUpdate< Ipv4InterfaceData >())->joinMulticastGroup(
//            Ipv4Address::LL_MANET_ROUTERS);

//    ieee80211::Ieee80211Mac *mac_ptr = nullptr;
//    for (SubmoduleIterator it(interface80211ptr); !it.end(); ++it) {
//        cModule *submodule = *it;
//        if (dynamic_cast<ieee80211::Ieee80211Mac*>(submodule)) {
//            mac_ptr = dynamic_cast<ieee80211::Ieee80211Mac*>(submodule);
//        }
//    }
//
//    ieee80211::Dcf *dcf_ptr = nullptr;
//    if (mac_ptr) {
//        for (SubmoduleIterator it(mac_ptr); !it.end(); ++it) {
//            cModule *submodule = *it;
//            if (dynamic_cast<ieee80211::Dcf*>(submodule)) {
//                dcf_ptr = dynamic_cast<ieee80211::Dcf*>(submodule);
//            }
//        }
//    }
//    if (dcf_ptr) {
//        for (SubmoduleIterator it(dcf_ptr); !it.end(); ++it) {
//            cModule *submodule = *it;
//            if (dynamic_cast<ieee80211::Dcaf*>(submodule)) {
//                dcaf_ptr = dynamic_cast<ieee80211::Dcaf*>(submodule);
//            }
//        }
//    }

//    // Populate the adjacency matrix
//    for (int i = 0; i < N; ++i) {
//        for (int j = 0; j < N; ++j) {
//            if (i != j) {  // No self-loops
//                Coord PosA = _globalMob[i].second->getCurrentPosition();
//                Coord PosB = _globalMob[j].second->getCurrentPosition();
//                double distance = PosA.distance(PosB);
//                double commRange = getRealCommRange(PosA, PosB);
//                if (distance <= commRange) {
//                    adjacencyMatrix[i][j] = 1.0; // Nodes i and j are within communication range
//                } else {
//                    adjacencyMatrix[i][j] = 0.0;
//                }
//            }
//        }
//    }
//
//    int nextNodeIdx = graphT::dijkstra(adjacencyMatrix, m_selfNodeIdx, getTargetNodeIdxByAddr(dest));
//
////    Coord PosA = _globalMob[m_selfNodeIdx].second->getCurrentPosition();
////    Coord PosB = _globalMob[nextNodeIdx].second->getCurrentPosition();
////    bool nlos = pathLoss->checkNlos(PosA, PosB);
////    double dist = PosA.distance(PosB);
////    bool is_connected = dist < getRealCommRange(PosA, PosB);
//
//    if(nextNodeIdx != -1)
//    {
//        Ipv4Address nextHopAddr = _globalMob[nextNodeIdx].first;
//        MacAddress nextHopMacAddr = arp->resolveL3Address(nextHopAddr, nullptr);
//        setDownControlInfo(packet, nextHopMacAddr);
//        sendDown(packet);
//    }
//    else
//    {
//        // drop packet
//        delete packet;
//    }


//const NetworkInterface *
//LMPR::getDestInterface(Packet *packet)
//{
//    const auto& tag = packet->findTag<InterfaceReq>();
//    return tag != nullptr ? ift->getInterfaceById(tag->getInterfaceId()) : nullptr;
//}

//void LMPR::handleOGM(OGM *ogm, int64_t len)
//{
    //    Ipv4Address origin = ogm->origin;
    //    Ipv4Address forwarder = ogm->forwarder;
    //    double forwarder_coord_x = ogm->getX();
    //    double forwarder_coord_y = ogm->getY();
    //    double forwarder_vx = ogm->getVx();
    //    double forwarder_vy = ogm->getVy();
    //
    //    bool forwarding = false;
    //
    //    // estimate LET for forwarding
    //    double link_expir_time = calculateLinkExpirationTime(forwarder_coord_x, forwarder_coord_y, forwarder_vx, forwarder_vy);
    //
    //    if(origin == forwarder)  // one hop neighbor
    //    {
    //        // LET
    //        // add
    //    }
    //    else    // multi hop
    //    {
    //
    //    }
//}


//void LMPR::sendOGM()
//{
//    auto ogm = makeShared<OGM>();
//
//    // Set identification fields
//    ogm->setOrigin(m_selfIpv4Address);
//    ogm->setSeq(m_squNr);
//    ogm->setForwarder(m_selfIpv4Address);
//    m_squNr++;
//
//    Coord p = check_and_cast<ExtendedBonnMotionMobility*>(mobility)->getCurrentRealPosition();
//    double Np = 1;
//    Coord forecast = forecastPosition(Np*predictDuration);
//    Coord v_int = (forecast - p)/(Np*predictDuration);
//
//    ogm->setX((float)p.getX());
//    ogm->setY((float)p.getY());
//    ogm->setVX((float)v_int.getX());
//    ogm->setVY((float)v_int.getY());
//
//    ogm->setScore(1);
//    ogm->setTTL(32);
//
//    int totalByteLength = sizeof(m_selfIpv4Address)*2
//            + sizeof(m_squNr)
//            + 6 * sizeof((float)p.getX());
//    ogm->setChunkLength(B(totalByteLength));
//
//    // Creating and sending packet
//    auto packet = new Packet("OGM", ogm);
//    packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(
//            Ipv4Address(255, 255, 255, 255));
//    packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(m_selfIpv4Address);
//    packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(
//            interface80211ptr->getInterfaceId());
//    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::manet);
//    packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);
//    sendDelayed(packet, broadcastDelay->doubleValue(), "ipOut");
//    packet = nullptr;
//    ogm = nullptr;
//}

//double LMPR::calculateLinkExpirationTime(float coord_x, float coord_y, float vx, float vy)
//{
//    double LET = 0;
//    int Np = ceil(neighborReliabilityTimeout/predictDuration);
//    for(int i = 0; i < Np; i++)
//    {
//        Coord forecast_self = forecastPosition(Np*predictDuration);
//        Coord forecast_other;
//        forecast_other.x = coord_x + vx*Np*predictDuration;
//        forecast_other.y = coord_y + vy*Np*predictDuration;
//        if(forecast_self.distance(forecast_other) <= txRange)
//        {
//            LET += predictDuration;
//        }
//        else
//        {
//            return LET;
//        }
//    }
//    return LET;
//
//}




} /* namespace inet */
