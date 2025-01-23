#include "PARRoT_map.h"
#include <random>
#include <fstream>

namespace inet {

#define LOS_ 0
#define NLOS_ 1
#define LOSMAP_ 2

void PARRoT_map::handleDataFromUpperLayer(Packet *packet)
{
    auto dh = makeShared<PDataHeader>();
    auto l3AddressReq = packet->removeTag<L3AddressReq>();
    Ipv4Address dest = l3AddressReq->getDestAddress().toIpv4();

    dh->setSrcAddr(m_selfIpv4Address);
    dh->setDestAddr(dest);
    dh->setSeqNum(m_dataSeqNum);
    m_dataSeqNum++;
    dh->setTtl(maxHops);
    int totalByteLength = sizeof(m_selfIpv4Address)*2 + sizeof((int)maxHops)*3;
    dh->setChunkLength(B(totalByteLength));

    packet->insertAtFront(dh);
    forwardData(packet, dest);
}


void PARRoT_map::handleDataFromLowerLayer(Packet *packet)
{
    auto dh = packet->peekAtFront<PDataHeader>();
    // forward
    Ipv4Address src = dh->getSourceAddress().toIpv4();
    Ipv4Address dest = dh->getDestinationAddress().toIpv4();
    int seqNum = dh->getSeqNum();
    int ttl = dh->getTtl();

    std::pair<Ipv4Address, int> targetPair(src, seqNum);
    if(duplicate_data_pkt.find(targetPair) != duplicate_data_pkt.end())
    {
        return;
    }
    duplicate_data_pkt.insert({src, seqNum});

    if(dest == m_selfIpv4Address)
    {
        dh = packet->popAtFront<PDataHeader>();
        packet->addTagIfAbsent<NetworkProtocolInd>()->setProtocol(&getProtocol());
        packet->addTagIfAbsent<NetworkProtocolInd>()->setNetworkProtocolHeader(dh);
        packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::udp);
        packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::udp);
        packet->addTagIfAbsent<HopLimitInd>()->setHopLimit(maxHops);
        auto l3AddressInd = packet->addTagIfAbsent<L3AddressInd>();
        l3AddressInd->setSrcAddress(dh->getSourceAddress());
        l3AddressInd->setDestAddress(dh->getDestinationAddress());
        sendUp(packet);

        int hop = maxHops - ttl;
        emit(destDataHopCountSignal, hop);
    }
    else if(ttl > 0)      // next == me! forward
    {
        forwardData(packet, dest);
    }
    else
    {
        EV_INFO << "NOT addressed to me! abort & drop" << std::endl;
    }
}

void PARRoT_map::forwardData(Packet* packet, Ipv4Address dest)
{
    Ipv4Address bestGateway = findBestNextHop(dest);
    Ipv4Address robustGateway = findRobustNextHop(dest);

    if(bestGateway == Ipv4Address("0.0.0.0"))
    {
        PacketDropDetails details;
        details.setReason(NO_ROUTE_FOUND);
        emit(packetDroppedSignal, packet, &details);
        delete packet;
        return;
    }

    auto dh = packet->removeAtFront<PDataHeader>();
    dh->setTtl(dh->getTtl()-1);
    packet->insertAtFront(dh);
    MacAddress nextHopMacAddr = arp->resolveL3Address(bestGateway, nullptr);
    if(robustGateway != Ipv4Address("0.0.0.0"))
    {
        MacAddress robustMacAddr = arp->resolveL3Address(robustGateway, nullptr);
        packet->addTagIfAbsent<RobustForwarderTag>()->setAddr(robustMacAddr);
    }

    setDownControlInfo(packet, nextHopMacAddr);
    sendDown(packet);
}

double PARRoT_map::qFunction_map(Ipv4Address target, Ipv4Address hop) {
    double gamma_future_map = Vi.at(hop)->Gamma_Future_map();
    return Gateways.at(target).at(hop)->Vmap() * gamma_future_map;
}

double PARRoT_map::qFunction_c(Ipv4Address target, Ipv4Address hop) {
    double gamma_future_c = Vi.at(hop)->Gamma_Future_c();
    return Gateways.at(target).at(hop)->Vc() * gamma_future_c;
}

double PARRoT_map::getMaxValueFor_map(Ipv4Address target) {

    double res = -1000;
    if(Gateways.find(target) != Gateways.end()){
    for (std::map<Ipv4Address, PCE_map*>::iterator act =
            Gateways.find(target)->second.begin();
            act != Gateways.find(target)->second.end(); act++) {
        double deltaT = simTime().dbl() - act->second->lastSeen();
        if (deltaT  <= std::max(neighborReliabilityTimeout, mhChirpInterval)) {
            if (act == Gateways.find(target)->second.begin()) {
                // First possible action, make sure the result gets this value anyway
                res = qFunction_map(target, act->first);
            }
            res = std::max(res, qFunction_map(target, act->first));

        }
    }
    }
    return res;
}

double PARRoT_map::getMaxValueFor_c(Ipv4Address target) {

    double res = -1000;
    if(Gateways.find(target) != Gateways.end()){
    for (std::map<Ipv4Address, PCE_map*>::iterator act =
            Gateways.find(target)->second.begin();
            act != Gateways.find(target)->second.end(); act++) {
        double deltaT = simTime().dbl() - act->second->lastSeen();
        if (deltaT  <= std::max(neighborReliabilityTimeout, mhChirpInterval)) {
            if (act == Gateways.find(target)->second.begin()) {
                // First possible action, make sure the result gets this value anyway
                res = qFunction_c(target, act->first);
            }
            res = std::max(res, qFunction_c(target, act->first));

        }
    }
    }
    return res;
}


Ipv4Address PARRoT_map::findBestNextHop(Ipv4Address target) {
    Ipv4Address a = Ipv4Address("0.0.0.0");
    double res = -1000;
    std::vector<Ipv4Address> toDelete;
    if(Gateways.find(target) != Gateways.end()){
        for (std::map<Ipv4Address, PCE_map*>::iterator act =
                Gateways.find(target)->second.begin();
                act != Gateways.find(target)->second.end(); act++) {
            double deltaT = simTime().dbl() - act->second->lastSeen();
            if (deltaT  <= std::max(neighborReliabilityTimeout, mhChirpInterval)) {
                if (act == Gateways.find(target)->second.begin()) {
    //               First possible action, make sure the result gets this value anyway
                    res = qFunction_map(target, act->first);
                    a = act->first;
                }
                else if (qFunction_map(target, act->first) > res) { //act->second->Q() > res) {
                    res = qFunction_map(target, act->first); //act->second->Q();
                    a = act->first;
                }
            }
            else {
                toDelete.push_back(act->first);
    //          delete act->second;
    //          Gateways.at(target).erase(act);
            }
        }

        for (const auto& addr : toDelete) {
            std::map<Ipv4Address, PCE_map*> interNodes = Gateways.find(target)->second;
            auto it = interNodes.find(addr);
            if (it != interNodes.end()) {
    //                delete static_cast<PCE*>(it->second);  // 释放 PCE 对象的内存
                interNodes.erase(it);  // 从 map 中删除元素
            }
        }
    }

    return a;
}

Ipv4Address PARRoT_map::findRobustNextHop(Ipv4Address target) {
    Ipv4Address a = Ipv4Address("0.0.0.0");
    double res = -1000;
    std::vector<Ipv4Address> toDelete;
    if(Gateways.find(target) != Gateways.end()){
        for (std::map<Ipv4Address, PCE_map*>::iterator act =
                Gateways.find(target)->second.begin();
                act != Gateways.find(target)->second.end(); act++) {
            double deltaT = simTime().dbl() - act->second->lastSeen();
            if (deltaT  <= std::max(neighborReliabilityTimeout, mhChirpInterval)) {
                if (act == Gateways.find(target)->second.begin()) {
    //               First possible action, make sure the result gets this value anyway
                    res = qFunction_c(target, act->first);
                    a = act->first;
                }
                else if (qFunction_c(target, act->first) > res) { //act->second->Q() > res) {
                    res = qFunction_c(target, act->first); //act->second->Q();
                    a = act->first;
                }
            }
            else {
                toDelete.push_back(act->first);
    //          delete act->second;
    //          Gateways.at(target).erase(act);
            }
        }

        for (const auto& addr : toDelete) {
            std::map<Ipv4Address, PCE_map*> interNodes = Gateways.find(target)->second;
            auto it = interNodes.find(addr);
            if (it != interNodes.end()) {
    //                delete static_cast<PCE*>(it->second);  // 释放 PCE 对象的内存
                interNodes.erase(it);  // 从 map 中删除元素
            }
        }
    }

    return a;
}

//double PARRoT_map::Cal_Gamma_Future_map(Ipv4Address neighbor) {
//    ExtendedBonnMotionMobility *neighbor_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[neighbor]);
//    ExtendedBonnMotionMobility *self_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[m_selfIpv4Address]);
//    double period = 0.1;
//    int Np = ceil(std::max(neighborReliabilityTimeout, mhChirpInterval)/period);
//    double link_expire_time = 0;
//    for(int i=0; i<Np; i++)
//    {
//        double lookAhead = i * period;
//        Coord P1 = neighbor_mob->getFuturePosition(lookAhead, simTime());
//        Coord P2 = self_mob->getFuturePosition(lookAhead, simTime());
//        double distance = P1.distance(P2);
//        if(LETRangeMode == LOS_)
//        {
//            if(distance >= losRange) break;
//            link_expire_time += period;
//        }
//        else if(LETRangeMode == NLOS_)
//        {
//            if(distance >= nlosRange) break;
//            link_expire_time += period;
//        }
//        else if(LETRangeMode == LOSMAP_)
//        {
//            if(pathLoss->checkNlos(P1, P2))
//            {
//                if(distance >= nlosRange) break;
//                link_expire_time += period;
//            }
//            else
//            {
//                if(distance >= losRange) break;
//                link_expire_time += period;
//            }
//        }
//        else
//            throw cRuntimeError("LETRangeMode not supported");
//    }
//
//    return link_expire_time/std::max(neighborReliabilityTimeout, mhChirpInterval);
//}
//
//double PARRoT_map::Cal_Gamma_Future_c(Ipv4Address neighbor) {
//    ExtendedBonnMotionMobility *neighbor_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[neighbor]);
//    ExtendedBonnMotionMobility *self_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[m_selfIpv4Address]);
//    double period = 0.1;
//    int Np = ceil(std::max(neighborReliabilityTimeout, mhChirpInterval)/period);
//    double link_expire_time = 0;
//    for(int i=0; i<Np; i++)
//    {
//        double lookAhead = i * period;
//        Coord P1 = neighbor_mob->getFuturePosition(lookAhead, simTime());
//        Coord P2 = self_mob->getFuturePosition(lookAhead, simTime());
//        double distance = P1.distance(P2);
//        if(distance >= nlosRange) break;
//        link_expire_time += period;
//    }
//
//    return link_expire_time/std::max(neighborReliabilityTimeout, mhChirpInterval);
//}

double PARRoT_map::Cal_Gamma_Future_map(Ipv4Address neighbor) {
    ExtendedBonnMotionMobility *neighbor_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[neighbor]);
    ExtendedBonnMotionMobility *self_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[m_selfIpv4Address]);
    double period = 0.1;
    int Np = ceil(std::max(neighborReliabilityTimeout, mhChirpInterval)/period);
    double phi_total = 0;
    for(int i=0; i<Np; i++)
    {
        double lookAhead = i * period;
        Coord P1 = neighbor_mob->getFuturePosition(lookAhead, simTime());
        Coord P2 = self_mob->getFuturePosition(lookAhead, simTime());
        double distance = P1.distance(P2);
        double P_outage = 1;
        double etx = 0;
        double phi_cur = 0;
        if(LETRangeMode == LOS_)
        {
            P_outage = rician_outage_vector[ceil(distance)];
        }
        else if(LETRangeMode == NLOS_)
        {
            P_outage = rayleigh_outage_vector[ceil(distance)];
        }
        else if(LETRangeMode == LOSMAP_)
        {
            if(pathLoss->checkNlos(P1, P2))
            {
                P_outage = rayleigh_outage_vector[ceil(distance)];
            }
            else
            {
                P_outage = rician_outage_vector[ceil(distance)];
            }
        }
        else
            throw cRuntimeError("LETRangeMode not supported");

        etx = 1/(1-P_outage);
        phi_cur = std::pow(discount, etx);
        phi_total += phi_cur;
    }

    return phi_total/Np;
}

double PARRoT_map::Cal_Gamma_Future_c(Ipv4Address neighbor) {
    ExtendedBonnMotionMobility *neighbor_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[neighbor]);
    ExtendedBonnMotionMobility *self_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[m_selfIpv4Address]);
    double period = 0.1;
    int Np = ceil(std::max(neighborReliabilityTimeout, mhChirpInterval)/period);
    double phi_total = 0;
    for(int i=0; i<Np; i++)
    {
        double lookAhead = i * period;
        Coord P1 = neighbor_mob->getFuturePosition(lookAhead, simTime());
        Coord P2 = self_mob->getFuturePosition(lookAhead, simTime());
        double distance = P1.distance(P2);
        double P_outage = 1;
        double etx = 0;
        double phi_cur = 0;

        P_outage = rayleigh_outage_vector[ceil(distance)];
        etx = 1/(1-P_outage);
        phi_cur = std::pow(discount, etx);
        phi_total += phi_cur;
    }

    return phi_total/Np;
}


}
