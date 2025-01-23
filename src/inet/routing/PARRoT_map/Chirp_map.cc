#include "PARRoT_map.h"
#include <fstream>
namespace inet {

void PARRoT_map::recv_parrot_map(cMessage *msg) {

    auto incomingMultiHop = (staticPtrCast<MultiHopChirp_map>(check_and_cast<Packet*>(msg)->peekData<MultiHopChirp_map>()->dupShared()));
    // todo: Only Chunk length (->getBitLength()) or whole packet? (->getTotalLength().get())
    int remainingHops = handleIncomingMultiHopChirp( &(*incomingMultiHop), check_and_cast<Packet*>(msg)->getBitLength());

    if (remainingHops > 0 && postliminaryChecksPassed(incomingMultiHop->getOrigin(), incomingMultiHop->getHop())) {
        incomingMultiHop->setOmega_map((float)getMaxValueFor_map(incomingMultiHop->getOrigin()));
        incomingMultiHop->setOmega_c((float)getMaxValueFor_c(incomingMultiHop->getOrigin()));

        // Set neighbor fallback information
        incomingMultiHop->setHop(m_selfIpv4Address);

//        Coord forecast = forecastPosition();
//        Coord p = (hist_coord.size() != 0) ? hist_coord[historySize - 1] : Coord::NIL;
//        incomingMultiHop->setX((float)p.getX());
//        incomingMultiHop->setY((float)p.getY());
//        incomingMultiHop->setZ((float)p.getZ());
//
//        Coord v_int = (forecast - p)/((neighborReliabilityTimeout != 0) ? neighborReliabilityTimeout : 1.0);
//        incomingMultiHop->setVX((float)v_int.getX());
//        incomingMultiHop->setVY((float)v_int.getY());
//        incomingMultiHop->setVZ((float)v_int.getZ());

        // Update hop count
        incomingMultiHop->setHopCount(remainingHops);

        auto packet = new Packet("multiHopChirpMap", incomingMultiHop);
        setDownControlInfo(packet, MacAddress::BROADCAST_ADDRESS);
        sendDown(packet);
//        packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(
//                Ipv4Address(255, 255, 255, 255)); // let's try the limited broadcast 255.255.255.255 but
//                                                  // multicast goes from 224.0.0.0 to 239.255.255.255
//        packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(
//                m_selfIpv4Address); // let's try the limited broadcast
//        packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(
//                interface80211ptr->getInterfaceId());
//        packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(
//                &Protocol::manet);
        packet = nullptr;
    }
    delete msg;
}

bool PARRoT_map::postliminaryChecksPassed(Ipv4Address origin, Ipv4Address gateway) {
    if (Vi.size() == 0) {
        // No neighbors
        return false;
    }
    else if (Vi.size() == 1) {
        // One neighbor, was this neighbor the sender/origin of the chirp?
        return !(Vi.begin()->first == origin || Vi.begin()->first == gateway);
    }
    else if (findRobustNextHop(origin) != gateway) {
        // Neighbor was not the best choice, no need to propagate this.
        return false;
    }
    else {
        return true;
    }
}

int PARRoT_map::handleIncomingMultiHopChirp(MultiHopChirp_map *chirp, int64_t len) {
    int node_id = getContainingNode(this)->getId();
    if(node_id == 4)
    {
        for (std::map<Ipv4Address, PDC_map*>::iterator n = Vi.begin(); n != Vi.end(); n++) {
            double gamma_map = n->second->Gamma_Future_map();
            double gamma_c = n->second->Gamma_Future_c();
            EV_WARN << "Neighbor: " << gamma_map << " " << gamma_c << std::endl;
        }
        std::map<Ipv4Address, std::map<Ipv4Address, PCE_map*>>::iterator gw = Gateways.find(chirp->getOrigin());
        if (gw != Gateways.end())
        {
            std::map<Ipv4Address, PCE_map*> interNodes = gw->second;
            for (std::map<Ipv4Address, PCE_map*>::iterator act = interNodes.begin(); act != interNodes.end(); act++) {
                Ipv4Address nextHop = act->first;
                Ipv4Address destination = act->second->getDestination();
                double Qmap = act->second->Qmap();
                double Vmap = act->second->Vmap();
                double Qc = act->second->Qc();
                double Vc = act->second->Vc();
                EV_WARN << "Qmap: " << Qmap << std::endl;
            }
        }
    }

    Ipv4Address origin = chirp->getOrigin();
    Ipv4Address gateway = chirp->getHop();
    float omega_map = chirp->getOmega_map();
    float omega_c = chirp->getOmega_c();


//    ExtendedBonnMotionMobility *neighbor_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[gateway]);
//    ExtendedBonnMotionMobility *self_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[m_selfIpv4Address]);
//    Coord pn = neighbor_mob->getCurrentPosition();
//    double x = pn.x;
//    double y = pn.y;
//    double z = pn.z;
//    Coord pi = self_mob->getCurrentPosition();
//    float self_x = pi.x;
//    float self_y = pi.y;
//    float self_z = pi.z;
//    double distance = sqrt((x-self_x)*(x-self_x) + (y-self_y)*(y-self_y) + (z-self_z)*(z-self_z));

    double gamma_future_map = Cal_Gamma_Future_map(gateway);
    double gamma_future_c = Cal_Gamma_Future_c(gateway);

    unsigned short squNr = chirp->getSquNr();
    int hopCount = chirp->getHopCount();

    // remove self message
    if (origin == m_selfIpv4Address) {
        return 0;
    }

    std::map<Ipv4Address, PDC_map*>::iterator nj = Vi.find(gateway);
    // Ensure next hop is registered in neighbors
    if (nj != Vi.end() && origin != m_selfIpv4Address) {
        nj->second->lastSeen(simTime().dbl());
        nj->second->Gamma_Future_map(gamma_future_map);
        nj->second->Gamma_Future_c(gamma_future_c);
    }
    else {
        PDC_map *data = new PDC_map();
        data->lastSeen(simTime().dbl());
        data->Gamma_Future_map(gamma_future_map);
        data->Gamma_Future_c(gamma_future_c);
        Vi.insert(std::make_pair(gateway, data));
    }

    if (Gateways.find(origin) == Gateways.end()) {
        // Case 1: Origin was not captured as destination at all
        PCE_map *data = new PCE_map(origin);
        data->lastSeen(simTime().dbl());
        data->squNr(squNr);
        data->Qmap(0);
        data->Qc(0);
        data->Vmap(omega_map);
        data->Vc(omega_c);
        std::map<Ipv4Address, PCE_map*> gw = { { gateway, data } };
        Gateways[origin] = gw;
        float omega_map_new = qFunction_map(origin, gateway);
        float omega_c_new = qFunction_c(origin, gateway);
        PCE_map *data_ = Gateways.at(origin).at(gateway);
        data_->Qmap(omega_map_new);
        data_->Qc(omega_c_new);
    }
    else {
        // Case 2: Origin was captured as destination, but not via this hop
        std::map<Ipv4Address, std::map<Ipv4Address, PCE_map*>>::iterator gw = Gateways.find(origin);
        if (gw->second.find(gateway) == gw->second.end()) {
            // Make new entry
            PCE_map *data = new PCE_map(origin);
            data->lastSeen(simTime().dbl());
            data->squNr(squNr);
            data->Qc(0);
            data->Qmap(0);
            data->Vmap(omega_map);
            data->Vc(omega_c);
            gw->second[gateway] = data;
            Gateways[origin][gateway]->Qmap(qFunction_map(origin, gateway));
            Gateways[origin][gateway]->Qc(qFunction_c(origin, gateway));
        }
        else {
            // Case 3: Origin was already captured as destination via this hop
            if (squNr > gw->second[gateway]->squNr()) {
                gw->second[gateway]->squNr(squNr);
                gw->second[gateway]->lastSeen(simTime().dbl());
                gw->second[gateway]->Vmap(omega_map);
                gw->second[gateway]->Vc(omega_c);
                gw->second[gateway]->Qmap(qFunction_map(origin, gateway));
                gw->second[gateway]->Qc(qFunction_c(origin, gateway));
            }
            else if(squNr == gw->second[gateway]->squNr()) {
                if(omega_map > gw->second[gateway]->Vmap()) {
                    gw->second[gateway]->squNr(squNr);
                    gw->second[gateway]->lastSeen(simTime().dbl());
                    gw->second[gateway]->Vmap(omega_map);
                    gw->second[gateway]->Qmap(qFunction_map(origin, gateway));
                }
                if(omega_c > gw->second[gateway]->Vc()) {
                    gw->second[gateway]->squNr(squNr);
                    gw->second[gateway]->lastSeen(simTime().dbl());
                    gw->second[gateway]->Vc(omega_c);
                    gw->second[gateway]->Qc(qFunction_c(origin, gateway));
                }
            }
            else {
                return 0;
            }
        }
    }

    return --hopCount;
}

//void PARRoT_map::purgeNeighbors() {
//    // First delete invalid entrys
//    for (std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator t =
//            Gateways.begin(); t != Gateways.end(); t++) {
//        Ipv4Address target = t->first;
//        std::map<Ipv4Address, PCE*> interNodes = t->second;
//        std::vector<Ipv4Address> toDelete;
//
//        for (std::map<Ipv4Address, PCE*>::iterator act = interNodes.begin();
//                act != interNodes.end();
//                act++) {
//            double deltaT = simTime().dbl() - act->second->lastSeen();
//            if (deltaT > std::min(std::max(neighborReliabilityTimeout, mhChirpInterval), Gamma_Pos(act->first))){
//                toDelete.push_back(act->first);
////              delete act->second;
////              Gateways.at(target).erase(act);
//            }
//        }
//
//        for (const auto& addr : toDelete) {
//            auto it = interNodes.find(addr);
//            if (it != interNodes.end()) {
//                // todo: check why delete cause crash
////                delete static_cast<PCE*>(it->second);  // 释放 PCE 对象的内存
//                interNodes.erase(it);  // 从 map 中删除元素
//            }
//        }
//    }
//
//    // Check if neighbor is still usefull
//    for (std::map<Ipv4Address, PDC*>::iterator n = Vi.begin(); n != Vi.end();
//            n++) {
//        bool useful = false;
//        for (std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator t =
//                Gateways.begin(); t != Gateways.end(); t++) {
//            useful = useful || (t->second.find(n->first) != t->second.end());
//        }
//        if (!useful) {
//            delete n->second;
//            Vi.erase(n);
//        }
//    }
//}

void PARRoT_map::sendMultiHopChirp() {
    auto chirp = makeShared<MultiHopChirp_map>();

    // Set identification fields
    chirp->setOrigin(m_selfIpv4Address);
    chirp->setHop(m_selfIpv4Address);
    chirp->setSquNr(m_squNr);
    m_squNr++;

    // Set fallback information
    Coord p;
    p = check_and_cast<ExtendedBonnMotionMobility*>(mobility)->getCurrentRealPosition();
//    Coord forecast = forecastPosition();
//    Coord v_int = (forecast - p)/((neighborReliabilityTimeout != 0) ? neighborReliabilityTimeout : 1.0);
//    chirp->setX((float)p.getX());
//    chirp->setY((float)p.getY());
//    chirp->setZ((float)p.getZ());
//    chirp->setVX((float)v_int.getX());
//    chirp->setVY((float)v_int.getY());
//    chirp->setVZ((float)v_int.getZ());
    // Set value (initially to 1 respectively 100%)
//    chirp->setValue(1);
    chirp->setOmega_c(1);
    chirp->setOmega_map(1);
    chirp->setHopCount(maxHops);

    int totalByteLength = sizeof(m_selfIpv4Address) * 2
          + sizeof(m_squNr)
          + 6 * sizeof(p.getX())
          + 2*sizeof(m_Gamma_Mob) + sizeof(maxHops);
    chirp->setChunkLength(B(totalByteLength));
    // Creating and sending packet
    auto packet = new Packet("multiHopChirpMap", chirp);
    setDownControlInfo(packet, MacAddress::BROADCAST_ADDRESS);
    sendDown(packet);

    packet = nullptr;
    chirp = nullptr;
}

}
