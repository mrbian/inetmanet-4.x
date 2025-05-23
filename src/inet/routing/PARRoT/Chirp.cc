/*
 * Chirp.cc
 *
 *  Created on: Mar 25, 2020
 *      Author: cedrik
 */

#include "PARRoT.h"
#include <fstream>
namespace inet {
// 	Incoming message management
void PARRoT::handleMessageWhenUp(cMessage *msg) {
	if (msg->isSelfMessage()) {
		handleSelfMessage(msg);
	}
	else if (check_and_cast<Packet*>(msg)->getTag<PacketProtocolTag>()->getProtocol()
	        == &Protocol::manet) {
	    const Protocol* proto = check_and_cast<Packet*>(msg)->getTag<PacketProtocolTag>()->getProtocol();
		if (strcmp(msg->getName(), "multiHopChirp") == 0) {
			auto incomingMultiHop =
			        (staticPtrCast<MultiHopChirp>(
			                check_and_cast<Packet*>(msg)->peekData<MultiHopChirp>()->dupShared()));
			// todo: Only Chunk length (->getBitLength()) or whole packet? (->getTotalLength().get())
			int remainingHops = handleIncomingMultiHopChirp(
			        &(*incomingMultiHop),
			        check_and_cast<Packet*>(msg)->getBitLength());
			if (remainingHops > 0
			        && postliminaryChecksPassed(incomingMultiHop->getOrigin(),
			                incomingMultiHop->getHop())) {

				// Set route value
				incomingMultiHop->setValue((float)
				        getMaxValueFor(incomingMultiHop->getOrigin()));

//                incomingMultiHop->setValue_c((float)getMaxValueFor_c(incomingMultiHop->getOrigin()));
//
//                incomingMultiHop->setValue_map((float)getMaxValueFor_map(incomingMultiHop->getOrigin()));


				// Set neighbor fallback information
				incomingMultiHop->setHop(m_selfIpv4Address);

				Coord forecast = forecastPosition();
				Coord p = (hist_coord.size() != 0) ? hist_coord[historySize - 1] : Coord::NIL;
				incomingMultiHop->setX((float)p.getX());
				incomingMultiHop->setY((float)p.getY());
				incomingMultiHop->setZ((float)p.getZ());

				Coord v_int = (forecast - p)/((neighborReliabilityTimeout != 0) ? neighborReliabilityTimeout : 1.0);
				incomingMultiHop->setVX((float)v_int.getX());
				incomingMultiHop->setVY((float)v_int.getY());
				incomingMultiHop->setVZ((float)v_int.getZ());

				incomingMultiHop->setGamma_mob((float)m_Gamma_Mob);

				// Update hop count
				incomingMultiHop->setHopCount(remainingHops);

				auto packet = new Packet("multiHopChirp", incomingMultiHop);
				packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(
				        Ipv4Address(255, 255, 255, 255)); // let's try the limited broadcast 255.255.255.255 but
				                                          // multicast goes from 224.0.0.0 to 239.255.255.255
				packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(
				        m_selfIpv4Address); // let's try the limited broadcast
				packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(
				        interface80211ptr->getInterfaceId());
				packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(
				        &Protocol::manet);
				packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(
				        &Protocol::ipv4);

				sendDelayed(packet, broadcastDelay->doubleValue(), "ipOut");
				packet = nullptr;
			}
			delete msg;
		}else if (strcmp(msg->getName(), "oneHopChirp") == 0){
			auto incomingOneHop =
			        (staticPtrCast<OneHopChirp>(
			                check_and_cast<Packet*>(msg)->peekData<OneHopChirp>()->dupShared()));
			// todo: Only Chunk length (->getBitLength()) or whole packet? (->getTotalLength().get())
			int remainingHops = handleIncomingOneHopChirp(
			        &(*incomingOneHop),
			        check_and_cast<Packet*>(msg)->getBitLength());
			delete msg;
		}else {
			delete msg;
		}
	}
	else {
		throw cRuntimeError("Message not supported %s", msg->getName());
	}
}

bool PARRoT::postliminaryChecksPassed(Ipv4Address origin, Ipv4Address gateway) {
	if (Vi.size() == 0) {
		// No neighbors
		return false;
	}
	else if (Vi.size() == 1) {
		// One neighbor, was this neighbor the sender/origin of the chirp?
		return !(Vi.begin()->first == origin || Vi.begin()->first == gateway);
	}
	else if (rt->findBestMatchingRoute(origin)
	        && rt->findBestMatchingRoute(origin)->getGateway() != gateway) {
		// Neighbor was not the best choice, no need to propagate this.
		return false;
	}
	else {
		return true;
	}
}

//bool PARRoT::postliminaryChecksPassed(Ipv4Address origin, Ipv4Address gateway) {
//    if (Vi.size() == 0) {
//        // No neighbors
//        return false;
//    }
////    else if (Vi.size() == 1) {
////        // One neighbor, was this neighbor the sender/origin of the chirp?
////        return !(Vi.begin()->first == origin || Vi.begin()->first == gateway);
////    }
//    else if (rt->findBestMatchingRoute(origin)
//            && rt->findBestMatchingRoute(origin)->getGateway() != gateway) {
//        // Neighbor was not the best choice, no need to propagate this.
//        return false;
//    }
//    else {
//        return true;
//    }
//}

void PARRoT::handleSelfMessage(cMessage *msg) {
	if (msg == multiHopChirpReminder) {
//	    int node_id = getContainingNode(this)->getId();
//	    if(node_id == 4)
//	    {
	        sendMultiHopChirp();
//	    }
		scheduleAt(simTime() + mhChirpInterval + broadcastDelay->doubleValue(),
		        multiHopChirpReminder);
	}
	else if (msg == destinationReminder) {
        // Erase handled Event
		if(!destinationsToUpdate.empty()){
            Ipv4Address target = destinationsToUpdate.begin()->second;
            destinationsToUpdate.erase(destinationsToUpdate.begin());
            for (std::map<SimTime, Ipv4Address>::iterator it = destinationsToUpdate.begin(); it != destinationsToUpdate.end(); it++){
                if(it->second == target){
                    destinationsToUpdate.erase(it);
                }
            }
            refreshRoutingTable(target);
		}
		// Schedule next event
		if(!destinationsToUpdate.empty()){
			if(destinationReminder->isScheduled()){
				cancelEvent(destinationReminder);
			}
			scheduleAt(destinationsToUpdate.begin()->first, destinationReminder);
		}
	}
	else {
		delete msg;
	}
}

void PARRoT::updateGamma_Mob() {
	int exclusive = 0;
	int merged = 0;
	std::vector<Ipv4Address> currentSetOfNeighbors;
	for (std::map<Ipv4Address, PDC*>::iterator it = Vi.begin(); it != Vi.end();
	        it++) {
		if (simTime().dbl() - it->second->lastSeen()
		        <= neighborReliabilityTimeout) {
			currentSetOfNeighbors.push_back(it->first);
		}
	}

	for (std::vector<Ipv4Address>::iterator o = lastSetOfNeighbors.begin();
	        o != lastSetOfNeighbors.end(); o++) {
		if (std::find(currentSetOfNeighbors.begin(),
		        currentSetOfNeighbors.end(), (*o))
		        == currentSetOfNeighbors.end()) {
			// If entry is not in other vector, it is exclusive in first vector
			exclusive++;
		}
		// Every entry in first vector is a member of the union
		merged++;
	}
	for (std::vector<Ipv4Address>::iterator n = currentSetOfNeighbors.begin();
	        n != currentSetOfNeighbors.end(); n++) {
		if (std::find(lastSetOfNeighbors.begin(), lastSetOfNeighbors.end(),
		        (*n)) == lastSetOfNeighbors.end()) {
			// Only exclusive entries of second vector are considered to the merged counter, as all common entries have already been concerned in first iteration
			exclusive++;
			merged++;
		}
	}

	lastSetOfNeighbors = currentSetOfNeighbors;
	m_Gamma_Mob =
	        (merged != 0) ?
	                sqrt(
	                        1
	                                - static_cast<double>(exclusive)
	                                        / static_cast<double>(merged)) :
	                0.0;
}

////  Multi-Hop signalling (incoming)
int PARRoT::handleIncomingMultiHopChirp(MultiHopChirp *chirp, int64_t len) {
    Ipv4Address origin = chirp->getOrigin();
    Ipv4Address gateway = chirp->getHop();

    float val = chirp->getValue();

    float x = chirp->getX();
    float y = chirp->getY();
    float z = chirp->getZ();
    float vx = chirp->getVX();
    float vy = chirp->getVY();
    float vz = chirp->getVZ();


    Coord pi = hist_coord[historySize - 1];
    double self_x = pi.x;
    double self_y = pi.y;
    double self_z = pi.z;
    double distance = sqrt((x-self_x)*(x-self_x) + (y-self_y)*(y-self_y) + (z-self_z)*(z-self_z));
//  if(enableLosMap)
//  {
////        if()
//  }
//  else
//  {
//      if(distance >= maxRangeForLET)
//      {
//          return 0;
//      }
//  }

    float gamma_mob = chirp->getGamma_mob();
    unsigned short squNr = chirp->getSquNr();

    int hopCount = chirp->getHopCount();

    if (origin == m_selfIpv4Address) {
        return 0;
    }

    std::map<Ipv4Address, PDC*>::iterator nj = Vi.find(gateway);
    // Ensure next hop is registered in neighbors
    if (nj != Vi.end() && origin != m_selfIpv4Address) {
        nj->second->lastSeen(simTime().dbl());
        nj->second->coord(x, y, z);
        nj->second->velo(vx, vy, vz);
        nj->second->Gamma_Mob(gamma_mob);
        nj->second->Gamma_Pos(Gamma_Pos(gateway));
    }
    else {
        PDC *data = new PDC();
        data->lastSeen(simTime().dbl());
        data->coord(x, y, z);
        data->velo(vx, vy, vz);
        data->Gamma_Mob(gamma_mob);
        Vi.insert(std::make_pair(gateway, data));
        data->Gamma_Pos(Gamma_Pos(gateway));
    }

    if (Gateways.find(origin) == Gateways.end()) {
        // Case 1: Origin was not captured as destination at all
        PCE *data = new PCE(origin);
        data->lastSeen(simTime().dbl());
        data->Q(0);
        data->squNr(squNr);
        data->V(val);
        std::map<Ipv4Address, PCE*> gw = { { gateway, data } };
        Gateways[origin] = gw;
        float qval = qFunction(origin, gateway);
        PCE *data_ = Gateways.at(origin).at(gateway);
        data_->Q(qval);
    }
    else {
        // Case 2: Origin was captured as destination, but not via this hop
        std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator gw =
                Gateways.find(origin);
        if (gw->second.find(gateway) == gw->second.end()) {
            // Make new entry
            PCE *data = new PCE(origin);
            data->lastSeen(simTime().dbl());
            data->Q(0);
            data->squNr(squNr);
            data->V(val);
            gw->second[gateway] = data;
            Gateways[origin][gateway]->Q(
                    qFunction(origin, gateway));
        }
        else {
            // Case 3: Origin was already captured as destination via this hop
            if (squNr > gw->second[gateway]->squNr()
                    || (squNr == gw->second[gateway]->squNr()
                            && val > gw->second[gateway]->V())) {
                gw->second[gateway]->squNr(squNr);
                gw->second[gateway]->lastSeen(simTime().dbl());
                gw->second[gateway]->V(val);
                gw->second[gateway]->Q(qFunction(origin, gateway));
            }
            else {
                return 0;
            }
        }
    }

    refreshRoutingTable(origin);
    return --hopCount;
}


//	Multi-Hop signalling (incoming)
//int PARRoT::handleIncomingMultiHopChirp(MultiHopChirp *chirp, int64_t len) {
//    int node_id = getContainingNode(this)->getId();
//    if(node_id == 5)
//    {
//        for (std::map<Ipv4Address, PDC*>::iterator n = Vi.begin(); n != Vi.end(); n++) {
//            double distance = n->second->Gamma_Distance();
//            EV_WARN << "Neighbor: " << n->second->Gamma_Distance() << std::endl;
//        }
//        std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator gw = Gateways.find(chirp->getOrigin());
//        if (gw != Gateways.end())
//        {
//            std::map<Ipv4Address, PCE*> interNodes = gw->second;
//            for (std::map<Ipv4Address, PCE*>::iterator act = interNodes.begin(); act != interNodes.end(); act++) {
//                Ipv4Address nextHop = act->first;
//                Ipv4Address destination = act->second->getDestination();
//                double Qc = act->second->Qc();
//                double Vc = act->second->Vc();
//                double Qmap = act->second->Qmap();
//                double Vmap = act->second->Vmap();
//                EV_WARN << "Qmap: " << Qmap << std::endl;
//            }
//        }
//    }
//
//    Ipv4Address origin = chirp->getOrigin();
//	Ipv4Address gateway = chirp->getHop();
//
////	float val = chirp->getValue();
//	float val_c = chirp->getValue_c();
//	float val_map = chirp->getValue_map();
//
//	ExtendedBonnMotionMobility *neighbor_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[gateway]);
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
//
////	float x = chirp->getX();
////	float y = chirp->getY();
////	float z = chirp->getZ();
////
////	Coord pi = hist_coord[historySize - 1];
////	double self_x = pi.x;
////	double self_y = pi.y;
////	double self_z = pi.z;
////	double distance = sqrt((x-self_x)*(x-self_x) + (y-self_y)*(y-self_y) + (z-self_z)*(z-self_z));
////	if(enableLosMap)
////	{
//////	    if()
////	}
////	else
////	{
////	    if(distance >= maxRangeForLET)
////	    {
////	        return 0;
////	    }
////	}
//
//	unsigned short squNr = chirp->getSquNr();
//
//	int hopCount = chirp->getHopCount();
//
//	if (origin == m_selfIpv4Address) {
//		return 0;
//	}
//
//	std::map<Ipv4Address, PDC*>::iterator nj = Vi.find(gateway);
//	// Ensure next hop is registered in neighbors
//	if (nj != Vi.end() && origin != m_selfIpv4Address) {
//		nj->second->lastSeen(simTime().dbl());
//		nj->second->Gamma_Distance(distance);
//	}
//	else {
//		PDC *data = new PDC();
//		data->lastSeen(simTime().dbl());
//		data->Gamma_Distance(distance);
//		Vi.insert(std::make_pair(gateway, data));
//	}
//
//	if (Gateways.find(origin) == Gateways.end()) {
//		// Case 1: Origin was not captured as destination at all
//		PCE *data = new PCE(origin);
//		data->lastSeen(simTime().dbl());
//		data->Qc(0);
//		data->Qmap(0);
//		data->squNr(squNr);
//		data->Vc(val_c);
//		data->Vmap(val_map);
//		std::map<Ipv4Address, PCE*> gw = { { gateway, data } };
//		Gateways[origin] = gw;
//		float qval_c = qFunction_c(origin, gateway);
//		float qval_map = qFunction_map(origin, gateway);
//		PCE *data_ = Gateways.at(origin).at(gateway);
//		data_->Qc(qval_c);
//		data_->Qmap(qval_map);
//	}
//	else {
//		// Case 2: Origin was captured as destination, but not via this hop
//		std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator gw =
//		        Gateways.find(origin);
//		if (gw->second.find(gateway) == gw->second.end()) {
//			// Make new entry
//			PCE *data = new PCE(origin);
//			data->lastSeen(simTime().dbl());
//	        data->Qc(0);
//	        data->Qmap(0);
//	        data->squNr(squNr);
//	        data->Vc(val_c);
//	        data->Vmap(val_map);
//			gw->second[gateway] = data;
//			Gateways[origin][gateway]->Qc(qFunction_c(origin, gateway));
//			Gateways[origin][gateway]->Qmap(qFunction_map(origin, gateway));
//		}
//		else {
//			// Case 3: Origin was already captured as destination via this hop
//			if (squNr > gw->second[gateway]->squNr()) {
//				gw->second[gateway]->squNr(squNr);
//				gw->second[gateway]->lastSeen(simTime().dbl());
//				gw->second[gateway]->Vc(val_c);
//				gw->second[gateway]->Qc(qFunction_c(origin, gateway));
//                gw->second[gateway]->Vmap(val_map);
//                gw->second[gateway]->Qmap(qFunction_map(origin, gateway));
//			}
//			else if(squNr == gw->second[gateway]->squNr()) {
//			    if(val_c > gw->second[gateway]->Vc()) {
//	                gw->second[gateway]->squNr(squNr);
//	                gw->second[gateway]->lastSeen(simTime().dbl());
//	                gw->second[gateway]->Vc(val_c);
//	                gw->second[gateway]->Qc(qFunction_c(origin, gateway));
//			    }
//			    if(val_map > gw->second[gateway]->Vmap()) {
//	                gw->second[gateway]->squNr(squNr);
//	                gw->second[gateway]->lastSeen(simTime().dbl());
//	                gw->second[gateway]->Vmap(val_map);
//	                gw->second[gateway]->Qmap(qFunction_map(origin, gateway));
//			    }
//			}
//			else {
//				return 0;
//			}
//		}
//	}
//
//	refreshRoutingTable(origin);
//	return --hopCount;
//}

void PARRoT::refreshRoutingTable(Ipv4Address origin) {
	// Purge
//	rt->purge();
	purgeNeighbors();

	// Determine current route and bestRoute
	Ipv4Route *_route = rt->findBestMatchingRoute(origin);
	PARRoTRoute *route = (_route) ? check_and_cast<PARRoTRoute*>(_route) : nullptr;
	Ipv4Address bestHop = getNextHopFor(origin);

	// Do nothing..
	if (!route && bestHop.isUnspecified()) {
		return;
	}
	else {
		if(route && route->getGateway() == bestHop){
			// Best route is already chosen, so don't change it
			return;
		}
		// Remove old route anyway
		if(route){
			rt->removeRoute(route);
		}
		if (bestHop.isUnspecified()) {
			// If bestHop is unspecified, we don't have a new route for this destination
			// Send alert here!
			if(useOHRepair){
				sendOneHopChirp(origin);
			}
		}
		else {
			// Add route to routing table
			PARRoTRoute *e = new PARRoTRoute();
			e->setDestination(origin);
			e->setNetmask(Ipv4Address::ALLONES_ADDRESS);
			e->setGateway(bestHop);
			e->setInterface(interface80211ptr);
			e->setSourceType(IRoute::MANET);
			e->setExpiryTime(simTime() + std::min(std::max(neighborReliabilityTimeout, mhChirpInterval), Gamma_Pos(bestHop, origin)));
			e->setMetric(1); // As only this route to destination is added, just add any number
			rt->addRoute(e);
		}
	}

}


//void PARRoT::refreshRoutingTable(Ipv4Address origin) {
//    // Purge
////  rt->purge();
//    purgeNeighbors();
//
//    // Determine current route and bestRoute
//    Ipv4Route *_route = rt->findBestMatchingRoute(origin);
//    PARRoTRoute *route = (_route) ? check_and_cast<PARRoTRoute*>(_route) : nullptr;
//    Ipv4Address bestHop = getNextHopFor(origin);
//
//    // Do nothing..
//    if (!route && bestHop.isUnspecified()) {
//        return;
//    }
//    else {
//        if(route && route->getGateway() == bestHop){
//            // Best route is already chosen, so don't change it
//            return;
//        }
//        // Remove old route anyway
//        if(route){
//            rt->removeRoute(route);
//        }
//        if (bestHop.isUnspecified()) {
//            // If bestHop is unspecified, we don't have a new route for this destination
//            // Send alert here!
//            if(useOHRepair){
//                sendOneHopChirp(origin);
//            }
//        }
//        else {
//            // Add route to routing table
//            PARRoTRoute *e = new PARRoTRoute();
//            e->setDestination(origin);
//            e->setNetmask(Ipv4Address::ALLONES_ADDRESS);
//            e->setGateway(bestHop);
//            e->setInterface(interface80211ptr);
//            e->setSourceType(IRoute::MANET);
//            e->setExpiryTime(simTime() + std::max(neighborReliabilityTimeout, mhChirpInterval));
//            e->setMetric(1); // As only this route to destination is added, just add any number
//            rt->addRoute(e);
//        }
//    }
//
//}

void PARRoT::purgeNeighbors() {
//    for (const auto& outerPair : Gateways) {
//        EV_WARN << "Gateway: " << outerPair.first << std::endl;
//
//        for (const auto& innerPair : outerPair.second) {
//            EV_WARN << "inner: " << innerPair.first << std::endl;
//            EV_WARN << static_cast<PCE*>(innerPair.second)->Q() << std::endl;
//        }
//    }

	// First delete invalid entrys
	for (std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator t =
	        Gateways.begin(); t != Gateways.end(); t++) {
		Ipv4Address target = t->first;
		std::map<Ipv4Address, PCE*> interNodes = t->second;
		std::vector<Ipv4Address> toDelete;

		for (std::map<Ipv4Address, PCE*>::iterator act = interNodes.begin();
		        act != interNodes.end();
		        act++) {
			double deltaT = simTime().dbl() - act->second->lastSeen();
			if (deltaT > std::min(std::max(neighborReliabilityTimeout, mhChirpInterval), Gamma_Pos(act->first))){
			    toDelete.push_back(act->first);
//			    delete act->second;
//				Gateways.at(target).erase(act);
			}
		}

		for (const auto& addr : toDelete) {
            auto it = interNodes.find(addr);
            if (it != interNodes.end()) {
                // todo: check why delete cause crash
//                delete static_cast<PCE*>(it->second);  // 释放 PCE 对象的内存
                interNodes.erase(it);  // 从 map 中删除元素
            }
        }
	}

	// Check if neighbor is still usefull
	for (std::map<Ipv4Address, PDC*>::iterator n = Vi.begin(); n != Vi.end();
	        n++) {
		bool useful = false;
		for (std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator t =
		        Gateways.begin(); t != Gateways.end(); t++) {
			useful = useful || (t->second.find(n->first) != t->second.end());
		}
		if (!useful) {
			delete n->second;
			Vi.erase(n);
		}
	}
}

void PARRoT::sendOneHopChirp(Ipv4Address destination) {
	auto chirp = makeShared<OneHopChirp>();
	// Set identification fields
	chirp->setOrigin(m_selfIpv4Address);
	chirp->setDestination(destination);

	int totalByteLength = sizeof(m_selfIpv4Address) * 2;
	chirp->setChunkLength(B(totalByteLength));
	// Creating and sending packet
	auto packet = new Packet("oneHopChirp", chirp);
	packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(
	        Ipv4Address(255, 255, 255, 255));
	packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(m_selfIpv4Address);
	packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(
	        interface80211ptr->getInterfaceId());
	packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::manet);
	packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);
	sendDelayed(packet, broadcastDelay->doubleValue(), "ipOut");
	packet = nullptr;
	chirp = nullptr;
}

int PARRoT::handleIncomingOneHopChirp(OneHopChirp *chirp, int64_t len) {
	Ipv4Address hop = chirp->getOrigin();
	Ipv4Address destination = chirp->getDestination();

	// Determine current route and bestRoute
	Ipv4Route *_route = rt->findBestMatchingRoute(destination);
	PARRoTRoute *route =
	        (_route) ? check_and_cast<PARRoTRoute*>(_route) : nullptr;

	std::map<Ipv4Address, std::map<Ipv4Address, PCE*>>::iterator dstEntry = Gateways.find(destination);
	if (dstEntry != Gateways.end()){
		std::map<Ipv4Address, PCE*>::iterator hopEntry = dstEntry->second.find(hop);
		if(hopEntry != dstEntry->second.end()){
			hopEntry->second->Q(0);
			hopEntry->second->V(0);
		}
		if(route && route->getGateway() == hop){
			refreshRoutingTable(destination);
		}
	}
	return 0;
}

////	Multi-Hop signalling (outgoing)
void PARRoT::sendMultiHopChirp() {
	auto chirp = makeShared<MultiHopChirp>();

	// Set identification fields
	chirp->setOrigin(m_selfIpv4Address);
	chirp->setHop(m_selfIpv4Address);
	chirp->setSquNr(m_squNr);
	m_squNr++;

	// Set fallback information
	Coord p;
	if(advancedMobility){
		p = check_and_cast<ExtendedBonnMotionMobility*>(mobility)->getCurrentRealPosition();
	}else{
		p = mobility->getCurrentPosition();
	}
	trackPosition(p);
	Coord forecast = forecastPosition();
	Coord v_int = (forecast - p)/((neighborReliabilityTimeout != 0) ? neighborReliabilityTimeout : 1.0);
	chirp->setX((float)p.getX());
	chirp->setY((float)p.getY());
	chirp->setZ((float)p.getZ());
	chirp->setVX((float)v_int.getX());
	chirp->setVY((float)v_int.getY());
	chirp->setVZ((float)v_int.getZ());

	updateGamma_Mob();
	chirp->setGamma_mob((float)m_Gamma_Mob);

	// Set value (initially to 1 respectively 100%)
	chirp->setValue(1);
	chirp->setHopCount(maxHops);

//	int totalByteLength = sizeof(m_selfIpv4Address) * 2
//	        + sizeof(m_squNr)
//	        + 6 * sizeof(p.getX())
//	        + 2*sizeof(m_Gamma_Mob) + sizeof(maxHops);
	int totalByteLength = sizeof(m_selfIpv4Address)
	        + sizeof(m_squNr)
	        + 6 * sizeof((float)p.getX())
	        + 2*sizeof((float)m_Gamma_Mob) + sizeof(maxHops);
	chirp->setChunkLength(B(totalByteLength));
	// Creating and sending packet
	auto packet = new Packet("multiHopChirp", chirp);
	packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(
	        Ipv4Address(255, 255, 255, 255));
	packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(m_selfIpv4Address);
	packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(
	        interface80211ptr->getInterfaceId());
	packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::manet);
	packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);
	sendDelayed(packet, broadcastDelay->doubleValue(), "ipOut");
	packet = nullptr;
	chirp = nullptr;
}


//  Multi-Hop signalling (outgoing)
//void PARRoT::sendMultiHopChirp() {
//    auto chirp = makeShared<MultiHopChirp>();
//
//    // Set identification fields
//    chirp->setOrigin(m_selfIpv4Address);
//    chirp->setHop(m_selfIpv4Address);
//    chirp->setSquNr(m_squNr);
//    m_squNr++;
//
//    // Set fallback information
//    Coord p;
//    if(advancedMobility){
//        p = check_and_cast<ExtendedBonnMotionMobility*>(mobility)->getCurrentRealPosition();
//    }else{
//        p = mobility->getCurrentPosition();
//    }
//    trackPosition(p);
//    Coord forecast = forecastPosition();
//    Coord v_int = (forecast - p)/((neighborReliabilityTimeout != 0) ? neighborReliabilityTimeout : 1.0);
//    chirp->setX((float)p.getX());
//    chirp->setY((float)p.getY());
//    chirp->setZ((float)p.getZ());
//    chirp->setVX((float)v_int.getX());
//    chirp->setVY((float)v_int.getY());
//    chirp->setVZ((float)v_int.getZ());
//
//    updateGamma_Mob();
//    chirp->setGamma_mob((float)m_Gamma_Mob);
//
//    // Set value (initially to 1 respectively 100%)
////    chirp->setValue(1);
//    chirp->setValue_c(1);
//    chirp->setValue_map(1);
//    chirp->setHopCount(maxHops);
//
////  int totalByteLength = sizeof(m_selfIpv4Address) * 2
////          + sizeof(m_squNr)
////          + 6 * sizeof(p.getX())
////          + 2*sizeof(m_Gamma_Mob) + sizeof(maxHops);
//    int totalByteLength = sizeof(m_selfIpv4Address)
//            + sizeof(m_squNr)
//            + 6 * sizeof((float)p.getX())
//            + 2*sizeof((float)m_Gamma_Mob) + sizeof(maxHops);
//    chirp->setChunkLength(B(totalByteLength));
//    // Creating and sending packet
//    auto packet = new Packet("multiHopChirp", chirp);
//    packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(
//            Ipv4Address(255, 255, 255, 255));
//    packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(m_selfIpv4Address);
//    packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(
//            interface80211ptr->getInterfaceId());
//    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::manet);
//    packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);
//    sendDelayed(packet, broadcastDelay->doubleValue(), "ipOut");
//    packet = nullptr;
//    chirp = nullptr;
//}

}	// namespace inet
