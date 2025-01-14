/*
 * Brain.cc
 *
 *  Created on: Mar 25, 2020
 *      Author: cedrik
 */
#include "PARRoT.h"
#include <random>
#include <fstream>

namespace inet {

#define LOS 0
#define NLOS 1
#define LOSMAP 2

double PARRoT::combineDiscounts(std::vector<double> gamma) {
	if (combinationMethod == "M") {
		return std::accumulate(gamma.begin(), gamma.end(), 1.0,
		        std::multiplies<double>());
	}
	else if (combinationMethod == "G") {
		// nth-root(g1*g2*..*gn)
		return pow(
		        std::accumulate(gamma.begin(), gamma.end(), 1.0,
		                std::multiplies<double>()),
		        (1 / static_cast<double>(gamma.size())));
	}
	else if (combinationMethod == "A") {
		// 1/n * (g1 + g2 + .. + gn)
		return std::accumulate(gamma.begin(), gamma.end(), 1.0) / gamma.size();
	}
	else if (combinationMethod == "H") {
		// n / (1/g1 + 1/g2 + .. + 1/gn)
		return static_cast<double>(gamma.size())
		        / std::accumulate(gamma.begin(), gamma.end(), 1.0,
		                [&](double res, double c) mutable {
			                res += (c != 0) ? 1 / c : INFINITY;
			                return res;
		                });
	}
	else {
		return std::accumulate(gamma.begin(), gamma.end(), 1.0,
		        std::multiplies<double>());
	}
}


double PARRoT::qFunction(Ipv4Address target, Ipv4Address hop) {
	std::vector<double> discounts;
	discounts.push_back(qFctGamma);
	discounts.push_back(std::min(1.0, sqrt(std::max(Gamma_Pos(hop), 0.0)/(std::max(neighborReliabilityTimeout, mhChirpInterval))) ));
	discounts.push_back(Vi.at(hop)->Gamma_Mob());

	return (1 - qFctAlpha) * Gateways.at(target).at(hop)->Q()
			+ qFctAlpha
					* (combineDiscounts(discounts)
							* Gateways.at(target).at(hop)->V());
}

double PARRoT::getMaxValueFor(Ipv4Address target) {

	double res = -1000;
	if(Gateways.find(target) != Gateways.end()){
	for (std::map<Ipv4Address, PCE*>::iterator act =
	        Gateways.find(target)->second.begin();
	        act != Gateways.find(target)->second.end(); act++) {
		double deltaT = simTime().dbl() - act->second->lastSeen();
		if (deltaT	<= std::min(std::max(neighborReliabilityTimeout, mhChirpInterval), Gamma_Pos(act->first))) {
			if (act == Gateways.find(target)->second.begin()) {
				// First possible action, make sure the result gets this value anyway
				res = qFunction(target, act->first);
			}
			res = std::max(res,
			        qFunction(target, act->first));

		}
	}
	}
	return res;
}

Ipv4Address PARRoT::getNextHopFor(Ipv4Address target) {
	Ipv4Address a = Ipv4Address("0.0.0.0");
	double res = -1000;
	std::vector<Ipv4Address> toDelete;
	if(Gateways.find(target) != Gateways.end()){
	for (std::map<Ipv4Address, PCE*>::iterator act =
	        Gateways.find(target)->second.begin();
	        act != Gateways.find(target)->second.end(); act++) {
		double deltaT = simTime().dbl() - act->second->lastSeen();
		if (deltaT	<= std::min(std::max(neighborReliabilityTimeout, mhChirpInterval), Gamma_Pos(act->first))) {

		    if (act == Gateways.find(target)->second.begin()) {
//				 First possible action, make sure the result gets this value anyway
				res = qFunction(target, act->first);
				a = act->first;
			}
			else if (qFunction(target, act->first) > res) { //act->second->Q() > res) {
				res = qFunction(target, act->first); //act->second->Q();
				a = act->first;
			}
		}
		else {
		    toDelete.push_back(act->first);
//			delete act->second;
//			Gateways.at(target).erase(act);
		}
	}

    for (const auto& addr : toDelete) {
        std::map<Ipv4Address, PCE*> interNodes = Gateways.find(target)->second;
        auto it = interNodes.find(addr);
        if (it != interNodes.end()) {
//                delete static_cast<PCE*>(it->second);  // 释放 PCE 对象的内存
            interNodes.erase(it);  // 从 map 中删除元素
        }
    }
	}



	return a;

}

//	Rewards
double PARRoT::R(Ipv4Address origin, Ipv4Address hop) {
	return 0.0;
}

double PARRoT::Gamma_Pos(Ipv4Address neighbor, Ipv4Address origin) {
    ExtendedBonnMotionMobility *neighbor_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[neighbor]);
    ExtendedBonnMotionMobility *self_mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[m_selfIpv4Address]);
    double period = 0.1;
    int Np = ceil(neighborReliabilityTimeout/period);
    double link_expire_time = 0;
    for(int i=0; i<Np; i++)
    {
        double lookAhead = i * period;
        Coord P1 = neighbor_mob->getFuturePosition(lookAhead, simTime());
        Coord P2 = self_mob->getFuturePosition(lookAhead, simTime());
        double distance = P1.distance(P2);
        if(LETRangeMode == LOS)
        {
            if(distance >= losRange) break;
            link_expire_time += period;
        }
        else if(LETRangeMode == NLOS)
        {
            if(distance >= nlosRange) break;
            link_expire_time += period;
        }
        else if(LETRangeMode == LOSMAP)
        {
            if(pathLoss->checkNlos(P1, P2))
            {
                if(distance >= nlosRange) break;
                link_expire_time += period;
            }
            else
            {
                if(distance >= losRange) break;
                link_expire_time += period;
            }
        }
        else
            throw cRuntimeError("LETRangeMode not supported");
    }

    if (!origin.isUnspecified() && rescheduleRoutesOnTimeout && link_expire_time >= 0.0){
        destinationsToUpdate.insert( { { simTime() + link_expire_time, origin } });
        cancelEvent(destinationReminder);
        scheduleAt(destinationsToUpdate.begin()->first, destinationReminder);
    }

    return link_expire_time;
}


//double PARRoT::Gamma_Pos(Ipv4Address neighbor, Ipv4Address origin) {
//	double t_elapsed_since_last_hello = simTime().dbl() - Vi.at(neighbor)->lastSeen();
//	Coord vj;
//	Coord pj;
//
//	vj = Vi.at(neighbor)->velo();
//	pj = Vi.at(neighbor)->coord() + (vj * (t_elapsed_since_last_hello));
//
//	Coord pi = hist_coord[historySize - 1];
//	Coord vi = (forecastPosition() - pi)/((neighborReliabilityTimeout != 0) ? neighborReliabilityTimeout : 1.0);
//
//	double px = (pj - pi).getX();
//	double vx = (vj - vi).getX();
//	double py = (pj - pi).getY();
//	double vy = (vj - vi).getY();
//	double pz = (pj - pi).getZ();
//	double vz = (vj - vi).getZ();
//
//	double a = pow(vx, 2) + pow(vy, 2) + pow(vz, 2);
//	double b = 2 * (px * vx + py * vy + pz * vz);
//	double c = pow(px, 2) + pow(py, 2) + pow(pz, 2) - pow(nlosRange, 2);
//
////	double c;
////    switch(LETRangeMode)
////    {
////        case LOS:
////            c = pow(px, 2) + pow(py, 2) + pow(pz, 2) - pow(losRange, 2);
////            break;
////        case NLOS:
////            c = pow(px, 2) + pow(py, 2) + pow(pz, 2) - pow(nlosRange, 2);
////            break;
////        case LOSMAP:
////            double sample_interval = 0.1;
////            int Np = (int)(neighborReliabilityTimeout/sample_interval);
////            double link_expire_time = 0;
////            for(int i=0; i < Np; i++)
////            {
////                double t_pred = Np*sample_interval;
////                Coord pi_pred = pi + vi*t_pred;
////                Coord pj_pred = pj + vj*t_pred;
////                double distance = pi_pred.distance(pj_pred);
////                if(pathLoss->checkNlos(pi_pred, pj_pred))
////                {
////                    if(distance > nlosRange)
////                        return link_expire_time;
////                    link_expire_time += period;
////                }
////                else
////                {
////                    if(distance < losRange)
////                        return link_expire_time;
////                    link_expire_time += period;
////                }
////            }
////            return link_expire_time;
////            break;
////        default:
////            throw cRuntimeError("LETRangeMode not supported");
////            break;
////    }
//
//	if (a==0){
////		return (c < 0) ? 1.0 : 0.0;
//	    return (c < 0) ? neighborReliabilityTimeout : 0.0;
//	}
//	double t1 = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
//	double t2 = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
//	double t = (t2 >= 0.0 || (t2 < 0.0 && t1 < 0.0)) ? 0.0 : t1;
//	if(isnan(t)){
//		t = 0.0;
//	}
//	if (!origin.isUnspecified() && rescheduleRoutesOnTimeout && t2 >= 0.0){
//		// If Gamma_Pos < 1.0 we need to schedule a re-calculation event
//		destinationsToUpdate.insert( { { simTime() + t2, origin } });
//		cancelEvent(destinationReminder);
//		scheduleAt(destinationsToUpdate.begin()->first, destinationReminder);
//	}else if(!origin.isUnspecified() && rescheduleRoutesOnTimeout && t2 <= 0.0 && t1 > 0.0 && t1 < neighborReliabilityTimeout){
//		//std::cout << "Reschedule update for " << origin.str() << " in " << t1 << "\n" << std::flush;
//		destinationsToUpdate.insert( { { simTime() + t1, origin } });
//        cancelEvent(destinationReminder);
//        scheduleAt(destinationsToUpdate.begin()->first, destinationReminder);
//	}
//
//	return t;
//}

} // namespace inet

