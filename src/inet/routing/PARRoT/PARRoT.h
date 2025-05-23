/*
 * 
 *
 *  Created on: Mar 25, 2020
 *      Author: cedrik
 *
 *      PARRoT - Predictive Ad-hoc Routing Fueled by Reinforcement Learning and Trajectory Knowledge
 */

#ifndef INET_ROUTING_PARRoT_PARRoT_H_
#define INET_ROUTING_PARRoT_PARRoT_H_

#include <iomanip>      // std::setprecision

#include "inet/common/geometry/common/Coord.h"
#include "inet/common/INETDefs.h"
#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/ModuleAccess.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "inet/networklayer/ipv4/IIpv4RoutingTable.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/ipv4/Ipv4InterfaceData.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/common/NextHopAddressTag_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/linklayer/ieee80211/mac/Ieee80211Mac.h"
//#include "inet/physicallayer/contract/packetlevel/IRadioMedium.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadioMedium.h"
#include "inet/mobility/single/ExtendedBonnMotionMobility.h"

#include "inet/physicallayer/wireless/common/medium/RadioMedium.h"
#include "inet/physicallayer/pathloss/FactoryFading.h"


#include "PDC.h"
#include "PCE.h"
#include "OneHopChirp_m.h"
#include "MultiHopChirp_m.h"

namespace inet {

class PARRoT: public RoutingProtocolBase {
	// PARRoT
	public:
		PARRoT();
		~PARRoT();

        physicallayer::FactoryFading * pathLoss = nullptr;

	protected:
		virtual int numInitStages() const override {
			return NUM_INIT_STAGES;
		}
		virtual void initialize(int stage) override;

		virtual void handleStartOperation(LifecycleOperation *operation)
		        override {
			start();
		}
		virtual void handleStopOperation(LifecycleOperation *operation)
		        override {
			stop();
		}
		virtual void handleCrashOperation(LifecycleOperation *operation)
		        override {
			stop();
		}

		void start();
		void stop();
		void finish() override;


		std::map<Ipv4Address, std::map<Ipv4Address, PCE*>> Gateways;
		std::map<Ipv4Address, PDC*> Vi;
		std::map<SimTime, Ipv4Address> destinationsToUpdate;

		// Pointer
		INetfilter *networkProtocol = nullptr;
		NetworkInterface *interface80211ptr = nullptr;
		IInterfaceTable *ift = nullptr;
		IIpv4RoutingTable *rt = nullptr;
		physicallayer::IRadioMedium *radioMedium = nullptr;
		ieee80211::Dcaf *dcaf_ptr = nullptr;
		IMobility *mobility = nullptr;
		cPar *broadcastDelay = nullptr;
		cMessage *multiHopChirpReminder = nullptr;
		cMessage *destinationReminder = nullptr;

		// Own Identification
		int interfaceId = -1;
		Ipv4Address m_selfIpv4Address;
		unsigned short m_squNr = 0;

	// Routing
	protected:
		int handleIncomingMultiHopChirp(MultiHopChirp *msg, int64_t len);
		bool postliminaryChecksPassed(Ipv4Address origin, Ipv4Address gateway);
		void sendMultiHopChirp();
		void refreshRoutingTable(Ipv4Address origin);
		void purgeNeighbors();
		void sendOneHopChirp(Ipv4Address destination);
		int handleIncomingOneHopChirp(OneHopChirp *chirp, int64_t len);
		virtual void handleMessageWhenUp(cMessage *msg) override;
		virtual void handleSelfMessage(cMessage *msg);

		double mhChirpInterval;
		unsigned short maxHops;
		double neighborReliabilityTimeout;
		bool rescheduleRoutesOnTimeout;
		bool useOHRepair;
		double rangeOffset;
		double losRange;
		double nlosRange;
		int LETRangeMode;
		std::map<Ipv4Address, ExtendedBonnMotionMobility*> _globalMob;

	// Reinforcement Learning
	protected:
		double qFunction(Ipv4Address target, Ipv4Address hop);
//		double qFunction_c(Ipv4Address target, Ipv4Address hop);
//		double qFunction_map(Ipv4Address target, Ipv4Address hop);
		double R(Ipv4Address origin, Ipv4Address hop);
		double Gamma_Pos(Ipv4Address a, Ipv4Address origin = Ipv4Address("0.0.0.0"));
		double combineDiscounts(std::vector<double> gamma);
		double getMaxValueFor(Ipv4Address target);
//		double getMaxValueFor_c(Ipv4Address target);
//		double getMaxValueFor_map(Ipv4Address target);
		Ipv4Address getNextHopFor(Ipv4Address target);
		void updateGamma_Mob();

		double qFctAlpha;
		double qFctGamma;
		double m_Gamma_Mob;
		std::string combinationMethod;

	// Mobility Prediction
	protected:
        void trackPosition(Coord pos);
        Coord forecastPosition();
        Coord predictWithHistory(std::deque<Coord> _historyData, std::deque<double> times, int _nextTime_ms);
        Coord predictWithTarget(Coord _currentData, int m_updateInterval_ms, Coord wp0);

		bool advancedMobility;
		int historySize;
        std::deque<double> hist_coord_t;
        std::deque<Coord> hist_coord;
		std::vector<Ipv4Address> lastSetOfNeighbors;
		std::string predictionMethod;

};

class INET_API PARRoTRoute: public Ipv4Route {
		// Adaption based on DSDV implementation
		// Credits go to its authors!
	protected:
		unsigned int sequencenumber; // originated from destination. Ensures loop freeness.
		simtime_t expiryTime;  // time the routing entry is valid until

	public:
		virtual bool isValid() const {
			return expiryTime == 0 || expiryTime > simTime();
		}

		simtime_t getExpiryTime() const {
			return expiryTime;
		}
		void setExpiryTime(simtime_t time) {
			expiryTime = time;
		}
		void setSequencenumber(int i) {
			sequencenumber = i;
		}
		unsigned int getSequencenumber() const {
			return sequencenumber;
		}
};
} // namespace inet

#endif /* INET_ROUTING_PARRoT_PARRoT_H_ */
