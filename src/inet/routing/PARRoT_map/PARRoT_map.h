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

#ifndef INET_ROUTING_PARROT_MAP_PARROTMAP_H_
#define INET_ROUTING_PARROT_MAP_PARROTMAP_H_

#include <iomanip>      // std::setprecision

#include "inet/networklayer/base/NetworkProtocolBase.h"
#include "inet/networklayer/contract/INetworkProtocol.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/common/INETDefs.h"
#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/ModuleAccess.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "inet/networklayer/ipv4/IIpv4RoutingTable.h"
#include "inet/networklayer/contract/IArp.h"
#include "inet/networklayer/arp/ipv4/GlobalArp.h"
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
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/networklayer/common/HopLimitTag_m.h"

#include "inet/physicallayer/wireless/common/medium/RadioMedium.h"
#include "inet/physicallayer/pathloss/FactoryFading.h"


#include "PDC_map.h"
#include "PCE_map.h"
#include "MultiHopChirp_map_m.h"
#include "PDataHeader_m.h"
#include "RobustForwarder_m.h"

#include <torch/script.h>

namespace inet {

class PARRoT_map: public NetworkProtocolBase, public INetworkProtocol {
    // PARRoT_map
    public:
        PARRoT_map();
        ~PARRoT_map();

    protected:
        std::map<Ipv4Address, std::map<Ipv4Address, PCE_map*>> Gateways;
        std::map<Ipv4Address, PDC_map*> Vi;

        // Pointer
        INetfilter *networkProtocol = nullptr;
        GlobalArp* arp = nullptr;
        NetworkInterface *interface80211ptr = nullptr;
        physicallayer::FactoryFading * pathLoss = nullptr;
        IInterfaceTable *ift = nullptr;
        physicallayer::IRadioMedium *radioMedium = nullptr;
        IMobility *mobility = nullptr;

        cPar *broadcastDelay = nullptr;
        cMessage *multiHopChirpReminder = nullptr;

        // Own Identification
        int interfaceId = -1;
        Ipv4Address m_selfIpv4Address;
        unsigned short m_squNr = 0;
        int m_dataSeqNum = 0;

        double mhChirpInterval;
        double discount;
        unsigned short maxHops;
        float m_Gamma_Mob;
        double neighborReliabilityTimeout;
        double losRange;
        double nlosRange;
        int LETRangeMode;
        std::map<Ipv4Address, ExtendedBonnMotionMobility*> _globalMob;
        bool considerSmallScale;
        std::vector<double> rician_outage_vector;
        std::vector<double> rayleigh_outage_vector;
        std::set<std::pair<Ipv4Address, int>> duplicate_data_pkt;

        torch::jit::script::Module model;
        double areaMaxX;
        double areaMaxY;
        double nlosThres;
        int _seed;
        int _block;

    // Routing
    protected:
        // Chirp
        void recv_parrot_map(cMessage *msg);
        int handleIncomingMultiHopChirp(MultiHopChirp_map *msg, int64_t len);
        bool postliminaryChecksPassed(Ipv4Address origin, Ipv4Address gateway);
        void sendMultiHopChirp();
        void purgeNeighbors();
        // Brain
        void handleDataFromUpperLayer(Packet *packet);
        void handleDataFromLowerLayer(Packet *packet);
        void forwardData(Packet* packet, Ipv4Address dest);
        double qFunction_c(Ipv4Address target, Ipv4Address hop);
        double qFunction_map(Ipv4Address target, Ipv4Address hop);
        double getMaxValueFor_c(Ipv4Address target);
        double getMaxValueFor_map(Ipv4Address target);
        Ipv4Address findBestNextHop(Ipv4Address target);
        Ipv4Address findRobustNextHop(Ipv4Address target);
        torch::Tensor get_points(double x1, double y1, double x2, double y2);
        double Cal_Gamma_Future_map(Ipv4Address neighbor);
        double Cal_Gamma_Future_c(Ipv4Address neighbor);

    protected:
      virtual int numInitStages() const override { return NUM_INIT_STAGES; }
      virtual void initialize(int stage) override;

      virtual void handleUpperPacket(Packet *packet) override;
      virtual void handleLowerPacket(Packet *packet) override;
      virtual void handleSelfMessage(cMessage *msg) override;

      void setDownControlInfo(Packet *const pMsg, const MacAddress& pDestAddr);

      const Protocol& getProtocol() const override { return Protocol::ipv4; }

      void start();
      void stop();
      void finish() override;
      virtual void handleStartOperation(LifecycleOperation *operation) override;  // application layer
      virtual void handleStopOperation(LifecycleOperation *operation) override { stop(); }
      virtual void handleCrashOperation(LifecycleOperation *operation) override { stop(); }


};

} // namespace inet

#endif /* INET_ROUTING_PARROT_MAP_PARROTMAP_H_ */
