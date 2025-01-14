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

#ifndef INET_ROUTING_LMPR_LMPR_H_
#define INET_ROUTING_LMPR_LMPR_H_

#include <iomanip>      // std::setprecision
#include <iostream>
#include <vector>
#include <cmath>  // For sqrt
#include "math.h"
#include <queue>        // priority_queue
#include <functional>   // greate

#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/OperationalBase.h"
#include "inet/common/packet/Message.h"

#include "inet/common/packet/Packet.h"
#include "inet/networklayer/base/NetworkProtocolBase.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/networklayer/contract/INetworkProtocol.h"

#include "inet/common/geometry/common/Coord.h"
#include "inet/common/INETDefs.h"
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
#include "inet/networklayer/contract/IArp.h"
#include "inet/networklayer/arp/ipv4/GlobalArp.h"
#include "inet/physicallayer/wireless/common/medium/RadioMedium.h"
#include "inet/physicallayer/pathloss/FactoryFading.h"
#include "inet/networklayer/common/HopLimitTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"

#include "OGM_m.h"
#include "LMPRHeader_m.h"
#include "OriginatorTag_m.h"

using namespace std;

#define INF std::numeric_limits<double>::infinity()

namespace inet {

extern simsignal_t nodeInfoLenSignal;
extern simsignal_t LETSignal;
extern simsignal_t nextNodeChoiceLETSignal;

struct Trajectory {
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
};

struct NodeInfo{
    Ipv4Address addr;
    double time_abstract;
    Trajectory trs;
    int last_bcast_seqno;
    simtime_t bcast_seqno_reset;
    simtime_t last_seen;
};

//struct MapDataEle{
//    double txX;
//    double txY;
//    double rxX;
//    double rxY;
//    double sinr;
//    int losCond;
//};
//
//typedef std::vector<MapDataEle> LosMapDataSet;

class LMPR: public NetworkProtocolBase, public INetworkProtocol
{
public:
    LMPR();
    virtual ~LMPR();

protected:
    // Pointer
    INetfilter *networkProtocol = nullptr;
    NetworkInterface *interface80211ptr = nullptr;
    IInterfaceTable *ift = nullptr;
    IIpv4RoutingTable *rt = nullptr;
    physicallayer::IRadioMedium *radioMedium = nullptr;
    ieee80211::Dcaf *dcaf_ptr = nullptr;
    IMobility *mobility = nullptr;
    cPar *broadcastDelay = nullptr;
    cMessage *OGMReminder = nullptr;
    GlobalArp* arp = nullptr;
    physicallayer::FactoryFading * pathLoss = nullptr;

    std::vector<std::pair<Ipv4Address, ExtendedBonnMotionMobility*>> _globalMob;

    int interfaceId = -1;
    int m_selfNodeIdx;
    Ipv4Address m_selfIpv4Address;
    int m_seqNum = 0;
    int m_OGM_seqNum = 0;

    int N;
    std::vector<std::vector<double>> adjacencyMatrix;

    bool setAutoRange;
    bool setAutoLETRange;

    int LETRangeMode;
    double nlosRange;
    double losRange;

    double losMapError;
    double maxRangeForLET;

    // OGM related
    class INET_API Bcast {
      public:
        int seqNum;
        Ipv4Address origAddr;
        simtime_t delTime;

      public:
        Bcast(int n = 0, const Ipv4Address& s = Ipv4Address(), simtime_t_cref d = SIMTIME_ZERO) :
            seqNum(n), origAddr(s), delTime(d)
        {
        }
    };
    typedef std::list<Bcast> cBroadcastList;
    cBroadcastList bcMsgs;
    std::list<NodeInfo*> nodesInfoList;
    unsigned int bcMaxEntries = 0;
    simtime_t bcDelTime;
    int defaultTTL;
    double mhOGMInterval;
    double neighborReliabilityTimeout;
//    double txRange;
    double predictDuration;
//    LosMapDataSet mapDataSet;


/* Main */
protected:
    virtual int numInitStages() const override {
        return NUM_INIT_STAGES;
    }
    virtual void initialize(int stage) override;
    virtual void handleStartOperation(LifecycleOperation *operation) override {}
    virtual void handleStopOperation(LifecycleOperation *operation) override { stop(); }
    virtual void handleCrashOperation(LifecycleOperation *operation) override { stop(); }
    void start();
    void stop();
    void finish() override;
    virtual void handleUpperPacket(Packet *packet) override;
    virtual void handleLowerPacket(Packet *packet) override;
    virtual void handleSelfMessage(cMessage *msg) override;

//    const NetworkInterface * getDestInterface(Packet *packet);

/* Chirp */
protected:
    void broadcastOGM();
    bool notBroadcasted(const Ptr<OGM> msg);
    void handleOGM(Ptr<OGM> msg, int64_t len);
    void handleOGMReminder();

/* Brain */
protected:
    void handleDataFromLowerLayer(Packet *packet);
    void handleDataFromUpperLayer(Packet *packet);
    void forwardData(Packet *packet, Ipv4Address dest);
    int forwardData_Optimal(Packet* packet, Ipv4Address dest);
    int forwardData_Forecast(Packet* packet, Ipv4Address dest);
    int forwardData_by_ETX_LET(Packet* packet, Ipv4Address dest);
    double calculateLET(int i, int j);

/* Util */
protected:
    const Protocol& getProtocol() const override { return Protocol::ipv4; }
    int dijkstra(const vector<vector<double>>& graph, int source, int target);
    Coord forecastSelfPosition_Optimal(double duration);
    Coord forecastNodePosition_Optimal(int idx, double duration);
    Coord forecastNodeLocation_by_ThreePos(NodeInfo* node_info, simtime_t t);
    int getTargetNodeIdxByAddr(Ipv4Address addr);
    NodeInfo* getNodeInfoByAddr(Ipv4Address addr);
    double getRealCommRange(Coord PosA, Coord PosB);
    double getRangeForLET(Coord PosA, Coord PosB);

    void setDownControlInfo(Packet *const pMsg, const MacAddress& pDestAddr);

//protected:
//    void addMapData(Packet *packet);
//    void storeMapDataToCSV();

};


//    virtual void handleMessageWhenUp(cMessage *msg) override;
//    double calculateLinkExpirationTime(Coord n1, float n1_VX, float n1_VY, Coord n2, float n2_VX, float n2_VY);

//class INET_API LMPR : public NetworkProtocolBase, public INetworkProtocol
//{
//  protected:
//    /** @brief Network layer sequence number*/
//    unsigned long seqNum = 0;
//
//    /** @brief cached variable of my networ address */
//    L3Address myNetwAddr;
//
//    /** @brief Length of the header*/
//    int headerLength = 0;
//
//    /** @brief Default time-to-live (ttl) used for this module*/
//    int defaultTtl = 0;
//
//    /** @brief Defines whether to use plain flooding or not*/
//    bool plainFlooding = false;
//
//    class INET_API Bcast {
//      public:
//        unsigned long seqNum;
//        L3Address srcAddr;
//        simtime_t delTime;
//
//      public:
//        Bcast(unsigned long n = 0, const L3Address& s = L3Address(), simtime_t_cref d = SIMTIME_ZERO) :
//            seqNum(n), srcAddr(s), delTime(d)
//        {
//        }
//    };
//
//    typedef std::list<Bcast> cBroadcastList;
//
//    /** @brief List of already broadcasted messages*/
//    cBroadcastList bcMsgs;
//
//    /**
//     * @brief Max number of entries in the list of already broadcasted
//     * messages
//     **/
//    unsigned int bcMaxEntries = 0;
//
//    /**
//     * @brief Time after which an entry for an already broadcasted msg
//     * can be deleted
//     **/
//    simtime_t bcDelTime;
//
//    long nbDataPacketsReceived = 0;
//    long nbDataPacketsSent = 0;
//    long nbDataPacketsForwarded = 0;
//    long nbHops = 0;
//
//  public:
//    Flooding() {}
//
//    /** @brief Initialization of omnetpp.ini parameters*/
//    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
//
//    virtual void initialize(int) override;
//
//    virtual void finish() override;
//
//    const Protocol& getProtocol() const override { return Protocol::flooding; }
//
//  protected:
//
//    /** @brief Handle messages from upper layer */
//    virtual void handleUpperPacket(Packet *packet) override;
//
//    /** @brief Handle messages from lower layer */
//    virtual void handleLowerPacket(Packet *packet) override;
//
//    /** @brief Checks whether a message was already broadcasted*/
//    bool notBroadcasted(const FloodingHeader *);
//
//    void decapsulate(Packet *packet);
//    void encapsulate(Packet *packet);
//    void forwardPacket(Packet *packet);
//
//    /**
//     * @brief Attaches a "control info" (NetwToMac) structure (object) to the message pMsg.
//     *
//     * This is most useful when passing packets between protocol layers
//     * of a protocol stack, the control info will contain the destination MAC address.
//     *
//     * The "control info" object will be deleted when the message is deleted.
//     * Only one "control info" structure can be attached (the second
//     * setL3ToL2ControlInfo() call throws an error).
//     *
//     * @param pMsg      The message where the "control info" shall be attached.
//     * @param pDestAddr The MAC address of the message receiver.
//     */
//    virtual void setDownControlInfo(Packet *const pMsg, const MacAddress& pDestAddr);
//
//    // OperationalBase:
//    virtual void handleStartOperation(LifecycleOperation *operation) override {} // TODO implementation
//    virtual void handleStopOperation(LifecycleOperation *operation) override {} // TODO implementation
//    virtual void handleCrashOperation(LifecycleOperation *operation) override {} // TODO implementation
//};



} /* namespace inet */


#endif /* INET_ROUTING_LMPR_LMPR_H_ */
