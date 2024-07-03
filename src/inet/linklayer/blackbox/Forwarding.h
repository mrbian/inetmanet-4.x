//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//
#ifndef _INET_BLACKBOX_FORWARDING__
#define _INET_BLACKBOX_FORWARDING__


#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif


#include <map>
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3Address.h"
#include "L2Queue.h"
#include "inet/common/DijktraKShortest.h"
/**
 * Demonstrates static routing, utilizing the cTopology class.
 */
namespace inet {
class NetworkInterface;

namespace blackbox {

class Forwarding: public cSimpleModule, public cListener {
public:

    struct BaseLink {
        virtual bool isPhysical() = 0;
        MacAddress origin;
        MacAddress destination;
        bps capacity;
        virtual bool operator <(const Forwarding::BaseLink &y) {
            if (this->origin != y.origin)
                return this->origin < y.origin;
            return this->destination < y.destination;
        }

        virtual bool operator >(const Forwarding::BaseLink &y) {
            if (this->origin != y.origin)
                return this->origin > y.origin;
            return this->destination > y.destination;
        }

        virtual bool operator ==(const Forwarding::BaseLink &y) {
            return (this->origin == y.origin)
                    && (this->destination == y.destination);
        }

        BaseLink& operator=(const BaseLink &val) {
            if (this == &val)
                return *this;
            this->origin = val.origin;
            this->destination = val.destination;
            this->capacity = val.capacity;
            return *this;
        }
        virtual ~BaseLink(){}
    };

    struct PhysicakLink: public BaseLink {
        virtual bool isPhysical()  override {return true;}
        L2Queue *l2Queue = nullptr;
        PhysicakLink& operator=(const PhysicakLink &val) {
            if (this == &val)
                return *this;
            this->capacity = val.capacity;
            this->origin = val.origin;
            this->destination = val.destination;
            this->l2Queue = val.l2Queue;
            return *this;
        }
        virtual ~PhysicakLink(){}
    };

    struct LogicalLinksLink: public BaseLink {
        virtual bool isPhysical()  override {return false;}
        virtual bool multiLink() {return links.size() > 1;}
        std::vector<std::shared_ptr<PhysicakLink>> links;
        std::vector<MacAddress> nodes;
        double cost = 1;
        uint64_t label = 0;
        LogicalLinksLink& operator=(const LogicalLinksLink &val) {
            if (this == &val)
                return *this;
            links = val.links;
            nodes = val.nodes;
            label = val.label;
            cost = val.cost;
            BaseLink::operator=(val);
            return *this;
        }
        virtual ~LogicalLinksLink(){links.clear(); nodes.clear();}
    };
    typedef std::map<std::pair<MacAddress, MacAddress>, std::shared_ptr<PhysicakLink>> Topology;
    typedef std::map<std::pair<MacAddress, MacAddress>, std::shared_ptr<LogicalLinksLink>> LogicalTopology;
    typedef std::map<uint64_t, std::shared_ptr<LogicalLinksLink>> LabelMap;
    typedef std::map<std::pair<MacAddress, MacAddress>, uint64_t> LabelMapInverse;

private:
    MacAddress myAddress; // the local address will be codified has Mac

    typedef std::map<L3Address, int> RoutingTable;  // destaddr -> gateindex
    RoutingTable rtable;
    RoutingTable neigborTable;
    struct OutNode {
        L3Address addr;
        int ifaceId;
        NetworkInterface *iface = nullptr;
    };
    std::map<int, OutNode> outNodesTable; // index, Ip Address
    std::vector<MacAddress> linkToNeigborTable; // mac address
    typedef std::vector<L3Address> ConnectedAddress; // ip address
    typedef std::map<MacAddress, ConnectedAddress> ConnectedL3Address; // mac address, vector ip address
    static std::map<L3Address, MacAddress> l3AddressDestination; // ip addr, Mac Address

    static Topology topology;
    static LogicalTopology logicalTopology;
    static LabelMap labelMap;
    static ConnectedL3Address connectedL3Address;
    static LabelMapInverse labelMapInverse;
    static uint64_t labelCount;

    Dijkstra djkVirtual;
    Dijkstra djkReal;

public:
    static uint64_t getActualLabel() {return labelCount;}
    static uint64_t getNextLabel() {labelCount++; return labelCount;}

    static const LabelMap& getLabelMap() {
        return labelMap;
    }
    static const LabelMapInverse& getInverseLabelMap() {
        return labelMapInverse;
    }
    static const LogicalTopology& getLogicalTopologyMap() {
        return logicalTopology;
    }
    static const Topology& getTopologyMap() {
        return topology;
    }
    static const ConnectedL3Address& getL3Address() {
        return connectedL3Address;
    }
    static MacAddress searchL3Address(const L3Address&);

    static LabelMap& getLabelMapForUpdate() {
        return labelMap;
    }
    static LogicalTopology& getLogicalTopologyMapForUpdate() {
        return logicalTopology;
    }
    static Topology& getTopologyMapForUpdate() {
        return topology;
    }
    static ConnectedL3Address& getL3AddressForUpdate() {
        return connectedL3Address;
    }

protected:
    static simsignal_t updateTopology;

    virtual int numInitStages() const override {
        return NUM_INIT_STAGES;
    }
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void computeRoutes();
public:
    virtual void receiveSignal(cComponent *source, simsignal_t signal, cObject *object, cObject *details) override;
};

}
}

#endif
