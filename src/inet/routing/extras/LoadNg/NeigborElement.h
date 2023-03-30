/*
 * NeigborElement.h
 *
 *  Created on: Dec 19, 2019
 *      Author: alfonso
 */

#ifndef INET_ROUTING_EXTRAS_LOADNG_NEIGBORELEMENT_H_
#define INET_ROUTING_EXTRAS_LOADNG_NEIGBORELEMENT_H_
#include "inet/common/INETDefs.h"
#include <map>
#include <deque>

namespace inet {
namespace inetmanet {

class NodeStatus {
public:
    bool isBidirectional = false;
    bool pendingConfirmation = false; // the latest notification has failed.
    int64_t seqNum = -1;
    double metric;
    uint8_t numHelloRec;
    inet::simtime_t delay;
    //double power;
    double recPower = NaN;
    double energy = NaN;
    double snir = NaN;
    int numNeig = std::numeric_limits<int>::quiet_NaN();
    bool stationary = false;
};

class NeigborElement {
public:
    inet::simtime_t lastNotification;
    inet::simtime_t creationTime;
    inet::simtime_t lifeTime;
    int64_t seqNumber = -1;
    uint64_t helloIdentify;
    bool stationary = false;
    bool isBidirectional = false;
    bool pendingConfirmation = false; // the latest notification has failed.
    bool pendingHello = true;
    std::map<inet::L3Address, NodeStatus> listNeigbours;
    std::deque<inet::simtime_t> helloTime;
    double metricToNeig = 255;
    int numHelloRec = 0;

    std::deque<inet::simtime_t> delayList;
    std::deque<double> powerList;
    std::deque<double> snirList;

    double energy = NaN;
    double powerRecCost = NaN;
    double delayCost = NaN;
    double snirCost = NaN;

    int32_t distRoot = -1;
    int32_t metricToRoot = -1;
};
typedef std::map<inet::L3Address, NeigborElement> NeighborsMap;

} // namespace aodv
} // namespace inet

#endif /* INET_ROUTING_EXTRAS_LOADNG_NEIGBORELEMENT_H_ */
