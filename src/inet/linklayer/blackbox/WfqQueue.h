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

#ifndef INET_LINKLAYER_BLACKBOX_WFQQUEUE_H_
#define INET_LINKLAYER_BLACKBOX_WFQQUEUE_H_

#include <omnetpp/cownedobject.h>
#include <deque>
#include <map>
#include "inet/common/packet/Packet.h"


namespace inet {
namespace blackbox {

class WfqQueue: public omnetpp::cOwnedObject {
    struct QueuePacket {
        double endTime;
        Packet *pkt;

        QueuePacket(double endTime, Packet *pkt):endTime(endTime), pkt(pkt){}
        QueuePacket& operator=(const QueuePacket &other) {
            this->endTime = other.endTime;
            this->pkt = other.pkt;
            return *this;
        }
    };
    typedef std::deque<QueuePacket> PacketQueue;
    struct QueueData {
        simtime_t lastAccess;
        PacketQueue queue;
        double weight = 0;
        bps virtualR = bps(0);
    };
    bps bandwidth = bps(0);
    std::map<uint64_t, QueueData> queues;
    std::map<uint64_t, QueueData> queuesDeleted;
    QueueData bestEffortQueue;
    std::tuple<int64_t, double> getMinQueue();
    std::tuple<int64_t, double> getMinQueueDeleted();

public:
    WfqQueue(bps);
    virtual ~WfqQueue();
    virtual unsigned int getNumQueues() {return queues.size() + 1;}
    virtual unsigned int getTotalPackets() const;
    virtual unsigned int getLength() const {return getTotalPackets();}
    virtual bool isEmpty() const;
    virtual bool addQueue(const uint64_t &label,const double &wg);
    virtual bool removeQueue(const uint64_t &label);
    virtual bool removeQueueLater(const uint64_t &label);
    virtual Packet * getPaket();
    virtual Packet * getPaket(const uint64_t &label);
    virtual bool addPacket(Packet *, const uint64_t &label);
    virtual bool addPacket(Packet *);
    virtual bool queueExist(const uint64_t &label) const;
};
}
} /* namespace inet */

#endif /* INET_LINKLAYER_BLACKBOX_WFQQUEUE_H_ */
