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
#include <limits>
#include "WfqQueue.h"

namespace inet {
namespace blackbox {

WfqQueue::WfqQueue(bps bandwidth):
        bandwidth(bandwidth)
{
    // TODO Auto-generated constructor stub
    bestEffortQueue.weight = 1.0;
    bestEffortQueue.virtualR = bandwidth;
}

WfqQueue::~WfqQueue() {
    // TODO Auto-generated destructor stub
    for (auto &elem : queues) {
        while(!elem.second.queue.empty()) {
            delete elem.second.queue.back().pkt;
            elem.second.queue.pop_back();
        }
    }
    queues.clear();
    for (auto &elem : queuesDeleted) {
        while(!elem.second.queue.empty()) {
            delete elem.second.queue.back().pkt;
            elem.second.queue.pop_back();
        }
    }
    queuesDeleted.clear();
    while (!bestEffortQueue.queue.empty()) {
        delete bestEffortQueue.queue.back().pkt;
        bestEffortQueue.queue.pop_back();
    }
}

bool WfqQueue::isEmpty() const
{
    if (!bestEffortQueue.queue.empty())
        return false;
    for (const auto &elem : queues) {
        if (!elem.second.queue.empty())
            return false;
    }
    for (const auto &elem : queuesDeleted) {
        if (!elem.second.queue.empty())
            return false;
    }
    return true;
}

unsigned int WfqQueue::getTotalPackets() const
{
    unsigned int s = bestEffortQueue.queue.size();
    for (const auto &elem : queues) {
        s += elem.second.queue.size();
    }
    for (const auto &elem : queuesDeleted) {
        s += elem.second.queue.size();
    }
    return s;
}

bool WfqQueue::addQueue(const uint64_t &label, const double &wg)
{
    auto it = queues.find(label);
    if (it != queues.end())
        return false;
    QueueData qData;
    qData.weight = wg;

    queues[label] = qData;
    double w = 0;
    for (const auto &elem : queues) {
        w += elem.second.weight;
    }
    if (w >=1.0)
        throw cRuntimeError("WfqQueue weights error");
    bestEffortQueue.weight = 1 - w;
    // compute virtualR
    for (auto &elem : queues) {
        elem.second.virtualR = bandwidth * elem.second.weight;
    }
    bestEffortQueue.virtualR = bandwidth * bestEffortQueue.weight;
    return true;
}

bool WfqQueue::removeQueue(const uint64_t &label)
{
    auto it = queues.find(label);
    if (it == queues.end())
        return false;
    while (!it->second.queue.empty()) {
        delete  it->second.queue.back().pkt;
        it->second.queue.pop_back();
    }
    queues.erase(it);
    double w = 0;
    for (const auto &elem : queues) {
        w += elem.second.weight;
    }
    bestEffortQueue.weight = 1 - w;
    // compute virtualR
    for (auto &elem : queues) {
        elem.second.virtualR = bandwidth * elem.second.weight;
    }
    bestEffortQueue.virtualR = bandwidth * bestEffortQueue.weight;
    return true;
}

// mark the queue to delete when the queue is empty
bool WfqQueue::removeQueueLater(const uint64_t &label)
{
    auto it = queues.find(label);
    if (it == queues.end())
        return false;
    // move to the deleted queue
    if (!it->second.queue.empty()) {
        // move the queue
        auto itAux = queuesDeleted.find(label);
        if (itAux != queuesDeleted.end()) {
            while (!itAux->second.queue.empty()) {
                delete  itAux->second.queue.back().pkt;
                itAux->second.queue.pop_back();
            }
        }
        queuesDeleted[label] = it->second;
    }
    queues.erase(it);
    double w = 0;
    for (const auto &elem : queues) {
        w += elem.second.weight;
    }
    bestEffortQueue.weight = 1 - w;
    // compute virtualR
    for (auto &elem : queues) {
        elem.second.virtualR = bandwidth * elem.second.weight;
    }
    bestEffortQueue.virtualR = bandwidth * bestEffortQueue.weight;
    return true;
}


std::tuple<int64_t, double> WfqQueue::getMinQueue()
{
    double min = std::numeric_limits<double>::max();
    int q = -2;
    if (!bestEffortQueue.queue.empty()) {
        min = bestEffortQueue.queue.front().endTime;
        q = -1;
    }
    for (const auto &elem : queues) {
        if (!elem.second.queue.empty()  && min > elem.second.queue.front().endTime) {
            min = elem.second.queue.front().endTime;
            q = elem.first;
        }
    }
    return std::tuple<int, double>(q,min);
}

std::tuple<int64_t, double> WfqQueue::getMinQueueDeleted()
{
    if (queuesDeleted.empty()) // most common case
        return std::tuple<int, double>(-2, std::numeric_limits<double>::max());

    double min = std::numeric_limits<double>::max();
    int q = -2;
    bool checkQueues = false;
    for (const auto &elem : queuesDeleted) {
        if (elem.second.queue.empty())
            checkQueues = true; // found empty queue in the queuesDeleted list,
        if (!elem.second.queue.empty()  && min > elem.second.queue.front().endTime) {
            min = elem.second.queue.front().endTime;
            q = elem.first;
        }
    }

    if (checkQueues) {
        // force erase empty queues in queuesDeleted list
        for (auto it = queuesDeleted.begin(); it != queuesDeleted.end();) {
            if (it->second.queue.empty())
                queuesDeleted.erase(it++);
            else
                ++it;
        }
    }
    return std::tuple<int, double>(q,min);
}

Packet * WfqQueue::getPaket()
{
    auto tup = getMinQueue();
    auto q = std::get<0>(tup);
    Packet *pkt = nullptr;
    if (-2 == q && queuesDeleted.empty())
        return pkt; // no packet available

    // check deleted queues
    if (!queuesDeleted.empty()) {
        auto tup2 = getMinQueueDeleted();
        auto q2 = std::get<0>(tup2);
        if (-2 == q) { // only exist packets in the delted queue
            auto it = queuesDeleted.find(q2);
            pkt = it->second.queue.front().pkt;
            it->second.queue.pop_front();
            if (it->second.queue.empty()) // if the queue is empty, erase it
                queuesDeleted.erase(it);
            return pkt;
        }
        // packets in the deleted and normal
        else {
            // packets in deleted and normal queues
            // check min
            auto min1 = std::get<1>(tup);
            auto min2 = std::get<1>(tup2);
            if (min2 < min1) {
                auto it = queuesDeleted.find(q2);
                pkt = it->second.queue.front().pkt;
                it->second.queue.pop_front();
                if (it->second.queue.empty()) // if the queue is empty, erase it
                    queuesDeleted.erase(it);
                return pkt;
            }
        }
    }

    if (-1 == q) {
        // best effort
        pkt =  bestEffortQueue.queue.front().pkt;
        bestEffortQueue.queue.pop_front();
        // compute next timer
    }
    else {
        auto it = queues.find(q);
        pkt = it->second.queue.front().pkt;
        it->second.queue.pop_front();
    }
    return pkt;
}

Packet * WfqQueue::getPaket(const uint64_t &label)
{
    Packet *pkt = nullptr;
    auto it = queues.find(label);
    if (it != queues.end()) {
        pkt = it->second.queue.front().pkt;
        it->second.queue.pop_front();
    }
    return pkt;
}

bool WfqQueue::addPacket(Packet *pkt, const uint64_t &label)
{
    auto it = queues.find(label);
    if (it != queues.end()) {
        it->second.lastAccess = simTime();
        auto tup = getMinQueue();
        auto q = std::get<0>(tup);
        if (q != -2) {// if q == -2 all the queues are empty, cf = 0
            double vtime = pkt->getBitLength() * it->second.virtualR.get();
            it->second.queue.push_back(QueuePacket(vtime, pkt));
        }
        else {
            auto cf = std::get<1>(tup);
            auto vtime = cf;
            if (!it->second.queue.empty()) {
                vtime = std::max(it->second.queue.back().endTime, cf);
            }
            vtime += pkt->getBitLength() * it->second.virtualR.get();
            it->second.queue.push_back(QueuePacket(vtime, pkt));
        }
        return true;
    }

    else {
        // now search in the pendings queues
        auto it = queuesDeleted.find(label);
        if (it != queues.end()) {
            it->second.lastAccess = simTime();
            double vtime = pkt->getBitLength() * it->second.virtualR.get();
            it->second.queue.push_back(QueuePacket(vtime, pkt));
        }
    }
    // now check queuesDeleted
    for (auto it = queuesDeleted.begin(); it != queuesDeleted.end(); ) {
        if (it->second.queue.empty() && simTime() - it->second.lastAccess  > 2) {
            queuesDeleted.erase(it++);
        }
        else
            ++it;
    }
    return false;
}

bool WfqQueue::addPacket(Packet *pkt)
{
    auto tup = getMinQueue();
    auto q = std::get<0>(tup);
    if (q != -2) {// if q == -2 all the queues are empty, cf = 0
        double vtime = pkt->getBitLength() * bestEffortQueue.virtualR.get();
        bestEffortQueue.queue.push_back(QueuePacket(vtime, pkt));
    }
    else {
        auto cf = std::get<1>(tup);
        auto vtime = cf;
        if (!bestEffortQueue.queue.empty()) {
            vtime = std::max(bestEffortQueue.queue.back().endTime, cf);
        }
        vtime += pkt->getBitLength() * bestEffortQueue.virtualR.get();
        bestEffortQueue.queue.push_back(QueuePacket(vtime, pkt));
    }
    // now check queuesDeleted
    for (auto it = queuesDeleted.begin(); it != queuesDeleted.end(); ) {
        if (it->second.queue.empty() && simTime() - it->second.lastAccess  > 2) {
            queuesDeleted.erase(it++);
        }
        else
            ++it;
    }
    return true;
}

bool WfqQueue::queueExist(const uint64_t &label) const
{
    auto it = queues.find(label);
    return it != queues.end();
}

}
} /* namespace inet */
