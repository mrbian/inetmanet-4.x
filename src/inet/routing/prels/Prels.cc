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

#include "Prels.h"

namespace inet {

Define_Module(Prels);

void Prels::initialize(int stage) {
    NetworkProtocolBase::initialize(stage);

    if(stage == INITSTAGE_LOCAL) {
        // Routing
        hello_ival_ = &par("Hello_ival");
        tc_ival_ = &par("Tc_ival");
        ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        hello_reminder = new cMessage("hello_reminder");
        tc_reminder = new cMessage("tc_reminder");
        send_reminder = new cMessage("send_reminder");

        state_ptr = new Prels_state();
        helloTimer = new Prels_HelloTimer();
        tcTimer = new Prels_TcTimer();
    }
    else if(stage == INITSTAGE_NETWORK_LAYER) {
        mobility = check_and_cast<IMobility*>(getContainingNode(this)->getSubmodule("mobility"));
        arp = check_and_cast<GlobalArp*>(this->getParentModule()->getSubmodule("arp"));

        cModule* topModule = getModuleByPath("Net80211_aodv");
        int numRouters = topModule->par("numRouters");
        int numFixHosts = topModule->par("numFixHosts");
        char mob_path_str[100];
        char node_path_str[100];
        for(int i = 0; i < numRouters; i ++)
        {
            sprintf(mob_path_str, "Net80211_aodv.router[%d].mobility", i);
            sprintf(node_path_str, "Net80211_aodv.router[%d]", i);
            IMobility* mob = check_and_cast<IMobility *>(getModuleByPath(mob_path_str));
            NodeBase *node = check_and_cast<NodeBase*>(getModuleByPath(node_path_str));
            _globalMob.insert(std::make_pair(node->getId(), mob));
        }
        for (int i = 0; i < numFixHosts; i ++)
        {
            sprintf(mob_path_str, "Net80211_aodv.fixhost[%d].mobility", i);
            sprintf(node_path_str, "Net80211_aodv.fixhost[%d]", i);
            IMobility* mob = check_and_cast<IMobility *>(getModuleByPath(mob_path_str));
            NodeBase *node = check_and_cast<NodeBase*>(getModuleByPath(node_path_str));
            _globalMob.insert(std::make_pair(node->getId(), mob));
        }

        self_node_id = getContainingNode(this)->getId();
    }
    else
    {
        if(stage != 1)
            EV << stage << std::endl;
    }

}

// called after INITSTAGE_LOCAL
void Prels::start()
{
    /* Search the 80211 interface */
    int num_80211 = 0;
    NetworkInterface *ie;
    NetworkInterface *i_face;
    const char *name;
    for (int i = 0; i < ift->getNumInterfaces(); i++) {
        ie = ift->getInterface(i);
        name = ie->getInterfaceName();
        if (strstr(name, "wlan") != nullptr) {
            i_face = ie;
            num_80211++;
            interfaceId = i;
        }
    }

// One enabled network interface (in total)
    if (num_80211 == 1) {
        interface80211ptr = i_face;
    }
    else {
        throw cRuntimeError("Router has found %i 80211 interfaces", num_80211);
    }

    m_selfIpv4Address = interface80211ptr->getProtocolData<Ipv4InterfaceData>()->getIPAddress();

    createTimerQueue();
    scheduleEvent();
}

void Prels::stop()
{
    scheduleEvent();
}

void Prels::finish()
{
}

void
Prels::setDownControlInfo(Packet *const pMsg, const MacAddress& pDestAddr)
{
    pMsg->addTagIfAbsent<MacAddressReq>()->setDestAddress(pDestAddr);
    pMsg->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&getProtocol());
    pMsg->addTagIfAbsent<DispatchProtocolInd>()->setProtocol(&getProtocol());
}

void
Prels::handleUpperPacket(Packet *packet)
{
    // application data packet
}

void
Prels::handleLowerPacket(Packet *packet)
{
    // hello, tc
    if(strstr(packet->getName(), "PRELS Pkt") != nullptr)
    {
        recv_prels(packet);
    }
    // data
//    else
//    {
//
//    }
}

void
Prels::handleSelfMessage(cMessage *msg)
{
    checkTimer(msg);
    scheduleEvent();  // 调度队列（按时间排序）的下一个timer
}


void Prels::handleStartOperation(LifecycleOperation *operation)
{
    start();
    helloTimer->resched(hello_ival());
    tcTimer->resched(hello_ival());
    scheduleEvent();
}


Prels::~Prels() {
    // TODO Auto-generated destructor stub
}

void Prels::createTimerQueue()
{
    if (timerMessagePtr == nullptr)
        timerMessagePtr = new cMessage("Prels TimerQueue");
    if (timerMultiMapPtr == nullptr)
        timerMultiMapPtr = new TimerMultiMap;
}

void Prels::scheduleEvent()
{
    if (!timerMessagePtr)
        return;
    if (!timerMultiMapPtr)
        return;

    if (timerMultiMapPtr->empty()) { // nothing to do
        if (timerMessagePtr->isScheduled())
            cancelEvent(timerMessagePtr);
        return;
    }

    simtime_t now = simTime();
    while (!timerMultiMapPtr->empty() && timerMultiMapPtr->begin()->first <= now) {
        auto e = timerMultiMapPtr->begin();
        Prels_Timer * timer = e->second;
        timerMultiMapPtr->erase(e);
        timer->expire();
    }

    auto e = timerMultiMapPtr->begin();
    if (timerMessagePtr->isScheduled()) {
        if (e->first < timerMessagePtr->getArrivalTime())  {
            cancelEvent(timerMessagePtr);
            scheduleAt(e->first, timerMessagePtr);
        }
        else if (e->first>timerMessagePtr->getArrivalTime()) { // Possible throw cRuntimeError, or the first event has been canceled
            cancelEvent(timerMessagePtr);
            scheduleAt(e->first, timerMessagePtr);
            EV << "timer Queue problem";
            // throw cRuntimeError("timer Queue problem");
        }
    }
    else {
        scheduleAt(e->first, timerMessagePtr);
    }
}

bool Prels::checkTimer(cMessage *msg)
{
    if (msg != timerMessagePtr)
        return false;
    if (timerMessagePtr == nullptr)
        throw cRuntimeError("Prels::checkTimer throw cRuntimeError timerMessagePtr doens't exist");
    if (timerMultiMapPtr->empty())
        return true;

    while (timerMultiMapPtr->begin()->first <= simTime())
    {
        auto it = timerMultiMapPtr->begin();
        Prels_Timer * timer = it->second;
        if (timer == nullptr)
            throw cRuntimeError ("timer owner is bad");
        timerMultiMapPtr->erase(it);
        timer->expire();
        if (timerMultiMapPtr->empty())
            break;
    }
    return true;
}


} /* namespace inet */



