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

#include "LMPR.h"

namespace inet {

void LMPR::handleOGMReminder()
{
//    broadcastOGM();
    scheduleAt(simTime() + mhOGMInterval + broadcastDelay->doubleValue(), OGMReminder);

    int len = nodesInfoList.size();
    emit(nodeInfoLenSignal, len);
}

void LMPR::broadcastOGM()
{
    auto ogm = makeShared<OGM>();

    Ipv4Address origin = (interface80211ptr->getProtocolData<Ipv4InterfaceData>()->getIPAddress());
    ogm->setOrigin(origin);
    m_OGM_seqNum += 1;
    ogm->setSeq(m_OGM_seqNum);
    ogm->setTTL(defaultTTL);
    ogm->setTimeAbstract(simTime().dbl());
    Coord pos1 = forecastSelfPosition_Optimal(predictDuration);
    ogm->setX1(pos1.x);
    ogm->setY1(pos1.y);
    Coord pos2 = forecastSelfPosition_Optimal(predictDuration*2);
    ogm->setX2(pos2.x);
    ogm->setY2(pos2.y);
    Coord pos3 = forecastSelfPosition_Optimal(predictDuration*3);
    ogm->setX3(pos3.x);
    ogm->setY3(pos3.y);

    int totalByteLength = sizeof(m_selfIpv4Address)*2
            + sizeof(m_OGM_seqNum)
            + 6 * sizeof((float)ogm->getX1())
            + sizeof((double)(simTime().dbl()))
            + sizeof((int)defaultTTL);
    ogm->setChunkLength(B(totalByteLength));

    std::ostringstream str;
    str << "OGM" << "-" << m_OGM_seqNum;
    auto packet = new Packet(str.str().c_str(), ogm);
    auto originTag = packet->addTagIfAbsent<OriginatorTag>();
    originTag->setName("OGM");
    originTag->setIsOrigin(true);
    setDownControlInfo(packet, MacAddress::BROADCAST_ADDRESS);
    sendDown(packet);
    packet = nullptr;
    ogm = nullptr;
}


void LMPR::handleOGM(Ptr<OGM> ogm, int64_t len)
{
    if(notBroadcasted(ogm)) {
        // store trajectory information
        NodeInfo* node_info = getNodeInfoByAddr(ogm->getOrigin());
        if(!node_info)
        {
            node_info = new NodeInfo();
            node_info->addr = ogm->getOrigin();
            node_info->last_bcast_seqno = ogm->getSeq();
            nodesInfoList.push_back(node_info);
        }

        if(node_info->last_bcast_seqno <= ogm->getSeq())
        {
            node_info->last_seen = simTime();
            node_info->time_abstract = ogm->getTimeAbstract();
            node_info->trs.x1 = ogm->getX1();
            node_info->trs.y1 = ogm->getY1();
            node_info->trs.x2 = ogm->getX2();
            node_info->trs.y2 = ogm->getY2();
            node_info->trs.x3 = ogm->getX3();
            node_info->trs.y3 = ogm->getY3();
        }

        // forward
        if(ogm->getTTL() >= 1)
        {
            ogm->setTTL(ogm->getTTL() - 1);
            auto packet = new Packet("OGM", ogm);
            setDownControlInfo(packet, MacAddress::BROADCAST_ADDRESS);
            sendDown(packet);
            packet = nullptr;
            ogm = nullptr;
        }
    }
}

bool LMPR::notBroadcasted(const Ptr<OGM> ogm)
{
    // serach the broadcast list of outdated entries and delete them
    for (auto it = bcMsgs.begin(); it != bcMsgs.end();) {
        if (it->delTime < simTime()) {
            it = bcMsgs.erase(it);
        }
        // message was already broadcasted
        else if ((it->origAddr == ogm->getOrigin()) && (it->seqNum == ogm->getSeq())) {
            // update entry
            it->delTime = simTime() + bcDelTime;
            return false;
        }
        else
            ++it;
    }

    // delete oldest entry if max size is reached
    if (bcMsgs.size() >= bcMaxEntries) {
        EV << "bcMsgs is full, delete oldest entry\n";
        bcMsgs.pop_front();
    }

    bcMsgs.push_back(Bcast(ogm->getSeq(), ogm->getOrigin(), simTime() + bcDelTime));
    return true;
}

//    packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(
//            Ipv4Address(255, 255, 255, 255));
//    packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(m_selfIpv4Address);
//    packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(interface80211ptr->getInterfaceId());

//    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::manet);
//    packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);
//    sendDelayed(packet, broadcastDelay->doubleValue(), "ipOut");


//    packet->addTagIfAbsent<L3AddressReq>()->setDestAddress(Ipv4Address(255, 255, 255, 255));
//    packet->addTagIfAbsent<L3AddressReq>()->setSrcAddress(m_selfIpv4Address);
//    packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(interface80211ptr->getInterfaceId());
//    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::ipv4);
//    packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::);
//    sendDelayed(packet, broadcastDelay->doubleValue(), "ipOut");

} /* namespace inet */
