/*
 * GraphUtil.cpp
 *
 *  Created on: Sep 19, 2024
 *      Author: bman
 */

#include "LMPR.h"

namespace inet
{

NodeInfo* LMPR::getNodeInfoByAddr(Ipv4Address addr)
{
    for(auto it = nodesInfoList.begin(); it != nodesInfoList.end(); ++ it)
    {
        if((*it)->addr == addr)
            return (*it);
    }
    return nullptr;
}

int LMPR::getTargetNodeIdxByAddr(Ipv4Address addr)
{
    for(int i = 0; i < N; i ++)
     {
         if(_globalMob[i].first == addr)
         {
             return i;
         }
     }
}

void LMPR::setDownControlInfo(Packet *const pMsg, const MacAddress& pDestAddr)
{
    pMsg->addTagIfAbsent<MacAddressReq>()->setDestAddress(pDestAddr);
    pMsg->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&getProtocol());
    pMsg->addTagIfAbsent<DispatchProtocolInd>()->setProtocol(&getProtocol());
}


}
