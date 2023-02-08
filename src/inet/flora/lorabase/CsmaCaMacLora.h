//
// Copyright (C) 2016 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_CSMACAMACLORA_H
#define __INET_CSMACAMACLORA_H

#include "inet/common/FSMA.h"
#include "inet/common/packet/Packet.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/contract/IMacProtocol.h"
#include "inet/linklayer/csmaca/CsmaCaMacHeader_m.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"
#include "inet/queueing/contract/IActivePacketSink.h"
#include "inet/queueing/contract/IPacketQueue.h"
#include "inet/linklayer/csmaca/CsmaCaMac.h"

namespace inet {
namespace flora {


class INET_API CsmaCaMacLora : public inet::CsmaCaMac
{
protected:
    cModule *simpleApp = nullptr;
    uint64_t sequenceNumber = 0;
  public:
    /**
     * @name Construction functions
     */
    //@{
    //virtual ~CsmaCaMacLora();

    //@}
  protected:
    virtual void initialize(int stage) override;
    virtual void handleUpperPacket(Packet *packet) override;
    virtual bps computeBitRate(Packet *packet);
    virtual void encapsulate(Packet *frame) override;
    virtual void decapsulate(Packet *frame)  override;
    virtual bool isForUs(Packet *frame) override;
    virtual bool isBroadcast(Packet *frame) override;
    virtual bool isAck(Packet *frame) override;
    virtual void generateBackoffPeriod() override;
    virtual bool isFcsOk(Packet *frame) override;

};

}
} // namespace inet

#endif

