//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_WAKEUPTRANSMISSION_H
#define __INET_WAKEUPTRANSMISSION_H

#include "inet/physicallayer/wireless/common/base/packetlevel/TransmissionBase.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"

namespace inet {

namespace physicallayer {

class INET_API WakeUpTransmission : public TransmissionBase
{
  protected:

    const W power = W(-1);
    const Hz centralFrequency = Hz(NaN);
    const Hz bandwidth = Hz(NaN);
  public:
    WakeUpTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime, const simtime_t endTime, const simtime_t preambleDuration, const simtime_t headerDuration, const simtime_t dataDuration, const Coord startPosition, const Coord endPosition, const Quaternion startOrientation, const Quaternion endOrientation, const ITransmissionPacketModel *packetModel, const ITransmissionBitModel *bitModel, const ITransmissionSymbolModel *symbolModel, const ITransmissionSampleModel *sampleModel, const ITransmissionAnalogModel *analogModel, W power, Hz centralFrequency, Hz bandwidth);
    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
    virtual W getTransmissionPower() const { return power; }
    virtual const Hz getCenterFrequency() const { return centralFrequency; }
    virtual const Hz getBandwidth() const { return bandwidth; }
};

} // namespace physicallayer

} // namespace inet

#endif

