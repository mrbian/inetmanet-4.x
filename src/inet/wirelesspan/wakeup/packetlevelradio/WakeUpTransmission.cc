//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpTransmission.h"

namespace inet {

namespace physicallayer {

WakeUpTransmission::WakeUpTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime, const simtime_t endTime, const simtime_t preambleDuration, const simtime_t headerDuration, const simtime_t dataDuration, const Coord startPosition, const Coord endPosition, const Quaternion startOrientation, const Quaternion endOrientation, const ITransmissionPacketModel *packetModel, const ITransmissionBitModel *bitModel, const ITransmissionSymbolModel *symbolModel, const ITransmissionSampleModel *sampleModel, const ITransmissionAnalogModel *analogModel, W power, Hz centralFrequency, Hz bandwidth) :
    TransmissionBase(transmitter, packet, startTime, endTime, preambleDuration, headerDuration, dataDuration, startPosition, endPosition, startOrientation, endOrientation, packetModel, bitModel, symbolModel, sampleModel, analogModel),
    power(power), centralFrequency(centralFrequency), bandwidth(bandwidth)
{
}

std::ostream& WakeUpTransmission::printToStream(std::ostream& stream, int level, int evFlags) const
{
    stream << "WakeUpTransmission";
    return TransmissionBase::printToStream(stream, level);
}

} // namespace physicallayer

} // namespace inet

