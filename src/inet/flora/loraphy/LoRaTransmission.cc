/*
 * LoRaTransmission.cc
 *
 *  Created on: Feb 17, 2017
 *      Author: slabicm1
 */

#include "inet/flora/loraphy/LoRaTransmission.h"

namespace inet {
namespace flora {
LoRaTransmission::LoRaTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime, const simtime_t endTime, const simtime_t preambleDuration, const simtime_t headerDuration, const simtime_t dataDuration, const Coord startPosition, const Coord endPosition, const Quaternion startOrientation, const Quaternion endOrientation, const ITransmissionPacketModel *packetModel, const ITransmissionBitModel *bitModel, const ITransmissionSymbolModel *symbolModel, const ITransmissionSampleModel *sampleModel, const ITransmissionAnalogModel *analogModel, W LoRaTP, Hz LoRaCF, Hz LoRaBW, int LoRaSF, int LoRaCR) :
        TransmissionBase(transmitter, packet, startTime, endTime, preambleDuration, headerDuration, dataDuration, startPosition, endPosition, startOrientation, endOrientation, packetModel, bitModel, symbolModel, sampleModel, analogModel),
        LoRaTP(LoRaTP),
        LoRaCF(LoRaCF),
        LoRaBW(LoRaBW),
        LoRaSF(LoRaSF),
        LoRaCR(LoRaCR)
{
    // TODO Auto-generated constructor stub

}

std::ostream& LoRaTransmission::printToStream(std::ostream& stream, int level, int evFlags) const
{
    return TransmissionBase::printToStream(stream, level);
}


} /* namespace physicallayer */
} /* namespace inet */
