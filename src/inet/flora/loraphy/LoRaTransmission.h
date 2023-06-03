/*
 * LoRaTransmission.h
 *
 *  Created on: Feb 17, 2017
 *      Author: slabicm1
 */

#ifndef LORATRANSMISSION_H_
#define LORATRANSMISSION_H_

#include "inet/physicallayer/wireless/common/base/packetlevel/TransmissionBase.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadioSignal.h"

namespace inet {
namespace flora {

using namespace physicallayer;

class INET_API LoRaTransmission : public TransmissionBase
{
protected:
    W LoRaTP;
    Hz LoRaCF;
    Hz LoRaBW;

    int LoRaSF;
    int LoRaCR;
public:

    LoRaTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime, const simtime_t endTime, const simtime_t preambleDuration, const simtime_t headerDuration, const simtime_t dataDuration, const Coord startPosition, const Coord endPosition, const Quaternion startOrientation, const Quaternion endOrientation, const ITransmissionPacketModel *packetModel, const ITransmissionBitModel *bitModel, const ITransmissionSymbolModel *symbolModel, const ITransmissionSampleModel *sampleModel, const ITransmissionAnalogModel *analogModel, W LoRaTP, Hz LoRaCF, Hz LoRaBW, int LoRaSF, int LoRaCR);
    virtual Hz getCenterFrequency() const  { return LoRaCF; }
    virtual Hz getBandwidth() const  { return LoRaBW; }
    virtual W getPower() const  { return LoRaTP; }
    virtual W computeMinPower(const simtime_t startTime, const simtime_t endTime) const  { return LoRaTP; }

    W getLoRaTP() const { return LoRaTP; }
    Hz getLoRaCF() const { return LoRaCF; }
    int getLoRaSF() const { return LoRaSF; }
    Hz getLoRaBW() const { return LoRaBW; }
    int getLoRaCR() const { return LoRaCR; }

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
};

} /* namespace physicallayer */
} /* namespace inet */

#endif /* LORATRANSMISSION_H_ */
