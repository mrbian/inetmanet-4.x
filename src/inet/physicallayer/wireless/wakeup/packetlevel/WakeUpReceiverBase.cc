//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/physicallayer/wireless/wakeup/packetlevel/WakeUpReceiverBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/NarrowbandReceptionBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/NarrowbandTransmissionBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatReceiverBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatReceptionBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatTransmissionBase.h"

namespace inet {

namespace physicallayer {

Define_Module(WakeUpReceiverBase);

WakeUpReceiverBase::WakeUpReceiverBase() :
    FlatReceiverBase()
{
}

const IListening *WakeUpReceiverBase::createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord& startPosition, const Coord& endPosition) const
{
    return new WakeUpBandListening(radio, startTime, endTime, startPosition, endPosition, centerFrequency, bandwidth, bandwithList);
}


bool WakeUpReceiverBase::computeIsReceptionPossible(const IListening *listening, const ITransmission *transmission) const
{
    // TODO check if modulation matches?

    const NarrowbandTransmissionBase *narrowbandTransmission = check_and_cast<const NarrowbandTransmissionBase *>(transmission);
    if (!bandwithList.empty()) {
        for (const auto &e : bandwithList) {
            if (e.getCenterFrequency() == narrowbandTransmission->getCenterFrequency() && e.getBandwidth() >= narrowbandTransmission->getBandwidth()) {
                return true;
            }
        }
        return false;
    }
    else {
        return centerFrequency == narrowbandTransmission->getCenterFrequency() && bandwidth >= narrowbandTransmission->getBandwidth();
    }
}


bool WakeUpReceiverBase::computeIsReceptionPossible(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part) const
{
    const WakeUpBandListening *bandListening = check_and_cast<const WakeUpBandListening *>(listening);
    const NarrowbandReceptionBase *narrowbandReception = check_and_cast<const NarrowbandReceptionBase *>(reception);
    auto list = bandListening->getBandList();
    bool itIsPossible = false;
    if (!list.empty()) {
        for (const auto &e : list) {
            if (e.getCenterFrequency() == narrowbandReception->getCenterFrequency() && e.getBandwidth() >= narrowbandReception->getBandwidth()) {
                itIsPossible = true;
            }
        }
    }
    else {
        if (bandListening->getCenterFrequency() == narrowbandReception->getCenterFrequency() && bandListening->getBandwidth() >= narrowbandReception->getBandwidth()) {
            itIsPossible = true;
        }
    }
    if (!itIsPossible)
        return false;

    const FlatReceptionBase *flatReception = check_and_cast<const FlatReceptionBase *>(reception);
    W minReceptionPower = flatReception->computeMinPower(reception->getStartTime(part), reception->getEndTime(part));
    ASSERT(W(0.0) <= minReceptionPower);
    bool isReceptionPossible = minReceptionPower >= sensitivity;
    EV_DEBUG << "Computing whether reception is possible" << EV_FIELD(minReceptionPower) << EV_FIELD(sensitivity) << " -> reception is " << (isReceptionPossible ? "possible" : "impossible") << endl;
    return isReceptionPossible;
}

void WakeUpReceiverBase::initialize(int stage)
{
    FlatReceiverBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        minInterferencePower = mW(math::dBmW2mW(par("minInterferencePower")));
    }
}

std::ostream& WakeUpReceiverBase::printToStream(std::ostream& stream, int level, int evFlags) const
{
    stream << "WakeUpScalarReceiver";
    return FlatReceiverBase::printToStream(stream, level);
}

} // namespace physicallayer

} // namespace inet

