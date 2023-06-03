//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpReceiverBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatReceiverBase.h"
#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpTransmission.h"
#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpTransmission.h"
#include "inet/physicallayer/wireless/common/analogmodel/scalar/ScalarReceptionAnalogModel.h"
#include "inet/physicallayer/wireless/common/analogmodel/scalar/ScalarMediumAnalogModel.h"
#include "inet/physicallayer/wireless/common/analogmodel/scalar/ScalarReceptionAnalogModel.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

using namespace inet::physicallayer;

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

    const WakeUpTransmission *wakeUpTransmission = check_and_cast<const WakeUpTransmission *>(transmission);
    if (!bandwithList.empty()) {
        for (const auto &e : bandwithList) {
            if (e.getCenterFrequency() == wakeUpTransmission->getCenterFrequency() && e.getBandwidth() >= wakeUpTransmission->getBandwidth()) {
                return true;
            }
        }
        return false;
    }
    else {
        return centerFrequency == wakeUpTransmission->getCenterFrequency() && bandwidth >= wakeUpTransmission->getBandwidth();
    }
}


bool WakeUpReceiverBase::computeIsReceptionPossible(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part) const
{
    const WakeUpBandListening *bandListening = check_and_cast<const WakeUpBandListening *>(listening);
    auto wakeUpTransmission = dynamic_cast<const WakeUpTransmission *>(reception->getTransmission());
    if (wakeUpTransmission == nullptr)
        return false;

    auto list = bandListening->getBandList();
    bool itIsPossible = false;
    if (!list.empty()) {
        for (const auto &e : list) {
            if (e.getCenterFrequency() == wakeUpTransmission->getCenterFrequency() && e.getBandwidth() >= wakeUpTransmission->getBandwidth()) {
                itIsPossible = true;
            }
        }
    }
    else {
        if (bandListening->getCenterFrequency() == wakeUpTransmission->getCenterFrequency() && bandListening->getBandwidth() >= wakeUpTransmission->getBandwidth()) {
            itIsPossible = true;
        }
    }
    if (!itIsPossible)
        return false;

    const INarrowbandSignalAnalogModel *narrowbandSignalAnalogModel = check_and_cast<const INarrowbandSignalAnalogModel *>(reception->getAnalogModel());
    W minReceptionPower = narrowbandSignalAnalogModel->computeMinPower(reception->getStartTime(), reception->getEndTime());
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
}
} // namespace inet

