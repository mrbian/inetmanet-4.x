//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#include "inet/physicallayer/wireless/common/base/packetlevel/SnirReceiverBase.h"

#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadioMedium.h"
#include "inet/physicallayer/wireless/common/radio/packetlevel/ReceptionDecision.h"
#include "inet/physicallayer/wireless/ieee80211/packetlevel/Ieee80211Radio.h"
#include "inet/physicallayer/wireless/ieee80211/packetlevel/Ieee80211Transmission.h"
#include "inet/physicallayer/wireless/common/modulation/BpskModulation.h"
#include "inet/physicallayer/wireless/common/modulation/Qam1024Modulation.h"
#include "inet/physicallayer/wireless/common/modulation/Qam16Modulation.h"
#include "inet/physicallayer/wireless/common/modulation/Qam256Modulation.h"
#include "inet/physicallayer/wireless/common/modulation/Qam64Modulation.h"
#include "inet/physicallayer/wireless/common/modulation/QbpskModulation.h"
#include "inet/physicallayer/wireless/common/modulation/QpskModulation.h"
#include "inet/physicallayer/wireless/ieee80211/mode/Ieee80211DsssMode.h"
#include "inet/physicallayer/wireless/ieee80211/mode/Ieee80211OfdmMode.h"

namespace inet {

namespace physicallayer {

void SnirReceiverBase::initialize(int stage)
{
    ReceiverBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        snirThreshold = math::dB2fraction(par("snirThreshold"));
        snirThreshold_rts_cts_ack = math::dB2fraction(par("snirThreshold_rts_cts_ack"));
        const char *snirThresholdModeString = par("snirThresholdMode");
        if (!strcmp("min", snirThresholdModeString))
            snirThresholdMode = SnirThresholdMode::STM_MIN;
        else if (!strcmp("mean", snirThresholdModeString))
            snirThresholdMode = SnirThresholdMode::STM_MEAN;
        else
            throw cRuntimeError("Unknown SNIR threshold mode: '%s'", snirThresholdModeString);
    }
}

std::ostream& SnirReceiverBase::printToStream(std::ostream& stream, int level, int evFlags) const
{
    if (level <= PRINT_LEVEL_TRACE)
        stream << EV_FIELD(snirThreshold);
    return ReceiverBase::printToStream(stream, level, evFlags);
}


//bool SnirReceiverBase::computeIsReceptionSuccessful(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISnir *snir) const
//{
//    auto transmission = check_and_cast<const Ieee80211Transmission *>(snir->getReception()->getTransmission());
//    auto packet = transmission->getPacket();
//    const char* packetName = packet->getName();
//    double minSnir = snir->getMin();
//    if((strstr(packetName, "WlanAck") != nullptr) || (strstr(packetName, "RTS") != nullptr) || (strstr(packetName, "CTS") != nullptr))
//    {
//        return minSnir > snirThreshold_rts_cts_ack;
//    }
////    else if((strstr(packetName, "UDPData") != nullptr) || (strstr(packetName, "OGM") != nullptr))
//    else
//    {
//        return minSnir > snirThreshold;
//    }
////    else
////        throw cRuntimeError("Unknown packetName: '%s'", packetName);
//}

bool SnirReceiverBase::computeIsReceptionSuccessful(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISnir *snir) const
{
    if (snirThresholdMode == SnirThresholdMode::STM_MIN) {
        double minSnir = snir->getMin();
        ASSERT(0.0 <= minSnir);
        return minSnir > snirThreshold;
    }
    else if (snirThresholdMode == SnirThresholdMode::STM_MEAN) {
        double meanSnir = snir->getMean();
        ASSERT(0.0 <= meanSnir);
        return meanSnir > snirThreshold;
    }
    else
        throw cRuntimeError("Unknown SNIR threshold mode: '%d'", static_cast<int>(snirThresholdMode));
}


//bool SnirReceiverBase::computeIsReceptionSuccessful(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISnir *snir) const
//{
    //    auto transmission = check_and_cast<const Ieee80211Transmission *>(snir->getReception()->getTransmission());
    //    auto mode = transmission->getMode();
    //    if(auto ofdmMode = dynamic_cast<const Ieee80211OfdmMode *>(mode))
    //    {
    //        const ApskModulationBase *subcarrierModulation = ofdmMode->getDataMode()->getModulation()->getSubcarrierModulation();
    //        if(subcarrierModulation == &Qam64Modulation::singleton)
    //        {
    //            EV << "data" << std::endl;
    //        }
    //        else
    //        {
    //            EV << "command" << std::endl;
    //        }
    //    }
    //    else if (auto dsssMode = dynamic_cast<const Ieee80211DsssMode *>(mode))
    //    {
    //        const bps bitrate = dsssMode->getDataMode()->getNetBitrate();
    //    }
//}



} // namespace physicallayer

} // namespace inet

