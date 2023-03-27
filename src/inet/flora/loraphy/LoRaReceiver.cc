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

#include "inet/flora/loraphy/LoRaReceiver.h"
#include "inet/flora/loraapp/SimpleLoRaApp.h"
#include "inet/flora/loraphy/LoRaPhyPreamble_m.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/NarrowbandNoiseBase.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/SignalTag_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/ModuleAccess.h"

namespace inet {

namespace flora {

using namespace physicallayer;

Define_Module(LoRaReceiver);

LoRaReceiver::LoRaReceiver() :
    snirThreshold(NaN)
{
}

void LoRaReceiver::initialize(int stage)
{
    FlatReceiverBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
    {
        snirThreshold = math::dB2fraction(par("snirThreshold"));
        if(strcmp(getParentModule()->getClassName(), "inet::flora::LoRaGWRadio") == 0)
        {
            iAmGateway = true;
        } else iAmGateway = false;
        alohaChannelModel = par("alohaChannelModel");
        LoRaReceptionCollision = registerSignal("LoRaReceptionCollision");
        numCollisions = 0;
        rcvBelowSensitivity = 0;
        SimpleLoRaApp * loRaAppAux = nullptr;
        auto node = getContainingNode(this);
        auto names = node->getSubmoduleNames();
        for (const auto &elem : names) {
            auto mod = node->getSubmodule(elem.c_str());
            loRaAppAux = dynamic_cast<SimpleLoRaApp *>(mod);
            if (loRaAppAux) { // SimpleLoRaApp found
                loraApp = loRaAppAux;
                break;
            }
        }
    }
    else if (stage == INITSTAGE_PHYSICAL_LAYER) {
        if(iAmGateway == false) {
            auto loRaAppAux = dynamic_cast<SimpleLoRaApp *>(loraApp);
            if (loRaAppAux) { // SimpleLoRaApp found
                setCenterFrequency(loRaAppAux->loRaCF);
                setBandwidth(loRaAppAux->loRaBW);
            }
            else {
                throw cRuntimeError("This node is not a Gateway and app module of type SimpleLoRaApp not found");
            }
        }
    }
}

void LoRaReceiver::finish()
{
        recordScalar("numCollisions", numCollisions);
        recordScalar("rcvBelowSensitivity", rcvBelowSensitivity);

}

bool LoRaReceiver::computeIsReceptionPossible(const IListening *listening, const ITransmission *transmission) const
{
    const LoRaTransmission *loRaTransmission = dynamic_cast<const LoRaTransmission *>(transmission);
    if (loRaTransmission == nullptr) // it is not a lora transmission reject it
        return false;
    //here we can check compatibility of LoRaTx parameters (or beeing a gateway)
    auto loRaAppAux = dynamic_cast<SimpleLoRaApp *>(loraApp);
    //auto *loRaApp = check_and_cast<flora::SimpleLoRaApp *>(getParentModule()->getParentModule()->getSubmodule("SimpleLoRaApp"));
    if(iAmGateway || (loRaTransmission->getLoRaCF() == loRaAppAux->loRaCF && loRaTransmission->getLoRaBW() == loRaAppAux->loRaBW && loRaTransmission->getLoRaSF() == loRaAppAux->loRaSF))
        return true;
    else
        return false;
}

bool LoRaReceiver::computeIsReceptionPossible(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part) const
{

    const LoRaReception *loRaReception = dynamic_cast<const LoRaReception *>(reception);
    if (loRaReception == nullptr) // it is not a lora reception
        return false;

    //here we can check compatibility of LoRaTx parameters (or beeing a gateway) and reception above sensitivity level
    const LoRaBandListening *loRaListening = check_and_cast<const LoRaBandListening *>(listening);

    if (iAmGateway == false && (loRaListening->getLoRaCF() != loRaReception->getLoRaCF() || loRaListening->getLoRaBW() != loRaReception->getLoRaBW() || loRaListening->getLoRaSF() != loRaReception->getLoRaSF())) {
        return false;
    } else {
        W minReceptionPower = loRaReception->computeMinPower(reception->getStartTime(part), reception->getEndTime(part));
        W sensitivity = getSensitivity(loRaReception);
        bool isReceptionPossible = minReceptionPower >= sensitivity;
        EV_DEBUG << "Computing whether reception is possible: minimum reception power = " << minReceptionPower << ", sensitivity = " << sensitivity << " -> reception is " << (isReceptionPossible ? "possible" : "impossible") << endl;
        if(isReceptionPossible == false) {
           const_cast<LoRaReceiver* >(this)->rcvBelowSensitivity++;
        }
        return isReceptionPossible;
    }
}

bool LoRaReceiver::computeIsReceptionAttempted(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference) const
{
    const auto loRaReception = dynamic_cast<const LoRaReception *>(reception);
    if (loRaReception == nullptr)
        return false; // a non lora packet, ignore it

    if(isPacketCollided(reception, part, interference))
    {
        auto packet = reception->getTransmission()->getPacket();
        const auto &chunk = packet->peekAtFront<FieldsChunk>();
        auto loraMac = dynamicPtrCast<const LoRaMacFrame>(chunk);
        auto loraPreamble = dynamicPtrCast<const LoRaPhyPreamble>(chunk);
        MacAddress rec;
        if (loraPreamble)
            rec = loraPreamble->getReceiverAddress();
        else if (loraMac)
            rec = loraMac->getReceiverAddress();

        if (iAmGateway == false) {
            auto macLayer = dynamic_cast<flora::LoRaMac *>(getParentModule()->getParentModule()->getSubmodule("mac"));
            if (macLayer && rec == macLayer->getAddress()) {
                const_cast<LoRaReceiver* >(this)->numCollisions++;
            }
            //EV << "Node: Extracted macFrame = " << loraMacFrame->getReceiverAddress() << ", node address = " << macLayer->getAddress() << std::endl;
        } else {
            auto *gwMacLayer = check_and_cast<flora::LoRaGWMac *>(getParentModule()->getParentModule()->getSubmodule("mac"));
            EV << "GW: Extracted macFrame = " << rec << ", node address = " << gwMacLayer->getAddress() << std::endl;
            if (rec == MacAddress::BROADCAST_ADDRESS) {
                const_cast<LoRaReceiver* >(this)->numCollisions++;
            }
        }
        return false;
    } else {
        return true;
    }
}

bool LoRaReceiver::isPacketCollided(const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference) const
{
    //auto radio = reception->getReceiver();
    //auto radioMedium = radio->getMedium();
    auto interferingReceptions = interference->getInterferingReceptions();
    const auto loRaReception = check_and_cast<const LoRaReception *>(reception);

    simtime_t m_x = (loRaReception->getStartTime() + loRaReception->getEndTime())/2;
    simtime_t d_x = (loRaReception->getEndTime() - loRaReception->getStartTime())/2;
    EV << "Czas transmisji to " << loRaReception->getEndTime() - loRaReception->getStartTime() << endl;
    //double P_threshold = 6;
    W signalRSSI_w = loRaReception->getPower();
    double signalRSSI_mw = signalRSSI_w.get()*1000;
    double signalRSSI_dBm = math::mW2dBmW(signalRSSI_mw);
    EV << signalRSSI_mw << endl;
    EV << signalRSSI_dBm << endl;

    int receptionSF = loRaReception->getLoRaSF();
    for (auto interferingReception : *interferingReceptions) {
        bool overlap = false;
        bool frequencyCollision = false;
        bool captureEffect = false;
        bool timingCollision = false; //Collision is acceptable in first part of preamble
        const LoRaReception *loRaInterference = check_and_cast<const LoRaReception *>(interferingReception);

        simtime_t m_y = (loRaInterference->getStartTime() + loRaInterference->getEndTime())/2;
        simtime_t d_y = (loRaInterference->getEndTime() - loRaInterference->getStartTime())/2;
        if(omnetpp::fabs(m_x - m_y) < d_x + d_y)
        {
            overlap = true;
        }

        if(loRaReception->getLoRaCF() == loRaInterference->getLoRaCF()) {
            frequencyCollision = true;
        }

        W interferenceRSSI_w = loRaInterference->getPower();
        double interferenceRSSI_mw = interferenceRSSI_w.get()*1000;
        double interferenceRSSI_dBm = math::mW2dBmW(interferenceRSSI_mw);
        int interferenceSF = loRaInterference->getLoRaSF();

        /* If difference in power between two signals is greater than threshold, no collision*/
        if(signalRSSI_dBm - interferenceRSSI_dBm >= nonOrthDelta[receptionSF-7][interferenceSF-7])
        {
            captureEffect = true;
        }

        EV << "[MSDEBUG] Received packet at SF: " << receptionSF << " with power " << signalRSSI_dBm << endl;
        EV << "[MSDEBUG] Received interference at SF: " << interferenceSF << " with power " << interferenceRSSI_dBm << endl;
        EV << "[MSDEBUG] Acceptable diff is equal " << nonOrthDelta[receptionSF-7][interferenceSF-7] << endl;
        EV << "[MSDEBUG] Diff is equal " << signalRSSI_dBm - interferenceRSSI_dBm << endl;
        if (captureEffect == false)
        {
            EV << "[MSDEBUG] Packet is discarded" << endl;
        } else
            EV << "[MSDEBUG] Packet is not discarded" << endl;

        /* If last 6 symbols of preamble are received, no collision*/
        double nPreamble = 8; //from the paper "Do Lora networks..."
        simtime_t Tsym = (pow(2, loRaReception->getLoRaSF()))/(loRaReception->getLoRaBW().get()/1000)/1000;
        simtime_t csBegin = loRaReception->getPreambleStartTime() + Tsym * (nPreamble - 6);
        if(csBegin < loRaInterference->getEndTime())
        {
            timingCollision = true;
        }

        if (overlap && frequencyCollision)
        {
            if(alohaChannelModel == true)
            {
                if(iAmGateway && (part == IRadioSignal::SIGNAL_PART_DATA || part == IRadioSignal::SIGNAL_PART_WHOLE)) const_cast<LoRaReceiver* >(this)->emit(LoRaReceptionCollision, true);
                return true;
            }
            if(alohaChannelModel == false)
            {
                if(captureEffect == false && timingCollision)
                {
                    if(iAmGateway && (part == IRadioSignal::SIGNAL_PART_DATA || part == IRadioSignal::SIGNAL_PART_WHOLE)) const_cast<LoRaReceiver* >(this)->emit(LoRaReceptionCollision, true);
                    return true;
                }
            }

        }
    }
    return false;
}

const IReceptionDecision *LoRaReceiver::computeReceptionDecision(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISnir *snir) const
{
    auto isReceptionPossible = computeIsReceptionPossible(listening, reception, part);
    auto isReceptionAttempted = isReceptionPossible && computeIsReceptionAttempted(listening, reception, part, interference);
    auto isReceptionSuccessful = isReceptionAttempted && computeIsReceptionSuccessful(listening, reception, part, interference, snir);
    return new ReceptionDecision(reception, part, isReceptionPossible, isReceptionAttempted, isReceptionSuccessful);
}

Packet *LoRaReceiver::computeReceivedPacket(const ISnir *snir, bool isReceptionSuccessful) const
{
    auto transmittedPacket = snir->getReception()->getTransmission()->getPacket();
    auto receivedPacket = transmittedPacket->dup();
    receivedPacket->clearTags();
//    receivedPacket->addTag<PacketProtocolTag>()->setProtocol(transmittedPacket->getTag<PacketProtocolTag>()->getProtocol());
    if (!isReceptionSuccessful)
        receivedPacket->setBitError(true);
    return receivedPacket;
}

const IReceptionResult *LoRaReceiver::computeReceptionResult(const IListening *listening, const IReception *reception, const IInterference *interference, const ISnir *snir, const std::vector<const IReceptionDecision *> *decisions) const
{
    bool isReceptionSuccessful = true;
    for (auto decision : *decisions)
        isReceptionSuccessful &= decision->isReceptionSuccessful();
    auto packet = computeReceivedPacket(snir, isReceptionSuccessful);

    auto signalPowerInd = packet->addTagIfAbsent<SignalPowerInd>();
    const LoRaReception *loRaReception = check_and_cast<const LoRaReception *>(reception);
    W signalRSSI_w = loRaReception->getPower();
    signalPowerInd->setPower(signalRSSI_w);

    auto snirInd = packet->addTagIfAbsent<SnirInd>();
    snirInd->setMinimumSnir(snir->getMin());
    snirInd->setMaximumSnir(snir->getMax());
    auto signalTimeInd = packet->addTagIfAbsent<SignalTimeInd>();
    signalTimeInd->setStartTime(reception->getStartTime());
    signalTimeInd->setEndTime(reception->getEndTime());
    auto errorRateInd = packet->addTagIfAbsent<ErrorRateInd>();
    errorRateInd->setPacketErrorRate(errorModel ? errorModel->computePacketErrorRate(snir, IRadioSignal::SIGNAL_PART_WHOLE) : 0.0);
    errorRateInd->setBitErrorRate(errorModel ? errorModel->computeBitErrorRate(snir, IRadioSignal::SIGNAL_PART_WHOLE) : 0.0);
    errorRateInd->setSymbolErrorRate(errorModel ? errorModel->computeSymbolErrorRate(snir, IRadioSignal::SIGNAL_PART_WHOLE) : 0.0);

    return new ReceptionResult(reception, decisions, packet);
}

bool LoRaReceiver::computeIsReceptionSuccessful(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISnir *snir) const
{
    return true;
    //we don't check the SINR level, it is done in collision checking by P_threshold level evaluation
}

const IListening *LoRaReceiver::createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord &startPosition, const Coord &endPosition) const
{
    if(iAmGateway == false) {
        if (loraApp) {
            auto loRaApp = check_and_cast<SimpleLoRaApp *>(loraApp);
            return new LoRaBandListening(radio, startTime, endTime, startPosition, endPosition, loRaApp->loRaCF, loRaApp->loRaBW, loRaApp->loRaSF);
        }
#ifdef CHECKLORAAPP
        throw cRuntimeError("SimpleLoRaApp not found");
#else
        EV << "SimpleLoRaApp not found";
#endif
    }
    return new LoRaBandListening(radio, startTime, endTime, startPosition, endPosition, centerFrequency, bandwidth, LoRaSF);
}

const IListeningDecision *LoRaReceiver::computeListeningDecision(const IListening *listening, const IInterference *interference) const
{
    const IRadio *receiver = listening->getReceiver();
    const IRadioMedium *radioMedium = receiver->getMedium();
    const IAnalogModel *analogModel = radioMedium->getAnalogModel();
    const INoise *noise = analogModel->computeNoise(listening, interference);
    const auto loRaNoise = check_and_cast<const NarrowbandNoiseBase *>(noise);
    W maxPower = loRaNoise->computeMaxPower(listening->getStartTime(), listening->getEndTime());
    bool isListeningPossible = maxPower >= energyDetection;
    delete noise;
    EV_DEBUG << "Computing whether listening is possible: maximum power = " << maxPower << ", energy detection = " << energyDetection << " -> listening is " << (isListeningPossible ? "possible" : "impossible") << endl;
    return new ListeningDecision(listening, isListeningPossible);
}


W LoRaReceiver::getSensitivityBwSf(const Hz &bandwidth, const int &Sf) const
{
    W sensitivity = mW(math::dBmW2mW(-126.5));
    switch (Sf) {
    case 6:
        if(bandwidth == Hz(125000))
            sensitivity = mW(math::dBmW2mW(-121));
        else if(bandwidth == Hz(250000))
            sensitivity = mW(math::dBmW2mW(-118));
        else if(bandwidth == Hz(500000))
            sensitivity = mW(math::dBmW2mW(-111));
        break;
    case 7:
        if(bandwidth == Hz(125000))
            sensitivity = mW(math::dBmW2mW(-124));
        else if(bandwidth == Hz(250000))
            sensitivity = mW(math::dBmW2mW(-122));
        else if(bandwidth == Hz(500000))
            sensitivity = mW(math::dBmW2mW(-116));
        break;
    case 8:
        if(bandwidth == Hz(125000))
            sensitivity = mW(math::dBmW2mW(-127));
        else if(bandwidth == Hz(250000))
            sensitivity = mW(math::dBmW2mW(-125));
        else if(bandwidth == Hz(500000))
            sensitivity = mW(math::dBmW2mW(-119));
        break;
    case 9:
        if(bandwidth == Hz(125000))
            sensitivity = mW(math::dBmW2mW(-130));
        else if(bandwidth == Hz(250000))
            sensitivity = mW(math::dBmW2mW(-128));
        else if(bandwidth == Hz(500000))
            sensitivity = mW(math::dBmW2mW(-122));
        break;
    case 10:
        if(bandwidth == Hz(125000))
            sensitivity = mW(math::dBmW2mW(-133));
        else if(bandwidth == Hz(250000))
            sensitivity = mW(math::dBmW2mW(-130));
        else if(bandwidth == Hz(500000))
            sensitivity = mW(math::dBmW2mW(-125));
        break;
    case 11:
        if(bandwidth == Hz(125000))
            sensitivity = mW(math::dBmW2mW(-135));
        else if(bandwidth == Hz(250000))
            sensitivity = mW(math::dBmW2mW(-132));
        else if(bandwidth == Hz(500000))
            sensitivity = mW(math::dBmW2mW(-128));
        break;
    case 12:
        if(bandwidth == Hz(125000))
            sensitivity = mW(math::dBmW2mW(-137));
        else if(bandwidth == Hz(250000))
            sensitivity = mW(math::dBmW2mW(-135));
        else if(bandwidth == Hz(500000))
            sensitivity = mW(math::dBmW2mW(-129));
        break;
    }
    return sensitivity;
}

W LoRaReceiver::getSensitivity(const LoRaReception *reception) const
{
    //function returns sensitivity -- according to LoRa documentation, it changes with LoRa parameters
    //Sensitivity values from Semtech SX1272/73 datasheet, table 10, Rev 3.1, March 2017
    W sensitivity = getSensitivityBwSf(reception->getLoRaBW(), reception->getLoRaSF());
#if 0
    if(reception->getLoRaSF() == 6)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBmW2mW(-121) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBmW2mW(-118) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBmW2mW(-111) / 1000);
    }
    if (reception->getLoRaSF() == 7)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBmW2mW(-124) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBmW2mW(-122) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBmW2mW(-116) / 1000);
    }

    if(reception->getLoRaSF() == 8)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBmW2mW(-127) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBmW2mW(-125) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBmW2mW(-119) / 1000);
    }
    if(reception->getLoRaSF() == 9)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBmW2mW(-130) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBmW2mW(-128) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBmW2mW(-122) / 1000);
    }
    if(reception->getLoRaSF() == 10)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBmW2mW(-133) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBmW2mW(-130) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBmW2mW(-125) / 1000);
    }
    if(reception->getLoRaSF() == 11)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBmW2mW(-135) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBmW2mW(-132) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBmW2mW(-128) / 1000);
    }
    if(reception->getLoRaSF() == 12)
    {
        if(reception->getLoRaBW() == Hz(125000)) sensitivity = W(math::dBmW2mW(-137) / 1000);
        if(reception->getLoRaBW() == Hz(250000)) sensitivity = W(math::dBmW2mW(-135) / 1000);
        if(reception->getLoRaBW() == Hz(500000)) sensitivity = W(math::dBmW2mW(-129) / 1000);
    }
#endif
    return sensitivity;
}

W LoRaReceiver::getMinReceptionPower() const
{
    if(iAmGateway) {
        return mW(math::dBmW2mW(-137));
    }
    auto loRaAppAux = dynamic_cast<SimpleLoRaApp *>(loraApp);
    return getSensitivityBwSf(loRaAppAux->loRaBW, loRaAppAux->loRaSF);
}

}
}


