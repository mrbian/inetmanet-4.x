//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/common/Simsignals.h"
#include "inet/common/packet/chunk/BitCountChunk.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/RadioControlInfo_m.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/SignalTag_m.h"
#include "inet/physicallayer/wireless/common/medium/RadioMedium.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatReceiverBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatTransmitterBase.h"
#include "inet/physicallayer/wireless/wakeup/packetlevel/WakeUpRadioBase.h"


namespace inet {

namespace physicallayer {

Define_Module(WakeUpRadioBase);

WakeUpRadioBase::WakeUpRadioBase() :
    NarrowbandRadioBase()
{
}

void WakeUpRadioBase::parseControllerRadioModeSwitchingTimes()
{
    const char *times = controlledRadio->par("switchingTimes");

    char prefix[3];
    unsigned int count = sscanf(times, "%s", prefix);

    if (count > 2)
        throw cRuntimeError("Metric prefix should be no more than two characters long");

    double metric = 1;

    if (strcmp("s", prefix) == 0)
        metric = 1;
    else if (strcmp("ms", prefix) == 0)
        metric = 0.001;
    else if (strcmp("ns", prefix) == 0)
        metric = 0.000000001;
    else
        throw cRuntimeError("Undefined or missed metric prefix for switchingTimes parameter");

    cStringTokenizer tok(times + count + 1);
    unsigned int idx = 0;
    while (tok.hasMoreTokens()) {
        controlledSwitchingTimes[idx / RADIO_MODE_SWITCHING][idx % RADIO_MODE_SWITCHING] = atof(tok.nextToken()) * metric;
        idx++;
    }
    if (idx != RADIO_MODE_SWITCHING * RADIO_MODE_SWITCHING)
        throw cRuntimeError("Check your switchingTimes parameter! Some parameters may be missed");
}

Radio::RadioMode WakeUpRadioBase::getRadioMode() const
{
    // TODO: emulate, if the radio is sleeping, the upper should perceive it like in receiving mode
    if (controlledRadio->Radio::getRadioMode() == IRadio::RADIO_MODE_SLEEP || controlledRadio->Radio::getRadioMode() == IRadio::RADIO_MODE_OFF)
        return IRadio::RADIO_MODE_RECEIVER;
    return controlledRadio->Radio::getRadioMode();
}

void WakeUpRadioBase::initialize(int stage)
{
    Radio::initialize(stage);
    if  (stage == INITSTAGE_PHYSICAL_LAYER) {
        // initialize
        toControlled = gate("toControlled");
        auto mod = toControlled->getPathEndGate()->getOwner();
        cModule *modAux = this->getParentModule()->getSubmodule("controlledRadio");
        controlledRadio = check_and_cast<Radio *>(modAux);
        if (mod != controlledRadio)
            throw cRuntimeError("check gate and radio controlled module");

        awake = new cMessage("awakeRadio");
        interval = par("interval");
        parseControllerRadioModeSwitchingTimes();
    }
}

void WakeUpRadioBase::setState(RadioMode newRadioMode)
{
    Enter_Method("setState");
    Radio::setRadioMode(newRadioMode);
}

void WakeUpRadioBase::handleSelfMessage(cMessage *message)
{
    if (message == awake) {
        if (radioMode == IRadio::RADIO_MODE_OFF || radioMode == IRadio::RADIO_MODE_SLEEP)
            setState(IRadio::RADIO_MODE_RECEIVER);
        scheduleAt(interval, awake);
        if (scanning->isScheduled()) {
            cancelEvent(scanning);
        }
        scheduleAt(scanInterval, scanning);
    }
    else if (message == scanning) {
        if (radioMode == IRadio::RADIO_MODE_RECEIVER) {
            if (getReceptionState () == RECEPTION_STATE_IDLE || getReceptionState () == RECEPTION_STATE_UNDEFINED) {
                setState(IRadio::RADIO_MODE_SLEEP);
                if (controlledRadio->Radio::getRadioMode() == RADIO_MODE_RECEIVER && controlledRadio->getReceptionState () == RECEPTION_STATE_IDLE){
                    controlledRadio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
                }
            }
            else if (getReceptionState () == RECEPTION_STATE_BUSY ) {
                controlledRadio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
            }
        }
    }
    else {
        Radio::handleSelfMessage(message);
    }
}


void WakeUpRadioBase::startReception(cMessage *timer, IRadioSignal::SignalPart part)
{
    auto signal = static_cast<WirelessSignal *>(timer->getControlInfo());
    auto arrival = signal->getArrival();
    auto reception = signal->getReception();
    // TODO should be this, but it breaks fingerprints: if (receptionTimer == nullptr && isReceiverMode(radioMode) && arrival->getStartTime(part) == simTime()) {
    if (isReceiverMode(radioMode) && arrival->getStartTime(part) == simTime()) {
        auto transmission = signal->getTransmission();
        auto isReceptionAttempted = medium->isReceptionAttempted(this, transmission, part);
        EV_INFO << "Reception started: " << (isReceptionAttempted ? "\x1b[1mattempting\x1b[0m" : "\x1b[1mnot attempting\x1b[0m") << " " << (IWirelessSignal *)signal << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
        if (isReceptionAttempted) {
            receptionTimer = timer;
            emit(receptionStartedSignal, check_and_cast<const cObject *>(reception));
        }
    }
    else
        EV_INFO << "Reception started: \x1b[1mignoring\x1b[0m " << (IWirelessSignal *)signal << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;

    sendAwakeReceiver();
    timer->setKind(part);
    scheduleAt(arrival->getEndTime(part), timer);
    updateTransceiverState();
    updateTransceiverPart();
    // TODO move to radio medium
    check_and_cast<RadioMedium *>(medium.get())->emit(IRadioMedium::signalArrivalStartedSignal, check_and_cast<const cObject *>(reception));
}

void WakeUpRadioBase::sendBeacon()
{
    Packet *packet = new Packet();
    auto chunk = makeShared<BitCountChunk>();
    chunk->setLength(b(16));
    packet->addTagIfAbsent<SignalBitrateReq>()->setDataBitrate(bps((int)std::ceil(16/interval.dbl())));
    encapsulate(packet);
    WirelessSignal *signal = check_and_cast<WirelessSignal *>(medium->transmitPacket(this, packet));
}

void WakeUpRadioBase::setRadioMode(RadioMode newRadioMode)
{
    if (newRadioMode == IRadio::RADIO_MODE_TRANSMITTER) {
        setState(IRadio::RADIO_MODE_TRANSMITTER);
        sendBeacon();
        controlledRadio->setRadioMode(newRadioMode);
    }
    else if (newRadioMode == IRadio::RADIO_MODE_RECEIVER) {
        controlledRadio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
    }
}

void WakeUpRadioBase::handleUpperPacket(Packet *packet)
{
    if (getRadioMode() == IRadio::RADIO_MODE_TRANSMITTER || getRadioMode() == IRadio::RADIO_MODE_TRANSCEIVER) {
        send(packet, toControlled);
        return;
    }
    controlledRadio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
    if (getRadioMode() == IRadio::RADIO_MODE_TRANSMITTER) {
        send(packet, toControlled);
    }
    else {
        simtime_t switchingTime = controlledSwitchingTimes[controlledRadio->getRadioMode()][RADIO_MODE_TRANSMITTER];
        sendDelayed(packet, switchingTime, toControlled);
    }
}

void WakeUpRadioBase::endReception(cMessage *timer)
{
    auto part = (IRadioSignal::SignalPart)timer->getKind();
    auto signal = static_cast<WirelessSignal *>(timer->getControlInfo());
    auto arrival = signal->getArrival();
    auto reception = signal->getReception();
    if (timer == receptionTimer && isReceiverMode(radioMode) && arrival->getEndTime() == simTime()) {
        auto transmission = signal->getTransmission();
// TODO: this would draw twice from the random number generator in isReceptionSuccessful: auto isReceptionSuccessful = medium->isReceptionSuccessful(this, transmission, part);
        auto isReceptionSuccessful = medium->getReceptionDecision(this, signal->getListening(), transmission, part)->isReceptionSuccessful();
        EV_INFO << "Reception ended: " << (isReceptionSuccessful ? "\x1b[1msuccessfully\x1b[0m" : "\x1b[1munsuccessfully\x1b[0m") << " for " << (IWirelessSignal *)signal << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
        auto macFrame = medium->receivePacket(this, signal);
        // TODO: FIXME: see handling packets with incorrect PHY headers in the TODO file
        delete macFrame;
        emit(receptionEndedSignal, check_and_cast<const cObject *>(reception));
    }
    else
        EV_INFO << "Reception ended: \x1b[1mignoring\x1b[0m " << (IWirelessSignal *)signal << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
    updateTransceiverState();
    updateTransceiverPart();
    if(timer == receptionTimer)
        receptionTimer = nullptr;
    delete timer;

    // TODO: move to radio medium
    check_and_cast<RadioMedium *>(medium.get())->emit(IRadioMedium::signalArrivalEndedSignal, check_and_cast<const cObject *>(reception));
}



void WakeUpRadioBase::handleUpperCommand(cMessage *message)
{
    send(message, "toControlled");
}

void WakeUpRadioBase::setPower(W newPower)
{
    FlatTransmitterBase *flatTransmitter = const_cast<FlatTransmitterBase *>(check_and_cast<const FlatTransmitterBase *>(transmitter));
    flatTransmitter->setPower(newPower);
}

void WakeUpRadioBase::sendAwakeReceiver()
{
    if (getRadioMode() == Radio::RADIO_MODE_OFF || getRadioMode() == Radio::RADIO_MODE_SLEEP) {
        controlledRadio->setRadioMode(RADIO_MODE_RECEIVER);
    }
}


void WakeUpRadioBase::sendAwakeTransmitter()
{
    if (getRadioMode() == Radio::RADIO_MODE_OFF || getRadioMode() == Radio::RADIO_MODE_SLEEP) {
        controlledRadio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
    }
}



} // namespace physicallayer

} // namespace inet

