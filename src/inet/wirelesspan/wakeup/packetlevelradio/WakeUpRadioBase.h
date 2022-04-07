//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_WAKEURADIOBASE_H
#define __INET_WAKEURADIOBASE_H

#include "inet/physicallayer/wireless/common/base/packetlevel/NarrowbandRadioBase.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

class INET_API WakeUpRadioBase : public inet::physicallayer::NarrowbandRadioBase, public cListener
{
protected:
    cMessage *awake = nullptr;
    cMessage *scanning = nullptr;
    simtime_t interval;
    simtime_t scanInterval;
    Radio *controlledRadio = nullptr;
    cGate *toControlled = nullptr;
    simtime_t controlledSwitchingTimes[RADIO_MODE_SWITCHING][RADIO_MODE_SWITCHING];

    bool inmediateAwake = true;

    int controlledSistemGateId = -1;
    std::set<int> channels;


  protected:

    virtual void parseControllerRadioModeSwitchingTimes();

    virtual ReceptionState getReceptionState() const override;

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;

    virtual void updateTransceiverState() override;


    virtual void handleUpperCommand(cMessage *message) override;
    virtual void initialize(int stage) override;
    virtual void handleSelfMessage(cMessage *message) override;
    virtual void handleUpperPacket(Packet *packet) override;
    virtual void handleMessageWhenUp(cMessage *message) override;


    virtual bool isFromControlled(cMessage *message) const;

    virtual RadioMode getRadioMode() const override;
    virtual void sendAwakeReceiver();
    virtual void sendAwakeTransmitter();
    virtual void setRadioMode(RadioMode newRadioMode) override;

    virtual void startReception(cMessage *timer, inet::physicallayer::IRadioSignal::SignalPart part) override;
    virtual void endReception(cMessage *timer) override;

    // internal methods
    virtual void sendBeacon();
    virtual void setState(RadioMode newRadioMode);

  public:


    WakeUpRadioBase();

    virtual void setPower(W newPower);
    virtual void setAwakeningInterval(simtime_t it) {interval = it;}
    virtual simtime_t getAwakeningInterval() const {return interval;}
    virtual void setBitrate(bps newBitrate);

    virtual void setWakeUpMode(); // force the controlled radio to sleep and start the scanning mode
    virtual void cancelScanning();
    virtual void setModeControlled(RadioMode newRadioMode);
    virtual void setRadioModeNoBeacon(RadioMode newRadioMode);
};

} // namespace physicallayer
}
} // namespace inet

#endif

