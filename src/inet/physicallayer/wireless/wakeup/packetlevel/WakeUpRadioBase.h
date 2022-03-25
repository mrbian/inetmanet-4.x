//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_WAKEURADIOBASE_H
#define __INET_WAKEURADIOBASE_H

#include "inet/physicallayer/wireless/common/base/packetlevel/NarrowbandRadioBase.h"

namespace inet {

namespace physicallayer {

class INET_API WakeUpRadioBase : public NarrowbandRadioBase
{
protected:
    cMessage *awake = nullptr;
    cMessage *scanning = nullptr;
    simtime_t interval;
    simtime_t scanInterval;
    Radio *controlledRadio = nullptr;
    cGate *toControlled = nullptr;
    simtime_t controlledSwitchingTimes[RADIO_MODE_SWITCHING][RADIO_MODE_SWITCHING];


  protected:

    virtual void parseControllerRadioModeSwitchingTimes();

    virtual void handleUpperCommand(cMessage *message) override;
    virtual void initialize(int stage) override;
    virtual void handleSelfMessage(cMessage *message) override;
    virtual void handleUpperPacket(Packet *packet) override;

    virtual RadioMode getRadioMode() const override;
    virtual void sendAwakeReceiver();
    virtual void sendAwakeTransmitter();
    virtual void setRadioMode(RadioMode newRadioMode) override;

    virtual void startReception(cMessage *timer, IRadioSignal::SignalPart part) override;
    virtual void endReception(cMessage *timer) override;

    // internal methods
    virtual void sendBeacon();
    virtual void setState(RadioMode newRadioMode);

  public:


    WakeUpRadioBase();

    virtual void setPower(W newPower);
    virtual void setAwakeningInterval(simtime_t interval);
    virtual simtime_t getAwakeningInterval() const;
    virtual void setBitrate(bps newBitrate);
};

} // namespace physicallayer

} // namespace inet

#endif

