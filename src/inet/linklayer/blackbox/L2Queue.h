//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//
#ifndef _INET_BLACKBOX_L2QUEUE__
#define _INET_BLACKBOX_L2QUEUE__

#include "WfqQueue.h"

/**
 * Point-to-point interface module. While one frame is transmitted,
 * additional frames get queued up; see NED file for more info.
 */
namespace inet {
namespace blackbox {
class L2Queue : public cSimpleModule
{
  private:
    intval_t frameCapacity;

    std::unique_ptr<WfqQueue> queue;
    cMessage *endTransmissionEvent = nullptr;
    bool isBusy;

    simsignal_t qlenSignal;
    simsignal_t busySignal;
    simsignal_t queueingTimeSignal;
    simsignal_t dropSignal;
    simsignal_t txBytesSignal;
    simsignal_t rxBytesSignal;

    bps bitRate;

  public:
    virtual ~L2Queue();
    virtual bps getChannelBitrate() const {return bitRate;}
    virtual void addQueue(const int&, const double&);
    virtual void addQueue(const int&, const bps&);
    virtual void removeQueue(const int &label);

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void startTransmitting(cMessage *msg);
};
}
}
#endif


