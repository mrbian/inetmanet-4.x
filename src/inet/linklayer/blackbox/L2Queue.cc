//
// This file is part of an OMNeT++/OMNEST simulation example.
//
//
#include <stdio.h>
#include <string.h>
#include "L2Queue.h"
#include "BlackBoxLabel_m.h"
/**
 * Point-to-point interface module. While one frame is transmitted,
 * additional frames get queued up; see NED file for more info.
 */

namespace inet {
namespace blackbox {

Define_Module(L2Queue);

L2Queue::~L2Queue()
{
    cancelAndDelete(endTransmissionEvent);
}

void L2Queue::initialize()
{
    endTransmissionEvent = new cMessage("endTxEvent");
    if (par("useCutThroughSwitching"))
        gate("line$i")->setDeliverImmediately(true);

    frameCapacity = par("frameCapacity");

    qlenSignal = registerSignal("qlen");
    busySignal = registerSignal("busy");
    queueingTimeSignal = registerSignal("queueingTime");
    dropSignal = registerSignal("drop");
    txBytesSignal = registerSignal("txBytes");
    rxBytesSignal = registerSignal("rxBytes");

    emit(qlenSignal, queue->getLength());
    emit(busySignal, false);
    isBusy = false;
    auto channel = gate("line$o")->getTransmissionChannel();
    auto channelRate = check_and_cast<cDatarateChannel *>(channel);
    bitRate = bps(channelRate->getDatarate());
    queue = std::make_unique<WfqQueue>(bitRate);
    queue->setName("queue");
}

void L2Queue::addQueue(const int &label, const double &w)
{
    queue->addQueue(label, w);
}

void L2Queue::addQueue(const int &label, const bps &rate)
{
    double w = rate.get()/bitRate.get();
    queue->addQueue(label, w);
}

void L2Queue::removeQueue(const int &label)
{
    queue->removeQueueLater(label);// mark the queue to be deleted when is empty
}

void L2Queue::startTransmitting(cMessage *msg)
{
    EV << "Starting transmission of " << msg << endl;
    isBusy = true;
    int64_t numBytes = check_and_cast<cPacket *>(msg)->getByteLength();
    send(msg, "line$o");

    emit(txBytesSignal, numBytes);

    // Schedule an event for the time when last bit will leave the gate.
    simtime_t endTransmission = gate("line$o")->getTransmissionChannel()->getTransmissionFinishTime();
    scheduleAt(endTransmission, endTransmissionEvent);
}

void L2Queue::handleMessage(cMessage *msg)
{
    if (msg == endTransmissionEvent) {
        // Transmission finished, we can start next one.
        EV << "Transmission finished.\n";
        isBusy = false;
        if (queue->isEmpty()) {
            emit(busySignal, false);
        }
        else {
            msg = (cMessage *)queue->getPaket();
            emit(queueingTimeSignal, simTime() - msg->getTimestamp());
            emit(qlenSignal, queue->getLength());
            startTransmitting(msg);
        }
    }
    else if (msg->arrivedOn("line$i")) {
        // pass up
        emit(rxBytesSignal, (intval_t)check_and_cast<cPacket *>(msg)->getByteLength());
        send(msg, "out");
    }
    else {  // arrived on gate "in"
        if (endTransmissionEvent->isScheduled()) {
            // We are currently busy, so just queue up the packet.
            if (frameCapacity && queue->getLength() >= frameCapacity) {
                EV << "Received " << msg << " but transmitter busy and queue full: discarding\n";
                emit(dropSignal, (intval_t)check_and_cast<cPacket *>(msg)->getByteLength());
                delete msg;
            }
            else {
                EV << "Received " << msg << " but transmitter busy: queueing up\n";
                msg->setTimestamp();
                // extract label
                auto pkt = check_and_cast<Packet*>(msg);
                auto chunk = pkt->peekAtFront<Chunk>();
                auto label = dynamicPtrCast<const BlackBoxLabel>(chunk);
                if (label != nullptr) {
                    label->getLabels().back();
                    queue->addPacket(pkt, label->getLabels().back());
                }
                else
                    queue->addPacket(pkt);
                emit(qlenSignal, queue->getLength());
            }
        }
        else {
            // We are idle, so we can start transmitting right away.
            EV << "Received " << msg << endl;
            emit(queueingTimeSignal, SIMTIME_ZERO);
            startTransmitting(msg);
            emit(busySignal, true);
        }
    }
}

void L2Queue::refreshDisplay() const
{
    getDisplayString().setTagArg("t", 0, isBusy ? "transmitting" : "idle");
    getDisplayString().setTagArg("i", 1, isBusy ? (queue->getLength() >= 3 ? "red" : "yellow") : "");
}

}
}
