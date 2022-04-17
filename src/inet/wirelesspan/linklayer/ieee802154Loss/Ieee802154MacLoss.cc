/* -*- mode:c++ -*- ********************************************************
 * file:        Ieee802154MacLoss.cc
 *
 * author:      Jerome Rousselot, Marcel Steine, Amre El-Hoiydi,
 *              Marc Loebbers, Yosia Hadisusanto, Andreas Koepke, Alfonso Ariza
 *
  *copyright:   (C) 2019 Universidad de Málaga
 *              (C) 2007-2009 CSEM SA
 *              (C) 2009 T.U. Eindhoven
 *              (C) 2004,2005,2006
 *              Telecommunication Networks Group (TKN) at Technische
 *              Universitaet Berlin, Germany.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 *
 * Funding: This work was partially financed by the European Commission under the
 * Framework 6 IST Project "Wirelessly Accessible Sensor Populations"
 * (WASP) under contract IST-034963.
 ***************************************************************************
 * part of:    Modifications to the MF-2 framework by CSEM
 **************************************************************************/

#include <cassert>

#include "inet/common/FindModule.h"
#include "inet/common/INETMath.h"
#include "inet/common/INETUtils.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolGroup.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/ieee802154/Ieee802154MacHeader_m.h"
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/common/WirelessGetNeig.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/NarrowbandTransmitterBase.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadioMedium.h"
#include "inet/wirelesspan/linklayer/ieee802154Loss/Ieee802154MacLoss.h"



namespace inet {
namespace wirelesspan {

using namespace inet::physicallayer;
using namespace inet::wirelesspan::physicallayer;

Define_Module(Ieee802154MacLoss);

void Ieee802154MacLoss::initialize(int stage)
{
    Ieee802154Mac::initialize(stage);
    if (stage == INITSTAGE_NETWORK_LAYER) {
        // Read topology and set probabilities.
        WirelessGetNeig *neig = nullptr;
        if (par("setLossProbability").boolValue())
            neig = new WirelessGetNeig();

        wakeUpRadio = dynamic_cast<physicallayer::WakeUpRadioBase *>(radio.get());

        if (neig) {
            auto transmitter = radio->getTransmitter();
            auto narrow = dynamic_cast<const NarrowbandTransmitterBase *>(transmitter);
            auto centerFrequency = narrow->getCenterFrequency();
            auto power = narrow->getMaxPower();
            auto sens = radio->getReceiver()->getMinReceptionPower();
            // compute distance using free space
            auto radioMedium = radio->getMedium();
            auto pathLoss = radioMedium->getPathLoss();
            double loss = unit(sens / power).get();
            auto dist = pathLoss->computeRange(radioMedium->getPropagation()->getPropagationSpeed(), centerFrequency, loss);
            std::vector<L3Address> list;
            neig->getNeighbours(L3Address(networkInterface->getMacAddress()), list, dist.get());
            double probabilityCompleteFailureLink = par("probabilityCompleteFailureLink"); // (between 0..1) percentage of links that will fail in every node
            double probabilityLink = par("probabilityLink"); // (between 0..1) percentage of links that will lost packet in every node
            double probabilityFailurePacket = par("probabilityFailurePacket"); // (between 0..1) probability of loss in link with loss.
            for (auto elem: list) {
                if (probabilityCompleteFailureLink != 0 && dblrand() <  probabilityCompleteFailureLink)
                    probability[elem] = 1.0;
                else if (probabilityLink != 0 && dblrand() <  probabilityLink)
                    probability[elem] = probabilityFailurePacket;
            }
            delete neig;
        }
    }
}

bool Ieee802154MacLoss::discard(const L3Address &addr) {

    auto it = probability.find(addr);
    if (it == probability.end())
        return false;
    if (it->second >= 1.0)
        return true;
    if (it->second > dblrand())
        return true;
    return false;
}

Ieee802154MacLoss::~Ieee802154MacLoss()
{
}



void Ieee802154MacLoss::sendUp(cMessage *message)
{
    auto packet = check_and_cast<Packet *> (message);
    auto  src = packet->getTag<MacAddressInd>()->getSrcAddress();
    // check if the packet must be discarded

    if (discard(src))
        delete message;
    else
        MacProtocolBase::sendUp(message);
}




void Ieee802154MacLoss::updateStatusIdle(t_mac_event event, cMessage *msg)
{
    switch (event) {
        case EV_SEND_REQUEST:
            if (!txQueue->isEmpty()) {
                EV_DETAIL << "(1) FSM State IDLE_1, EV_SEND_REQUEST and [TxBuff avail]: startTimerBackOff -> BACKOFF." << endl;
                updateMacState(BACKOFF_2);
                NB = 0;
//                BE = macMinBE;
                startTimer(TIMER_BACKOFF);
            }
            break;

        case EV_DUPLICATE_RECEIVED:
            EV_DETAIL << "(15) FSM State IDLE_1, EV_DUPLICATE_RECEIVED: setting up radio tx -> WAITSIFS." << endl;
//            sendUp(decapsMsg(static_cast<MacSeqPkt *>(msg)));
            delete msg;

            if (useMACAcks) {
                if (wakeUpRadio)
                    wakeUpRadio->setRadioModeNoBeacon(IRadio::RADIO_MODE_TRANSMITTER);
                else
                    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
                updateMacState(WAITSIFS_6);
                startTimer(TIMER_SIFS);
            }
            break;

        case EV_FRAME_RECEIVED:
            EV_DETAIL << "(15) FSM State IDLE_1, EV_FRAME_RECEIVED: setting up radio tx -> WAITSIFS." << endl;
            decapsulate(check_and_cast<Packet *>(msg));
            sendUp(msg);
            nbRxFrames++;

            if (useMACAcks) {
                if (wakeUpRadio)
                    wakeUpRadio->setRadioModeNoBeacon(IRadio::RADIO_MODE_TRANSMITTER);
                else
                    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
                updateMacState(WAITSIFS_6);
                startTimer(TIMER_SIFS);
            }
            break;

        case EV_BROADCAST_RECEIVED:
            EV_DETAIL << "(23) FSM State IDLE_1, EV_BROADCAST_RECEIVED: Nothing to do." << endl;
            nbRxFrames++;
            decapsulate(check_and_cast<Packet *>(msg));
            sendUp(msg);
            break;

        default:
            fsmError(event, msg);
            break;
    }
}

void Ieee802154MacLoss::updateStatusBackoff(t_mac_event event, cMessage *msg)
{
    switch (event) {
        case EV_TIMER_BACKOFF:
            EV_DETAIL << "(2) FSM State BACKOFF, EV_TIMER_BACKOFF:"
                      << " starting CCA timer." << endl;
            startTimer(TIMER_CCA);
            updateMacState(CCA_3);
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
            break;

        case EV_DUPLICATE_RECEIVED:
            // suspend current transmission attempt,
            // transmit ack,
            // and resume transmission when entering manageQueue()
            EV_DETAIL << "(28) FSM State BACKOFF, EV_DUPLICATE_RECEIVED:";
            if (useMACAcks) {
                EV_DETAIL << "suspending current transmit tentative and transmitting ack";
                transmissionAttemptInterruptedByRx = true;
                cancelEvent(backoffTimer);

                if (wakeUpRadio)
                    wakeUpRadio->setRadioModeNoBeacon(IRadio::RADIO_MODE_TRANSMITTER);
                else
                    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

                updateMacState(WAITSIFS_6);
                startTimer(TIMER_SIFS);
            }
            else {
                EV_DETAIL << "Nothing to do.";
            }
//            sendUp(decapsMsg(static_cast<MacSeqPkt *>(msg)));
            delete msg;

            break;

        case EV_FRAME_RECEIVED:
            // suspend current transmission attempt,
            // transmit ack,
            // and resume transmission when entering manageQueue()
            EV_DETAIL << "(28) FSM State BACKOFF, EV_FRAME_RECEIVED:";
            if (useMACAcks) {
                EV_DETAIL << "suspending current transmit tentative and transmitting ack";
                transmissionAttemptInterruptedByRx = true;
                cancelEvent(backoffTimer);

                if (wakeUpRadio)
                    wakeUpRadio->setRadioModeNoBeacon(IRadio::RADIO_MODE_TRANSMITTER);
                else
                    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

                updateMacState(WAITSIFS_6);
                startTimer(TIMER_SIFS);
            }
            else {
                EV_DETAIL << "sending frame up and resuming normal operation.";
                if (wakeUpRadio)
                    wakeUpRadio->setWakeUpMode();
            }
            decapsulate(check_and_cast<Packet *>(msg));
            sendUp(msg);
            break;

        case EV_BROADCAST_RECEIVED:
            EV_DETAIL << "(29) FSM State BACKOFF, EV_BROADCAST_RECEIVED:"
                      << "sending frame up and resuming normal operation." << endl;
            if (wakeUpRadio)
                wakeUpRadio->setWakeUpMode();
            decapsulate(check_and_cast<Packet *>(msg));
            sendUp(msg);
            break;

        default:
            fsmError(event, msg);
            break;
    }
}

void Ieee802154MacLoss::updateStatusCCA(t_mac_event event, cMessage *msg)
{
    switch (event) {
        case EV_TIMER_CCA: {
            EV_DETAIL << "(25) FSM State CCA_3, EV_TIMER_CCA" << endl;
            bool isIdle = radio->getReceptionState() == IRadio::RECEPTION_STATE_IDLE;
            if (isIdle) {
                EV_DETAIL << "(3) FSM State CCA_3, EV_TIMER_CCA, [Channel Idle]: -> TRANSMITFRAME_4." << endl;
                updateMacState(TRANSMITFRAME_4);
                // Here, the code should a function of the destination address.

                if (wakeUpRadio)
                    wakeUpRadio->awakeNodes(0); // code 0 is default, all nodes
                else
                    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

                if (currentTxFrame == nullptr) {
                    currentTxFrame = dequeuePacket();
                    encapsulate(currentTxFrame);
                }
                Packet *mac = currentTxFrame->dup();
                attachSignal(mac, simTime() + aTurnaroundTime);
//                sendDown(msg);
                // give time for the radio to be in Tx state before transmitting
                if (wakeUpRadio && wakeUpRadio->getAwakeningInterval() > simtime_t::ZERO) // must wait until all the radios are awake
                    sendDelayed(mac, wakeUpRadio->getAwakeningInterval(), lowerLayerOutGateId);
                else
                    sendDelayed(mac, aTurnaroundTime, lowerLayerOutGateId);
                nbTxFrames++;
            }
            else {
                // Channel was busy, increment 802.15.4 backoff timers as specified.
                EV_DETAIL << "(7) FSM State CCA_3, EV_TIMER_CCA, [Channel Busy]: "
                          << " increment counters." << endl;
                NB = NB + 1;
//                BE = std::min(BE+1, macMaxBE);

                // decide if we go for another backoff or if we drop the frame.
                if (NB > macMaxCSMABackoffs) {
                    // drop the frame
                    EV_DETAIL << "Tried " << NB << " backoffs, all reported a busy "
                              << "channel. Dropping the packet." << endl;
                    txAttempts = 0;
                    if (currentTxFrame) {
                        nbDroppedFrames++;
                        PacketDropDetails details;
                        details.setReason(CONGESTION);
                        details.setLimit(macMaxCSMABackoffs);
                        dropCurrentTxFrame(details);
                    }
                    else {
                        EV_ERROR << "too many Backoffs, but currentTxFrame is empty\n"; // TODO is it good, or model error?
                    }
                    manageQueue();
                }
                else {
                    // redo backoff
                    updateMacState(BACKOFF_2);
                    startTimer(TIMER_BACKOFF);
                }
            }
            break;
        }

        case EV_DUPLICATE_RECEIVED:
            EV_DETAIL << "(26) FSM State CCA_3, EV_DUPLICATE_RECEIVED:";
            if (useMACAcks) {
                EV_DETAIL << " setting up radio tx -> WAITSIFS." << endl;
                // suspend current transmission attempt,
                // transmit ack,
                // and resume transmission when entering manageQueue()
                transmissionAttemptInterruptedByRx = true;
                cancelEvent(ccaTimer);

                radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
                updateMacState(WAITSIFS_6);
                startTimer(TIMER_SIFS);
            }
            else {
                EV_DETAIL << " Nothing to do." << endl;
            }
//            sendUp(decapsMsg(static_cast<MacPkt*>(msg)));
            delete msg;
            break;

        case EV_FRAME_RECEIVED:
            EV_DETAIL << "(26) FSM State CCA_3, EV_FRAME_RECEIVED:";
            if (useMACAcks) {
                EV_DETAIL << " setting up radio tx -> WAITSIFS." << endl;
                // suspend current transmission attempt,
                // transmit ack,
                // and resume transmission when entering manageQueue()
                transmissionAttemptInterruptedByRx = true;
                cancelEvent(ccaTimer);
                radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);
                updateMacState(WAITSIFS_6);
                startTimer(TIMER_SIFS);
            }
            else {
                EV_DETAIL << " Nothing to do." << endl;
            }
            decapsulate(check_and_cast<Packet *>(msg));
            sendUp(msg);
            break;

        case EV_BROADCAST_RECEIVED:
            EV_DETAIL << "(24) FSM State BACKOFF, EV_BROADCAST_RECEIVED:"
                      << " Nothing to do." << endl;
            decapsulate(check_and_cast<Packet *>(msg));
            sendUp(msg);
            break;

        default:
            fsmError(event, msg);
            break;
    }
}

void Ieee802154MacLoss::updateStatusTransmitFrame(t_mac_event event, cMessage *msg)
{
    if (event == EV_FRAME_TRANSMITTED && currentTxFrame != nullptr) {
//        delete msg;
        Packet *packet = currentTxFrame;
        bool expectAck = useMACAcks;
        const auto& csmaHeader = packet->peekAtFront<Ieee802154MacHeader>();


        if (!csmaHeader->getDestAddr().isBroadcast() && !csmaHeader->getDestAddr().isMulticast()) {
            // unicast
            EV_DETAIL << "(4) FSM State TRANSMITFRAME_4, "
                      << "EV_FRAME_TRANSMITTED [Unicast]: ";
        }
        else {
            // broadcast
            EV_DETAIL << "(27) FSM State TRANSMITFRAME_4, EV_FRAME_TRANSMITTED "
                      << " [Broadcast]";
            expectAck = false;
        }

        if (wakeUpRadio) {
            if (expectAck) // no sleep and awake the remote module
                wakeUpRadio->setModeControlled(IRadio::RADIO_MODE_RECEIVER);
            else
                wakeUpRadio->setWakeUpMode();
        }
        else
            radio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);

        if (expectAck) {
            EV_DETAIL << "RadioSetupRx -> WAITACK." << endl;
            updateMacState(WAITACK_5);
            startTimer(TIMER_RX_ACK);
        }
        else {
            EV_DETAIL << ": RadioSetupRx, manageQueue..." << endl;
            deleteCurrentTxFrame();
            manageQueue();
        }
        delete msg;
    }
    else {
        fsmError(event, msg);
    }
}

void Ieee802154MacLoss::updateStatusWaitAck(t_mac_event event, cMessage *msg)
{
    assert(useMACAcks);

    switch (event) {
        case EV_ACK_RECEIVED: {
            EV_DETAIL << "(5) FSM State WAITACK_5, EV_ACK_RECEIVED: "
                      << " ProcessAck, manageQueue..." << endl;
            if (rxAckTimer->isScheduled())
                cancelEvent(rxAckTimer);
            deleteCurrentTxFrame();
            txAttempts = 0;
            if (wakeUpRadio)
                wakeUpRadio->setWakeUpMode();
            delete msg;
            manageQueue();
            break;
        }
        case EV_ACK_TIMEOUT:
            EV_DETAIL << "(12) FSM State WAITACK_5, EV_ACK_TIMEOUT:"
                      << " incrementCounter/dropPacket, manageQueue..." << endl;
            manageMissingAck(event, msg);
            if (wakeUpRadio)
                wakeUpRadio->setWakeUpMode();
            break;

        case EV_BROADCAST_RECEIVED:
        case EV_FRAME_RECEIVED:
            decapsulate(check_and_cast<Packet *>(msg));
            sendUp(msg);
            break;

        case EV_DUPLICATE_RECEIVED:
            EV_DETAIL << "Error ! Received a frame during SIFS !" << endl;
            delete msg;
            break;

        default:
            fsmError(event, msg);
            if (wakeUpRadio)
                wakeUpRadio->setWakeUpMode();
            break;
    }
}
}

} // namespace inet

