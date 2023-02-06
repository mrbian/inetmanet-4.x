//
// Copyright (C) 2016 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


package inet.flora.lorabase;

import inet.linklayer.csmaca.CsmaCaMac;


//
// Implements an imaginary CSMA/CA-based MAC protocol with optional
// acknowledgements and a retry mechanism. With the appropriate settings,
// it can approximate basic 802.11b ad-hoc mode operation.
//
// Parameters include:
// - acknowledgements on/off
// - bit rate (this is used for both data and ACK frames)
// - protocol overhead: MAC header length, ACK frame length
// - backoff parameters: minimum/maximum contention window (in slots),
//   slot time, maximum retry count
// - timing: interval to wait before transmitting ACK frame (SIFS) and
//   before data frames in addition to the backoff slots (DIFS)
//
// When acknowledgements are turned off, the MAC assumes that its transmissions
// are successful, so no frame is ever retransmitted.
//
// When acknowledgements are turned on, the operation is as follows.
// On the receiver side, the change is quite simple: when the MAC correctly
// receives a data frame addressed to it, it responds with an ACK frame after
// a fixed-length gap (SIFS). If the originator of the data frame does not
// receive the ACK correctly within due time, it will initiate a
// retransmission. The contention window (from which the random backoff period is
// drawn) will be doubled for each retransmission until it reaches the maximum
// (and then it will stay constant for further retransmissions). After a given
// number of unsuccessful retries, the MAC will give up and discard the data
// frame, and will take the next data frame from the queue. The next frame
// will start with a clean slate (i.e. the contention window and the retry
// count will be reset). Giving up a frame will cause a link break signal
// to be emitted.
//
// Note: This module does not contain a duplicate detection algorithm in order
// to keep its code simple and accessible, so lost ACKs will cause duplicates
// in the stream of packets sent up to higher layers.
//
module CsmaCaMacLora extends CsmaCaMac
{
    parameters:
        useAck = false; // Lora never send ACK, must be the upper layers.
        @class(CsmaCaMacLora);
}

