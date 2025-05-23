//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_IPATHLOSS_H
#define __INET_IPATHLOSS_H

#include "inet/physicallayer/wireless/common/contract/packetlevel/IArrival.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/ITransmission.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadio.h"

namespace inet {

namespace physicallayer {

typedef struct{
    double txX;
    double txY;
    double rxX;
    double rxY;
    int losCondFlag;
} LOSCond;

/**
 * This interface models path loss (or path attenuation) that is the reduction
 * in power density of a radio signal as it propagates through space.
 */
class INET_API IPathLoss : public virtual IPrintableObject
{
  public:
    /**
     * Returns the loss factor for the provided transmission and arrival.
     * The value is in the range [0, 1] where 1 means no loss at all and 0
     * means all power is lost.
     */
    virtual double computePathLoss(const ITransmission *transmission, const IArrival *arrival) const = 0;

    /**
     * Returns the loss factor as a function of propagation speed, carrier
     * frequency and distance. The value is in the range [0, 1] where 1 means
     * no loss at all and 0 means all power is lost.
     */
    virtual double computePathLoss(mps propagationSpeed, Hz frequency, m distance) const = 0;

    /**
     * Returns the range for the given loss factor. The value is in the range
     * [0, +infinity) or NaN if unspecified.
     */
    virtual m computeRange(mps propagationSpeed, Hz frequency, double loss) const = 0;

    virtual LOSCond* getLOSCondByTxId(int tranmission_id, int rx_id) const
    {
        return nullptr;
    }

    virtual double computePathLoss(const IRadio *receiverRadio, const ITransmission *transmission, const IArrival *arrival) const
    {
        throw cRuntimeError("Should not use PathLossBase::computePathLoss(receiverRadio)!");
    }

};

} // namespace physicallayer

} // namespace inet

#endif

