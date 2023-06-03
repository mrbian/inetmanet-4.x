//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_UNITDISKMEDIUMANALOGMODELLOSS_H
#define __INET_UNITDISKMEDIUMANALOGMODELLOSS_H

#include "inet/physicallayer/wireless/common/analogmodel/unitdisk/UnitDiskMediumAnalogModel.h"

namespace inet {

namespace physicallayer {

/**
 * Implements the UnitDiskMediumAnalogModel model, see the NED file for details.
 */

class INET_API UnitDiskMediumAnalogModelLoss : public UnitDiskMediumAnalogModel
{
    typedef std::vector<int> Links;
    static std::map<int, Links> uniLinks;
    static std::map<int, Links> lossLinks;
    int hostId = -1;
    m communicationRange;

    bool checkLinkLoss(const IRadio *radio, const ITransmission *transmission) const;
  public:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int) override;
    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
    virtual const IReception *computeReception(const IRadio *radio, const ITransmission *transmission, const IArrival *arrival) const override;
};

} // namespace physicallayer

} // namespace inet

#endif

