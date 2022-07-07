//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_FLATRADIOBASEPOWERLEVELS_H
#define __INET_FLATRADIOBASEPOWERLEVELS_H

#include "inet/physicallayer/wireless/common/base/packetlevel/NarrowbandRadioBase.h"
#include "StateBasedCcEnergyConsumerLevel.h"
#include "StateBasedEpEnergyConsumerLevel.h"

namespace inet {

namespace physicallayer {

class INET_API FlatRadioBasePowerLevels : public NarrowbandRadioBase
{
    struct EnergyData {
        W power;
        A consumption;
        W consumptionEp;
    };
    std::vector<EnergyData> powerVector;

    StateBasedCcEnergyConsumerLevel *energyConsumerCc = nullptr;
    StateBasedEpEnergyConsumerLevel *energyConsumerEp = nullptr;
  protected:
    virtual void handleUpperCommand(cMessage *message) override;
    virtual void initialize(int stage) override;
    virtual void parseLevels();

  public:
    FlatRadioBasePowerLevels();
    virtual void setPowerLevel(int level);
    virtual void setPower(W newPower);
    virtual void setBitrate(bps newBitrate);

};

} // namespace physicallayer

} // namespace inet

#endif

