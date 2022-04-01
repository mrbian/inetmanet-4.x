//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_WAKEUPRECEIVERBASE_H
#define __INET_WAKEUPRECEIVERBASE_H

#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpBandListening.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatReceiverBase.h"

namespace inet {

namespace physicallayer {

class INET_API WakeUpReceiverBase : public FlatReceiverBase
{
  protected:
    W minInterferencePower;
    std::vector<FreqItem> bandwithList;
    

  public:
    WakeUpReceiverBase();

    void initialize(int stage) override;

    const IListening *createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord& startPosition, const Coord& endPosition) const override;

    bool computeIsReceptionPossible(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part) const override;

    bool computeIsReceptionPossible(const IListening *listening, const ITransmission *transmission) const override;

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;

    virtual W getMinInterferencePower() const override { return minInterferencePower; }
};

} // namespace physicallayer

} // namespace inet

#endif

