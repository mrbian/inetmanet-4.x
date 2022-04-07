//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_WAKEUPDIMENSIONALTRANSMITTER_H
#define __INET_WAKEUPDIMENSIONALTRANSMITTER_H

#include "inet/physicallayer/wireless/common/base/packetlevel/DimensionalTransmitterBase.h"
#include "inet/physicallayer/wireless/common/base/packetlevel/FlatTransmitterBase.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

class INET_API WakeUpDimensionalTransmitter : public inet::physicallayer::FlatTransmitterBase, public inet::physicallayer::DimensionalTransmitterBase
{
  public:
    WakeUpDimensionalTransmitter();

    virtual void initialize(int stage) override;

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
    virtual const inet::physicallayer::ITransmission *createTransmission(const inet::physicallayer::IRadio *radio, const Packet *packet, const simtime_t startTime) const override;
};

} // namespace physicallayer
}
} // namespace inet

#endif

