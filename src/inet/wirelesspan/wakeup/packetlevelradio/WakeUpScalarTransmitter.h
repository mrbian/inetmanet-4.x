//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_WAKEUPSCALARTRANSMITTER_H
#define __INET_WAKEUPSCALARTRANSMITTER_H

#include "inet/physicallayer/wireless/common/base/packetlevel/FlatTransmitterBase.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

class INET_API WakeUpScalarTransmitter : public inet::physicallayer::FlatTransmitterBase
{
  public:
    WakeUpScalarTransmitter();

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
    virtual const inet::physicallayer::ITransmission *createTransmission(const inet::physicallayer::IRadio *radio, const Packet *packet, const simtime_t startTime) const override;
};

} // namespace physicallayer
}
} // namespace inet

#endif

