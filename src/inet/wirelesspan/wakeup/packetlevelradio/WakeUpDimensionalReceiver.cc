//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpDimensionalReceiver.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

using namespace inet::physicallayer;

Define_Module(WakeUpDimensionalReceiver);

WakeUpDimensionalReceiver::WakeUpDimensionalReceiver() :
        WakeUpReceiverBase()
{
}

void WakeUpDimensionalReceiver::initialize(int stage)
{
    WakeUpReceiverBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        minInterferencePower = mW(math::dBmW2mW(par("minInterferencePower")));
    }
}



std::ostream& WakeUpDimensionalReceiver::printToStream(std::ostream& stream, int level, int evFlags) const
{
    stream << "WakeUpDimensionalReceiver";
    return FlatReceiverBase::printToStream(stream, level);
}

} // namespace physicallayer
}
} // namespace inet

