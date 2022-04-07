//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpScalarReceiver.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

using namespace inet::physicallayer;
Define_Module(WakeUpScalarReceiver);

WakeUpScalarReceiver::WakeUpScalarReceiver() :
        WakeUpReceiverBase()
{
}


void WakeUpScalarReceiver::initialize(int stage)
{
    WakeUpReceiverBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        minInterferencePower = mW(math::dBmW2mW(par("minInterferencePower")));
    }
}

std::ostream& WakeUpScalarReceiver::printToStream(std::ostream& stream, int level, int evFlags) const
{
    stream << "WakeUpScalarReceiver";
    return FlatReceiverBase::printToStream(stream, level);
}

} // namespace physicallayer
}
} // namespace inet

