//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpBandListening.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

using namespace inet::physicallayer;

WakeUpBandListening::WakeUpBandListening(const IRadio *radio, simtime_t startTime, simtime_t endTime, Coord startPosition, Coord endPosition, Hz centerFrequency, Hz bandwidth, std::vector<FreqItem> b) :
        BandListening(radio, startTime, endTime, startPosition, endPosition,centerFrequency,bandwidth),
    bandwithList(b)
{
}

std::ostream& WakeUpBandListening::printToStream(std::ostream& stream, int level, int evFlags) const
{
    stream << "WakeUpBandListening";
    if (level <= PRINT_LEVEL_DETAIL)
        stream << EV_FIELD(centerFrequency)
               << EV_FIELD(bandwidth);
    return ListeningBase::printToStream(stream, level);
}

} // namespace physicallayer
}
} // namespace inet

