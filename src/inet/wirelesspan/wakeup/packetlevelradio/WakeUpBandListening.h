//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#ifndef __INET_WAKEUPBANDLISTENING_H
#define __INET_WAKEUPBANDLISTENING_H

#include "inet/common/Units.h"
#include "inet/physicallayer/wireless/common/radio/packetlevel/BandListening.h"

namespace inet {
namespace wirelesspan {
namespace physicallayer {

using namespace inet::units::values;

class FreqItem {
    Hz centerFrequency;
    Hz bandwidth;
public:
    Hz getCenterFrequency() const {return centerFrequency;}
    Hz getBandwidth() const {return bandwidth;}
    void setCenterFrequency(Hz p) {centerFrequency = p;}
    void setBandwidth(Hz p) {bandwidth = p;}
};

class INET_API WakeUpBandListening : public inet::physicallayer::BandListening
{

  protected:
    std::vector<FreqItem> bandwithList;

  public:
    WakeUpBandListening(const inet::physicallayer::IRadio *radio, simtime_t startTime, simtime_t endTime, Coord startPosition, Coord endPosition, Hz centerFrequency, Hz bandwidth, std::vector<FreqItem> b);

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
    virtual std::vector<FreqItem> getBandList() const {return bandwithList;}
};

} // namespace physicallayer
}
} // namespace inet

#endif

