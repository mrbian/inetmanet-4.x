//
// Copyright (C) 2013 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "inet/flora/loraphy/LoRaMediumCache.h"

#include <algorithm>

#include "inet/flora/loraphy/LoRaLogNormalShadowing.h"
#include "inet/flora/loraphy/LoRaMedium.h"

namespace inet {

namespace flora {

Define_Module(LoRaMediumCache);


LoRaMediumCache::LoRaMediumCache():MediumLimitCache()
{
}

void LoRaMediumCache::initialize(int stage)
{
    MediumLimitCache::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        // check type
        const auto loraRadioMedium = dynamic_cast<const LoRaMedium *>(radioMedium);
        if (loraRadioMedium == nullptr)
            throw cRuntimeError("Radio medium is not of the type LoRaMedium, is type %s", typeid(radioMedium).name());
    }
}

std::ostream& LoRaMediumCache::printToStream(std::ostream& stream, int level, int evFlags) const
{
    stream << "LoRaMediumCache";
    if (level <= PRINT_LEVEL_TRACE)
        stream << EV_FIELD(minConstraintArea)
               << EV_FIELD(maxConstraintArea)
               << EV_FIELD(maxSpeed)
               << EV_FIELD(maxTransmissionPower)
               << EV_FIELD(minInterferencePower)
               << EV_FIELD(minReceptionPower)
               << EV_FIELD(maxAntennaGain)
               << EV_FIELD(minInterferenceTime)
               << EV_FIELD(maxTransmissionDuration)
               << EV_FIELD(maxCommunicationRange)
               << EV_FIELD(maxInterferenceRange);
    return stream;
}

} // namespace physicallayer

} // namespace inet

