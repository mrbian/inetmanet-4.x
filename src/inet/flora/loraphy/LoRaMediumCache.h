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

#ifndef LORAPHY_LORAMEDIUMCACHE_H
#define LORAPHY_LORAMEDIUMCACHE_H
#include "inet/physicallayer/wireless/common/medium/MediumLimitCache.h"

namespace inet {

namespace flora {
using namespace physicallayer;

class INET_API LoRaMediumCache : public MediumLimitCache
{
  protected:
    virtual void initialize(int stage) override;
  public:
    LoRaMediumCache();
    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;
};

} // namespace physicallayer

} // namespace inet

#endif // ifndef LORAPHY_LORAMEDIUMCACHE_H

