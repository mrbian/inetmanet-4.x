//
// Copyright (C) 2014 OpenSim Ltd.
// Author: Benjamin Seregi
// Copyright (C) 2019 Universidad de Malaga
// Author: Alfonso Ariza
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "inet/wirelesspan/routing/loadng/LoadNgRouteData.h"

namespace inet {
namespace wirelesspan {
namespace routing {

std::string LoadNgRouteData::str() const
{
    std::ostringstream out;
    out << "isActive = " << isActive()
        << ", hasValidDestNum = " << hasValidDestNum()
        << ", destNum = " << this->getDestSeqNum()
        << ", lifetime = " << getLifeTime();

    return out.str();
};

}
}
} // namespace inet

