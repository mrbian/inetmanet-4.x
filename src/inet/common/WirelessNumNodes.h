//
// Copyright (C) 2012 Univerdidad de Malaga.
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

#ifndef __INET_WIRELESSNUMNODES_H_
#define __INET_WIRELESSNUMNODES_H_

#include <vector>
#include "inet/common/geometry/common/Coord.h"

namespace inet{

class IMobility;

class WirelessNumNodes : public cOwnedObject
{
    std::vector<IMobility*> vectorList;
public:
    virtual int getNumNodesCenter(const Coord & center, const double &distance) const;
    virtual int getNumNodesArea(const Coord & upCorner, const Coord & lowerCorner) const;
    WirelessNumNodes();
    virtual ~WirelessNumNodes() {vectorList.clear();}
};

}

#endif /* WIRELESSNUMHOPS_H_ */
