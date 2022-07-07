//
// Copyright (C) 2022 Universidad de Malaga.
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

#include "inet/common/WirelessNumNodes.h"
#include "inet/common/ModuleAccess.h"
#include "inet/mobility/contract/IMobility.h"

namespace inet{

WirelessNumNodes::WirelessNumNodes()
{
    cTopology topo("topo");
    topo.extractByProperty("networkNode");
    vectorList.clear();
    for (int i = 0; i < topo.getNumNodes(); i++) {
        cTopology::Node *node = topo.getNode(i);
        IMobility *mod;
        cModule *host = node->getModule();
        mod = check_and_cast<IMobility *>(host->getSubmodule("mobility"));
        if (mod == nullptr)
            throw cRuntimeError("node or mobility module not found");
        vectorList.push_back(mod);
    }
}


int WirelessNumNodes::getNumNodesCenter(const Coord & center, const double &distance) const
{
    int total = 0;
    for (const auto &elem : vectorList) {
        if (elem->getCurrentPosition().distance(center) <= distance)
            total++;
    }
    return total;
}

int WirelessNumNodes::getNumNodesArea(const Coord & upCorner, const Coord & lowerCorner) const
{

    int total = 0;
    if (upCorner.x > lowerCorner.x)
        throw cRuntimeError("upCorner.x > lowerCorner.x");
    if (upCorner.y > lowerCorner.y)
        throw cRuntimeError("upCorner.y > lowerCorner.y");
    for (const auto &elem : vectorList) {
        auto pos = elem->getCurrentPosition();
        if (upCorner.x < pos.x && pos.x < lowerCorner.x && upCorner.y < pos.y && pos.y < lowerCorner.y)
            total++;
    }
    return total;
}

}



