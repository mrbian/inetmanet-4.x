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

#include "BlackBoxRouting.h"
#include "Forwarding.h"
#include "inet/common/DijktraKShortest.h"

namespace inet {
namespace blackbox {
Define_Module(BlackBoxRouting);

simsignal_t BlackBoxRouting::updateTopology = registerSignal("blackBoxUpdateTopology");

BlackBoxRouting::BlackBoxRouting() {
    // TODO Auto-generated constructor stub

}

BlackBoxRouting::~BlackBoxRouting() {
    // TODO Auto-generated destructor stub
}

void BlackBoxRouting::initialize(int stage)
{

}

void BlackBoxRouting::handleMessage(cMessage *msg)
{

}

void BlackBoxRouting::modifyTopology()
{
    auto &topology = Forwarding::getLogicalTopologyMapForUpdate();
    Dijkstra dj;
    emit(updateTopology, this);
}



}
} /* namespace inet */
