//
// Copyright (C) 2004 Andras Varga
// Copyright (C) 2000 Institut fuer Telematik, Universitaet Karlsruhe
// Copyright (C) 2011 Zoltan Bojthe
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

#ifndef __INET_UDPBASICBURST2_H
#define __INET_UDPBASICBURST2_H

#include <map>
#include <vector>

#include "inet/common/INETDefs.h"
#include "inet/applications/udpapp/UdpBasicBurst.h"

namespace inet {

/**
 * UDP application. See NED for more info.
 */
class INET_API UdpBasicBurst2 : public UdpBasicBurst
{

    uint64_t &numStatics = SIMULATION_SHARED_COUNTER(numStatics);
    uint64_t &numMobiles = SIMULATION_SHARED_COUNTER(numMobiles);

    std::vector<L3Address> &staticNodes = SIMULATION_SHARED_VARIABLE(staticNodes);
    uint64_t &packetStatic  = SIMULATION_SHARED_COUNTER(packetStatic);
    uint64_t &packetMob = SIMULATION_SHARED_COUNTER(packetMob);
    uint64_t &packetStaticRec = SIMULATION_SHARED_COUNTER(packetStaticRec);;
    uint64_t &packetMobRec = SIMULATION_SHARED_COUNTER(packetMobRec);;
    uint64_t &stablePaths = SIMULATION_SHARED_COUNTER(stablePaths);;
    cHistogram &delayHist  = SIMULATION_SHARED_VARIABLE(delayHist);
    bool isStatic = false;


  protected:
    // chooses random destination address
    virtual void processPacket(Packet *msg) override;
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void generateBurst() override;

  public:
    UdpBasicBurst2() {}

};

} // namespace inet

#endif // ifndef __INET_UDPBASICBURST_H

