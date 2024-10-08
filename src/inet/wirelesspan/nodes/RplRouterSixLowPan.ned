// 
//   Simulation model for RPL (Routing Protocol for Low-Power and Lossy Networks)
//  
//   Copyright (C) 2021  Institute of Communication Networks (ComNets),
//                       Hamburg University of Technology (TUHH)
//             (C) 2021  Yevhenii Shudrenko
//  
//   This program is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//  
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//  
//   You should have received a copy of the GNU General Public License
//   along with this program.  If not, see <https://www.gnu.org/licenses/>.
//  

package inet.wirelesspan.nodes;

import inet.node.inet.AdhocHost;
import inet.wirelesspan.routing.rpl.Rpl;
import inet.wirelesspan.routing.rpl.TrickleTimer;

module RplRouterSixLowPan extends AdhocHost
{
    parameters:
        forwarding = true;
        hasIpv4 = default(false);
        hasIpv6 = default(true);
        
        ipv6.typename = default("SixLowPanNetworkLayer");
        ipv6.neighbourDiscovery.typename = default("WpanIpv6NeighbourDiscovery");
        *.routingTableModule = default("^.ipv6.routingTable");
        *.routingTableModule = default("^.ipv6.routingTable");
        *.networkProtocolModule = default("^.ipv6.ipv6");
    submodules:
        rpl: Rpl {
            @display("p=825,226");
        }
        trickleTimer: TrickleTimer {
            @display("p=946.57495,225.22499");
        }
    connections:
        rpl.ipOut --> tn.in++;
        rpl.ipIn <-- tn.out++;
        rpl.ttModule <--> trickleTimer.rpModule;
}

