//
// Copyright (C) 2010 Alfonso Ariza
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

package inet.examples.manetrouting.net80211_aodv;

import inet.common.scenario.ScenarioManager;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.environment.common.PhysicalEnvironment;
import inet.visualizer.canvas.integrated.IntegratedCanvasVisualizer;

import parrot.node.parrot_host;
//import parrot.node.parrot_host;

//
// Example network for testing aodv routing. Contains
// fixed and mobile nodes. Mobile nodes are not moving,
// but are scattered randomly on the playground.
//
network Net80211_aodv
{
    parameters:
        int numServers;
        int numClients;
        int numRouters;
    submodules:
        radioMedium: Ieee80211ScalarRadioMedium {
            parameters:
                @display("p=60,50;i=misc/sun");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                config = xml("<config><interface hosts='*' address='145.236.x.x' netmask='255.255.0.0'/></config>");
                @display("p=140,50;i=block/cogwheel_s");
        }        
        // must before server, for finish() clean destAddress first 
        client[numClients]: AdhocHost {
            parameters:
                @display("i=device/pocketpc_s;r=,,#707070");
        }
        server[numServers]: AdhocHost {
            parameters:
                @display("i=device/pocketpc_s;r=,,#707070");
        }
        router[numRouters]: AdhocHost {
            parameters:
                @display("i=device/pocketpc_s;r=,,#707070");
        }
		physicalEnvironment: PhysicalEnvironment {
            parameters:
                @display("p=60,50");
        }
        visualizer: IntegratedCanvasVisualizer {
            @display("p=500,240");
        }

    connections allowunconnected:
}

network Net80211_parrot
{
    parameters:
        int numServers;
        int numClients;
        int numRouters;
    submodules:
        radioMedium: Ieee80211ScalarRadioMedium {
            parameters:
                @display("p=60,50;i=misc/sun");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
                config = xml("<config><interface hosts='*' address='145.236.x.x' netmask='255.255.0.0'/></config>");
                @display("p=140,50;i=block/cogwheel_s");
        }        
        client[numClients]: parrot_host {
            parameters:
                @display("i=device/pocketpc_s;r=,,#707070");
        }
        server[numServers]: parrot_host {
            parameters:
                @display("i=device/pocketpc_s;r=,,#707070");
        }
        router[numRouters]: parrot_host {
            parameters:
                @display("i=device/pocketpc_s;r=,,#707070");
        }
		physicalEnvironment: PhysicalEnvironment {
            parameters:
                @display("p=60,50");
        }
        visualizer: IntegratedCanvasVisualizer {
            @display("p=500,240");
        }

    connections allowunconnected:
}



