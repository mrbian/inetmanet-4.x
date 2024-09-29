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

package inet.routing.lmpr;

//import inet.routing.contract.IManetRouting;
//import inet.protocolelement.contract.IProtocolLayer;

import inet.networklayer.base.NetworkProtocolBase;
import inet.networklayer.contract.INetworkProtocol;

simple LMPR extends NetworkProtocolBase like INetworkProtocol
{
    parameters:
        @display("i=block/routing");
        @class(LMPR);
        
        @signal[nodeInfoLen](type=int);
        @statistic[nodeInfoLen](title="cached node info length"; record=vector; interpolationmode=none);
	
		// Modules
//        string interfaceTableModule;   // The path to the InterfaceTable module
//        string routingTableModule;
//        string networkProtocolModule = default("^.ipv4.ip");
		string arpModule;
        
        // Routing
        double mhOGMInterval @unit(s) = default(1s);
        double neighborReliabilityTimeout @unit(s) = default(1s);
//        double txRange @unit(m) = default(76.3598m);
		bool setAutoRange = default(false);
		double losMapError = default(0.0);
        double maxRangeForLET @unit(m) = default(32.49m);
        double predictDuration @unit(s) = default(0.5s);
                
        // Basic Configureation
        string netmask = default("255.255.0.0");  // netmask for the ad-hoc network
        double maxJitter = default(0.1); // This is the maximum of a random value to determine when the first hello message will be sent out 
        volatile double broadcastDelay @unit(s) = default(uniform(0s,0.01s));
        
        int bcMaxEntries = default(100);
        double bcDelTime @unit(s) = default(100 s);
        int defaultTTL = default(10);
        
    gates:
//        input ipIn;
//        output ipOut;
//        input lowerlayerIn;
//        output lowerlayerOut;
//        input upperLayerIn;
//        output upperLayerOut;
//        input lowerLayerIn;
//        output lowerLayerOut;
}
