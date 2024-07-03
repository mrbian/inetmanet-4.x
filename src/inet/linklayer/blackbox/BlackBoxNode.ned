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


package inet.linklayer.blackbox;

import inet.node.base.NodeBase;
import inet.node.contract.INetworkNode;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import inet.linklayer.blackbox.L2Queue;
import inet.linklayer.blackbox.Forwarding;
import inet.linklayer.contract.IPppInterface;        
import inet.common.MessageDispatcher;

module BlackBoxNode extends NodeBase like INetworkNode
{
    parameters:
        int numPppInterfaces = default(0);  // Minimum number of PPP interfaces
        int numBlaclBoxInterfaces = default(0);
        @backBoxNode;
        @display("bgb=297,621;i=device/accesspoint;is=vs");
    gates:
        inout pppg[numPppInterfaces] @labels(PppFrame-conn) @allowUnconnected;        
        inout blackbox[numBlaclBoxInterfaces] @allowUnconnected;      
    submodules:
        interfaceTable: InterfaceTable {
            @display("p=125,240;is=s");
        }
        li: MessageDispatcher {
            @display("p=750,900;b=1000,5,,,,1");
        }  
       forwarding : Forwarding;
       ppp[sizeof(pppg)]: <default("PppInterface")> like IPppInterface {
            @display("p=300,975,row,150;q=txQueue");
        }
       l2queue [sizeof(blackbox)]: L2Queue;
     connections allowunconnected:
        for i=0..sizeof(pppg)-1 {
            pppg[i] <--> { @display("m=s"); } <--> ppp[i].phys;
        }
        for i=0..sizeof(pppg)-1 {
            ppp[i].upperLayerOut --> li.in++;
            ppp[i].upperLayerIn <-- li.out++;
        }
        forwarding.outPpp --> li.in++;
        li.out++ --> forwarding.inPpp;   
}
