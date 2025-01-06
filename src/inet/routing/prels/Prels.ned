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

package inet.routing.prels;

import inet.networklayer.base.NetworkProtocolBase;
import inet.networklayer.contract.INetworkProtocol;

simple Prels extends NetworkProtocolBase like INetworkProtocol
{
    parameters:
        @display("i=block/routing");
        @class(Prels);	

		string arpModule;

        double Hello_ival @unit(s) = default(2s); // (s) HELLO messages' emission interval. (section 6.2)
        double Tc_ival @unit(s) = default(5s); // (s) Topology Control (TC) messages' emission interval. (section 9.3)

        @signal[mobInfoAverLapse](type=double);
        @statistic[mobInfoAverLapse](title="mobility info average lapse"; record=histogram; interpolationmode=none);
        @signal[mobInfoMaxLapse](type=double);
        @statistic[mobInfoMaxLapse](title="mobility info max lapse"; record=histogram; interpolationmode=none);
        @signal[mobInfoMinLapse](type=double);
        @statistic[mobInfoMinLapse](title="mobility info min lapse"; record=histogram; interpolationmode=none);
        @signal[mobInfoCount](type=int);
        @statistic[mobInfoCount](title="mobility info length"; record=vector; interpolationmode=none);
        
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