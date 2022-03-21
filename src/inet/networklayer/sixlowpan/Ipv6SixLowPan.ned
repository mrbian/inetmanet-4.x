//
// Copyright (C) 2005 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


//
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Copyright (c) 2013 Universita' di Firenze, Italy
// Author: Tommaso Pecorella <tommaso.pecorella@unifi.it>
//         Michele Muccio <michelemuccio@virgilio.it>
// Copyright (c) 2022 Universidad de Malaga, Spain
// Author: Alfonso Ariza aarizaq@uma.es
//

package inet.networklayer.sixlowpan;
import inet.networklayer.ipv6.Ipv6;

//
// Implements the IPv6 protocol.
//
// The IPv6 protocol header is represented by the ~Ipv6Header message class.
//
// <b>Interfacing with higher layer protocols</b>
//
// To send a packet over IPv6 from a higher layer protocol, the module should
// fill in an ~L3AddressReq object, attach it to the packet with Packet's
// addTag() method, then send the packet to the ~Ipv6 module.
//
// When ~Ipv6 sends up a packet to a higher layer protocol, it will also attach
// an ~L3AddressInd to the packet, with the source and destination IPv6 addresses
// of the ~Ipv6Header in which the packet arrived.
//
// ~Ipv6 can serve several higher-layer protocols. The higher layer protocols
// should call registerProtocol with their gate towards the IPv6 module,
// for fill up the protocol-to-gateindex map. When delivering packets to them,
// the output gate is determined from the Protocol field in the IPv6 header.
//
// <b>Routing and interfacing with lower layers</b>
//
// The routing table is stored in the module ~Ipv6RoutingTable. When a datagram
// needs to be routed, Ipv6 queries ~Ipv6RoutingTable for the output interface
// (or "port") and next hop address of the packet. This is done by directly
// calling C++ methods of ~Ipv6RoutingTable. No message exchange with ~Ipv6RoutingTable
// takes place.
//
//#FIXME describe operation
//
// Routing protocol implementations can also query and manipulate the route table
// by calling ~Ipv6RoutingTable's methods in C++.
//
// <b>Performance model, QoS</b>
//
// In the current form, ~Ipv6 contains a FIFO which queues up IPv6 frames;
// these are processed in order. The processing time is determined by the
// procDelay module parameter.
//
// @see ~Ipv6RoutingTable, ~Ipv6NeighbourDiscovery, ~Icmpv6
//
simple Ipv6SixLowPan extends Ipv6
{
    parameters:
        bool useIphc = default(true);
        bool OmitUDPchecksum = default(true);
        int FragmentReassemblyListSize = default(0); // unlimited
        double FragmentExpirationTimeout @unit(s) = default(60s);
        int CompressionThreshold @unit(B) = default(0B);
        bool UseMeshUnder = default(false);
        int MeshUnderRadius = default(10);
        int MeshCacheLength = default(10);
        volatile double MeshUnderJitter @unit(s) = default(uniform(10ms));
        string sixlowpanInterfaces = default("wlan0");
        string ContexCompresionList = default(""); // Format index?addrerss/prefix Length|allowed|valid time   example "0?fe80::8aa:ff:fe00/120|true|100 1?fa80::8aa:ff:fe00/110|true|200"
        bool aceptAllInterfaces = default(true); // accept all sixlowpan messages even if the interface is not sixlowpan
        int ipv6FragemtationMtu @unit(B) = default(B(-1)); //Only if the interface is sixlowpan, the value to determine to fragment the packet is std::max(mtu output interface, ipv6FragemtationMtu).  the six low pan will fragment the packet if the compressed packet is bigger than the MTU. This can reduce the IPV6 overhead (sveral ipv6 fragments can use the same ipv6 header)   
        @class(sixlowpan::Ipv6SixLowPan);
        @display("i=block/network2");
        @signal[packetDropped](type=inet::Packet);
        @signal[packetSentToUpper](type=inet::Packet);
        @signal[packetReceivedFromUpper](type=inet::Packet);
        @signal[packetSentToLower](type=inet::Packet);
        @signal[packetReceivedFromLower](type=inet::Packet);
}

