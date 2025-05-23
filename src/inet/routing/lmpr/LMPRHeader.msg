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

import inet.common.INETDefs;
import inet.common.Units;
import inet.networklayer.contract.NetworkHeaderBase;
import inet.networklayer.common.L3Address;
import inet.networklayer.common.IpProtocolId;

cplusplus {{
#include "inet/common/ProtocolGroup.h"
}}

namespace inet;

class LMPRHeader extends NetworkHeaderBase
{
    L3Address    srcAddr;
    L3Address    destAddr;
    L3Address    prevAddr;
    L3Address 	 nextAddr;
    int           ttl = 1;    // time to live field
    int seqNum = 0; // sequence number
    IpProtocolId  protocolId;
//    B             payloadLengthField;
}

cplusplus(LMPRHeader) {{
  public:
    virtual L3Address getSourceAddress() const override { return L3Address(getSrcAddr()); }
    virtual void setSourceAddress(const L3Address& address) override { setSrcAddr(address); }
    virtual L3Address getDestinationAddress() const override { return L3Address(getDestAddr()); }
    virtual void setDestinationAddress(const L3Address& address) override { setDestAddr(address); }
    virtual const Protocol *getProtocol() const override { return ProtocolGroup::getIpProtocolGroup()->findProtocol(getProtocolId()); }
    virtual void setProtocol(const Protocol *protocol) override { setProtocolId(static_cast<IpProtocolId>(ProtocolGroup::getIpProtocolGroup()->getProtocolNumber(protocol))); }
    virtual bool isFragment() const override { return false; }
}}