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
import inet.common.packet.chunk.Chunk;
import inet.networklayer.common.L3Address;
import inet.routing.prels.Prels_msg;

//
// Represents an OLSR packet
//
// Uses the following cMessage fields:
//    - length () / setLength(err)  in bits
//    - hasBitError() / setBitError()
//    - timestamp() / setTimestamp (simtime) used in timestamp option
//
// Additional length fields defined in this class are in bytes
// (totalLength()=length()/8 and header_length), or 8 bytes (fragment_offset).
//
// Only one of the option fields can exist at a time.
//
//class noncobject OLSR_msg;

namespace inet;

class PrelsPkt extends FieldsChunk
{
    @omitGetVerb(true);
    @fieldNameSuffix("_var");
    bool reduceFuncionality=false;
    short pkt_seq_num;          ///< Packet sequence number.
    long sn;                    /// CapProbe packet sequence number
    double send_time;           /// Packet sending timestamp
    PrelsMsg msg[];             ///< Packet body.
}
