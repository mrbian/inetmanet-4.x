//
// This program is property of its copyright holder. All rights reserved.
//

import inet.common.INETDefs;
import inet.common.TagBase;
import inet.common.Units;
import inet.networklayer.common.L3Address;

namespace inet::inetmanet;

class DeepFirstForwardTag extends TagBase
{
    uint64_t seqNumber;
    bool dupFlag;
    bool retFlag;
}

class RetransmitTag extends TagBase
{
    int numRetransmissions = 1;
    L3Address retransmitNode;
}