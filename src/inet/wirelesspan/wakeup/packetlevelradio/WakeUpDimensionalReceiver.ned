//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.wirelesspan.wakeup.packetlevelradio;

import inet.physicallayer.wireless.common.base.packetlevel.NarrowbandReceiverBase;

module WakeUpDimensionalReceiver extends NarrowbandReceiverBase
{
    parameters:
        modulation = default("BPSK");
        errorModel.typename = default("ApskErrorModel");
        double minInterferencePower @unit(dBm);
        @class(physicallayer::WakeUpDimensionalReceiver);
}

