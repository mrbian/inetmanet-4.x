//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.wirelesspan.wakeup.packetlevelradio;

import inet.physicallayer.wireless.common.base.packetlevel.NarrowbandTransmitterBase;

module WakeUpScalarTransmitter extends NarrowbandTransmitterBase
{
    parameters:
        modulation = default("BPSK");
        @class(physicallayer::WakeUpScalarTransmitter);
}

