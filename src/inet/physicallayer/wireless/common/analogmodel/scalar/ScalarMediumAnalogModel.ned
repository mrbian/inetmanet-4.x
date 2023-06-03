//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


package inet.physicallayer.wireless.common.analogmodel.scalar;

import inet.physicallayer.wireless.common.contract.packetlevel.IMediumAnalogModel;

//
// This analog model computes with scalar analog power representation. In other
// words, the signal power does not change over time or frequency, except for
// the boundaries of the signal.
//
module ScalarMediumAnalogModelLoss like IMediumAnalogModel
{
    parameters:
        double perUniLinks = default(0);
        double perLosLinks = default(0);
        double errorProb = default(0);
        bool  forceUni = default(true);
        bool checkMobility = default(false);
        bool ignorePartialInterference = default(false); // enabled means interference only partially overlapping with the signal being received is ignored, otherwise the simulation stops with an error
        @display("i=block/tunnel");
        @class(ScalarMediumAnalogModelLoss);
}

