//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


package inet.physicallayer.wireless.common.analogmodel.unitdisk;

import inet.physicallayer.wireless.common.contract.packetlevel.IMediumAnalogModel;

//
// This analog model provides a very simple and predictable physical layer
// behavior. It determines the reception power by categorizing transmissions
// based on the distance between the transmitter and the receiver.
//
// @see ~UnitDiskRadioMedium, ~UnitDiskTransmitterAnalogModel, ~UnitDiskReceiverAnalogModel.
//
module UnitDiskMediumAnalogModelLoss like IMediumAnalogModel
{
    parameters:
        double perUniLinks = default(0);
        double perLosLinks = default(0);
        double errorProb = default(0);
        bool  forceUni = default(true);
        bool checkMobility = default(false);
        @class(UnitDiskMediumAnalogModelLoss);
        @display("i=block/tunnel");
}

