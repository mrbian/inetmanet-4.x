//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.wirelesspan.wakeup.packetlevelradio;

import inet.linklayer.contract.INetworkInterface;
import inet.linklayer.contract.IWirelessInterface;
import inet.networklayer.common.NetworkInterface;
import inet.physicallayer.wireless.common.contract.packetlevel.IRadio;
import inet.queueing.contract.IPacketQueue;
import inet.linklayer.ieee802154.Ieee802154NarrowbandMac;
import inet.wirelesspan.linklayer.ieee802154Loss.Ieee802154MacLoss;
import inet.wirelesspan.linklayer.ieee802154Loss.Ieee802154LossNarrowbandMac;
import inet.wirelesspan.wakeup.packetlevelradio.WakeUpRadioBase;

//
// This module implements an IEEE 802.15.4 narrowband network interface.
//
//module WakeUpIeee802154NarrowbandInterface extends NetworkInterface like IWirelessInterface
module WakeUpIeee802154NarrowbandInterface extends NetworkInterface like INetworkInterface
{
    parameters:
        @display("i=block/ifcard");
        @class(NetworkInterface);
        string interfaceTableModule;
        string energySourceModule = default("");
        *.interfaceTableModule = default(absPath(this.interfaceTableModule));
        *.energySourceModule = default(absPath(this.energySourceModule));
    gates:
        input upperLayerIn;
        output upperLayerOut;
        input radioIn[2] @labels(IWirelessSignal);
    submodules:
        queue: <default("DropTailQueue")> like IPacketQueue {
            parameters:
                packetCapacity = default(100);
                @display("p=200,100;q=l2queue");
        }
        mac: Ieee802154LossNarrowbandMac {
            parameters:
                @display("p=300,100");
        }
        controlledRadio: <default("Ieee802154NarrowbandScalarRadio")> like IRadio {
            parameters:
                @display("p=222,208");
        }
        radio: WakeUpRadioBase {
            parameters:
                @display("p=300,208");
        }
    connections:
        upperLayerIn --> { @display("m=n"); } --> queue.in;
        queue.out --> mac.upperLayerIn;
        mac.lowerLayerOut --> radio.upperLayerIn;

        radioIn[0] --> { @display("m=s"); } --> radio.radioIn;
        radioIn[1] --> { @display("m=s"); } --> controlledRadio.radioIn;
        controlledRadio.upperLayerIn <-- radio.toControlled;
        controlledRadio.upperLayerOut --> radio.fromControlled;
        radio.upperLayerOut --> mac.lowerLayerIn;
        mac.upperLayerOut --> { @display("m=n"); } --> upperLayerOut;
}

