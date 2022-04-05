package inet.examples.wirelesspan.wakeonradio;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.wirelesspan.nodes.WakeUpSensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.wireless.apsk.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.wireless.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.wireless.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.visualizer.common.IntegratedMultiVisualizer;
import inet.visualizer.common.IntegratedVisualizer;
import inet.wirelesspan.wakeup.packetlevelradio.WakeUpScalarRadioMedium;

network WakeOnRadioIeee802154
{
    @display("bgb=20,20;bgi=showcases/floorplan,s");

    **.wlanWake.radio.radioMediumModule = default("radioMedium2");
    submodules:
        sensor1: WakeUpSensorNode {
            @display("p=9.5,3.5;i=misc/sensor");
        }
        lamp1: WakeUpSensorNode {
            @display("p=8,5;i=status/bulb");
        }
        controller: WakeUpSensorNode {
            @display("p=6.5,10.5;i=misc/sensorgateway");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=18,0.75");
        }
        radioMedium: Ieee802154NarrowbandScalarRadioMedium {
            @display("p=18,2.25");
        }
        radioMedium2: WakeUpScalarRadioMedium {
            @display("p=18,8.32");
        }
        visualizer: IntegratedMultiVisualizer {
            @display("p=18,3.75");
        }
}
