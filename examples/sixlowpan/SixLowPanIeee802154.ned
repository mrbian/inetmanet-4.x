package inet.examples.sixlowpan;

import inet.networklayer.configurator.ipv6.Ipv6FlatNetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.networklayer.sixlowpan.SensorNode6lowpan;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.wireless.apsk.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.wireless.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.wireless.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.visualizer.common.IntegratedMultiVisualizer;
import inet.visualizer.common.IntegratedVisualizer;


network SixLowPanIeee802154
{
    @display("bgb=20,20;bgi=showcases/floorplan,s");
    submodules:
        sensor1: SensorNode6lowpan {
            @display("p=9.5,3.5;i=misc/sensor");
        }
        sensor2: SensorNode6lowpan {
            @display("p=9.5,16.5;i=misc/sensor");
        }
        sensor3: SensorNode6lowpan {
            @display("p=10.75,10.75;i=misc/sensor");
        }
        sensor4: SensorNode6lowpan {
            @display("p=2.5,17.5;i=misc/sensor");
        }
        sensor5: SensorNode6lowpan {
            @display("p=2.5,7;i=misc/sensor");
        }
        lamp1: SensorNode6lowpan {
            @display("p=8,5;i=status/bulb");
        }
        lamp2: SensorNode6lowpan {
            @display("p=8.75,13.5;i=status/bulb");
        }
        lamp3: SensorNode6lowpan {
            @display("p=13.5,8.5;i=status/bulb");
        }
        lamp4: SensorNode6lowpan {
            @display("p=16.5,9.5;i=status/bulb");
        }
        lamp5: SensorNode6lowpan {
            @display("p=13.5,14;i=status/bulb");
        }
        lamp6: SensorNode6lowpan {
            @display("p=4,13;i=status/bulb");
        }
        lamp7: SensorNode6lowpan {
            @display("p=4,10.5;i=status/bulb");
        }
        lamp8: SensorNode6lowpan {
            @display("p=4,5;i=status/bulb");
        }
        controller: SensorNode6lowpan {
            @display("p=6.5,10.5;i=misc/sensorgateway");
        }
        configurator: Ipv6FlatNetworkConfigurator {
            @display("p=18,0.75");
        }
        radioMedium: Ieee802154NarrowbandScalarRadioMedium {
            @display("p=18,2.25");
        }
        visualizer: IntegratedMultiVisualizer {
            @display("p=18,3.75");
        }
}
