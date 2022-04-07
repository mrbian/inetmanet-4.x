//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.wirelesspan.wakeup.packetlevelradio;

import inet.common.LayeredProtocolBase;
import inet.physicallayer.wireless.common.base.packetlevel.PhysicalLayerBase;
import inet.physicallayer.wireless.common.contract.packetlevel.IAntenna;
import inet.physicallayer.wireless.common.contract.packetlevel.IRadio;
import inet.physicallayer.wireless.common.contract.packetlevel.IReceiver;
import inet.physicallayer.wireless.common.contract.packetlevel.ITransmitter;
import inet.power.contract.IEnergyConsumer;

module WakeUpRadioBase extends LayeredProtocolBase like IRadio
{
    parameters:

        string radioMediumModule = default("radioMedium"); // module path of the medium module where this radio communicates
        string energySourceModule = default("");           // module path of the energy source module which provides energy to the radio

        string initialRadioMode @enum("off","sleep","receiver","transmitter","transceiver") = default("off");
        string switchingTimes = default("ms 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"); // time parameters to switch between radio modes

        bool sendRawBytes = default(false); // when true packets are serialized into a sequence of bytes before sending out

        bool separateTransmissionParts = default(false); // when enabled the transmission of preamble, header and data part are simulated separately
        bool separateReceptionParts = default(false);    // when enabled the reception of preamble, header and data part are simulated separately

        bool displayCommunicationRange = default(false); // if true communication range is displayed as a blue circle around the node
        bool displayInterferenceRange = default(false);  // if true interference range is displayed as a gray circle around the node

        *.energySourceModule = default(absPath(this.energySourceModule));

        @signal[packetSentToUpper](type=cPacket);
        @signal[packetReceivedFromUpper](type=cPacket);
        @signal[radioModeChanged](type=long);
        @signal[listeningChanged];
        @signal[receptionStateChanged](type=long);
        @signal[transmissionStateChanged](type=long);
        @signal[receivedSignalPartChanged](type=long);
        @signal[transmittedSignalPartChanged](type=long);
        @signal[transmissionStarted];
        @signal[transmissionEnded];
        @signal[receptionStarted];
        @signal[receptionEnded];

        @statistic[radioMode](title="Radio mode"; source=radioModeChanged; record=count,vector; interpolationmode=sample-hold);
        @statistic[receptionState](title="Radio reception state"; source=receptionStateChanged; record=count,vector; interpolationmode=sample-hold);
        @statistic[transmissionState](title="Radio transmission state"; source=transmissionStateChanged; record=count,vector; interpolationmode=sample-hold);

        @statistic[minSnir](title="Min SNIR"; source=minimumSnir(packetSentToUpper); record=histogram);
        @statistic[packetErrorRate](title="Packet error rate"; source=packetErrorRate(packetSentToUpper); record=histogram);
        @statistic[bitErrorRate](title="Bit error rate"; source=bitErrorRate(packetSentToUpper); record=histogram);
        @statistic[symbolErrorRate](title="Symbol error rate"; source=symbolErrorRate(packetSentToUpper); record=histogram);

        double centerFrequency @unit(Hz) = default(433 MHz);     // center frequency of the band where the radio transmits and receives signals on the medium
        double bandwidth @unit(Hz) = default(1 kHz);            // bandwidth of the band where the radio transmits and receives signals on the medium
        *.centerFrequency = default(this.centerFrequency); // passed down to transmitter and receiver by default
        *.bandwidth = default(this.bandwidth);               // passed down to transmitter and receiver by default

        string signalAnalogRepresentation @enum("scalar","dimensional") = default("scalar");
        transmitter.typename = default(signalAnalogRepresentation == "scalar" ? "WakeUpScalarTransmitter" : signalAnalogRepresentation == "dimensional" ? "WakeUpDimensionalTransmitter" : "");
        receiver.typename = default(signalAnalogRepresentation == "scalar" ? "WakeUpScalarReceiver" : signalAnalogRepresentation == "dimensional" ? "WakeUpDimensionalReceiver" : "");


        // 802.15.4-2006 (page 28)
        *.bitrate = default(250 kbps);

        // PHY Header (without preamble), 802.15.4-2006 (page 43)
        // 1 octet SFD
        // 7 bit Frame length
        // 1 bit Reserved
        *.headerLength = 0 b;

        // Preamble
        // 4 octets Preamble
        // 1 symbol of 16us -> 4 bit
        transmitter.preambleDuration = 0 us;

        // RSSI sensitivity (ATmega256RFR2, page 566)
        receiver.energyDetection = default(-90dBm);

        // Receiver sensitivity (ATmega256RFR2, page 565)
        // TODO That is not quite true, because sensitivity
        //      is defined as the input signal power that yields
        //      a PER < 1% for a PSDU of 20 octets, but INET handles it
        //      as minimum reception power.
        receiver.sensitivity = default(-100dBm);

        // There is no fixed boundary, because of the
        // DSSS and the capture effect. Taking the sensitivity minus some
        // arbitrary value as an approximate guess.
        receiver.minInterferencePower = default(-120dBm);

        // Minimum SNIR
        // -8 dB results into 98% PER for a PSDU of 20 octets
        receiver.snirThreshold = default(-8 dB);

        // TX Output power (typ. 3.5 dBm, ATmega256RFR2, page 564)
        transmitter.power = default(2.24mW);

        double interval @unit(s) = default(33ms);
        double scanTime @unit(s) = default(0.1ms);
        @class(wirelesspan::physicallayer::WakeUpRadioBase);

        @display("i=block/wrxtx");
    gates:
        input upperLayerIn @labels(ILinkLayerFrame/down);
        output upperLayerOut @labels(ILinkLayerFrame/up);
        input fromControlled @labels(ILinkLayerFrame/up);
        output toControlled @labels(ILinkLayerFrame/down);
        input radioIn @labels(IWirelessSignal);

    submodules:
        antenna: <default("IsotropicAntenna")> like IAntenna {
            parameters:
                @display("p=100,100");
        }
        transmitter: <> like ITransmitter {
            parameters:
                @display("p=100,200");
        }
        receiver: <> like IReceiver {
            parameters:
                @display("p=100,300");
        }
        energyConsumer: <default("")> like IEnergyConsumer if typename != "" {
            parameters:
                @display("p=100,400");
        }
}

