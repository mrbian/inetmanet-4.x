package inet.routing.PARRoT_map;

import inet.networklayer.base.NetworkProtocolBase;
import inet.networklayer.contract.INetworkProtocol;

simple PARRoT_map extends NetworkProtocolBase like INetworkProtocol
{
    parameters:
        @display("i=block/routing");
        @class(PARRoT_map);
        
        @signal[destDataHopCount](type=int);
        @statistic[destDataHopCount](title="app data arrive on destination hop count"; record=histogram; interpolationmode=none);
 		@signal[packetDropped](type=inet::Packet);
 		@statistic[dropDataPacket](title="data packets dropped"; source=packetDropped; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);    
        
        // Routing
        int maxHops = default(32);
        double neighborReliabilityTimeout @unit(s) = default(2s);
        bool useOHRepair = default(false);
        double mhChirpInterval @unit(s) = default(1s);
        
        // Mobility Prediction
        int historySize = default(5);
        string predictionMethod = default("slope");        
        bool advancedMobility = default(false);
        double discount = default(0.8);
                
        // Basic Configureation
        string netmask = default("255.255.0.0");  // netmask for the ad-hoc network
        double maxJitter = default(0.1); // This is the maximum of a random value to determine when the first hello message will be sent out 
        volatile double broadcastDelay @unit(s) = default(uniform(0s,0.01s));

		double losRange;
		double nlosRange;
		int LETRangeMode;
		
		bool considerSmallScale = default(true);
		string RicianFadingFile = default("");
		string RayleighFadingFile = default("");
}

