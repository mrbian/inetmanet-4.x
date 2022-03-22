
package inet.wirelesspan.networklayer.configurator;

import inet.networklayer.configurator.ipv6.Ipv6FlatNetworkConfigurator;

@namespace(inet::wirelesspan);

simple RplNetConfigurator extends Ipv6FlatNetworkConfigurator
{
    parameters:
        @class(RplNetConfigurator);
        @display("i=block/cogwheel");
        @labels(node);
}

