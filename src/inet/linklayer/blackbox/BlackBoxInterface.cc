//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "BlackBoxInterface.h"

namespace inet {
namespace blackbox {

std::map<MacAddress, opp_component_ptr<NetworkInterface>> BlackBoxInterface::interfaces;

std::map<L3Address, opp_component_ptr<NetworkInterface>> BlackBoxInterface::interfacesIpAddr;

Define_Module(BlackBoxInterface);

void BlackBoxInterface::initialize(int stage)
{
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {

    }
    else if (stage == INITSTAGE_NETWORK_CONFIGURATION) {
        interfaces.insert(std::make_pair(networkInterface->getMacAddress(), networkInterface));
        interfacesIpAddr.insert(std::make_pair(networkInterface->getNetworkAddress(), networkInterface));
    }
}

void BlackBoxInterface::configureNetworkInterface()
{
    // MTU: typical values are 576 (Internet de facto), 1500 (Ethernet-friendly),
    // 4000 (on some point-to-point links), 4470 (Cisco routers default, FDDI compatible)
    networkInterface->setMtu(10000);
    // capabilities
    networkInterface->setMulticast(true);
    networkInterface->setBroadcast(true);
}

void BlackBoxInterface::handleStartOperation(LifecycleOperation *operation)
{
    networkInterface->setState(NetworkInterface::State::UP);
}

void BlackBoxInterface::handleStopOperation(LifecycleOperation *operation)
{
    if (currentTxFrame != nullptr || !txQueue->isEmpty()) {
        networkInterface->setState(NetworkInterface::State::GOING_DOWN);
    }
    else {
        networkInterface->setCarrier(false);
        networkInterface->setState(NetworkInterface::State::DOWN);
    }
}

void BlackBoxInterface::handleCrashOperation(LifecycleOperation *operation)
{
    networkInterface->setCarrier(false);
    networkInterface->setState(NetworkInterface::State::DOWN);
}

void BlackBoxInterface::handleMessage(cMessage *msg)
{
    // TODO - Generated method body
}

queueing::IPassivePacketSource *BlackBoxInterface::getProvider(const cGate *gate)
{
    return (gate->getId() == upperLayerInGateId) ? txQueue.get() : nullptr;
}

void BlackBoxInterface::handleCanPullPacketChanged(const cGate *gate)
{
    Enter_Method("handleCanPullPacketChanged");
    if (gate->getId() == upperLayerInGateId) {

    }
}

void BlackBoxInterface::handlePullPacketProcessed(Packet *packet, const cGate *gate, bool successful)
{
    Enter_Method("handlePullPacketProcessed");
    throw cRuntimeError("Not supported callback");
}

}
} //namespace
