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

#ifndef __INETMANET_4_X_BLACKBOXINTERFACE_H_
#define __INETMANET_4_X_BLACKBOXINTERFACE_H_

#include <omnetpp.h>
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/queueing/contract/IActivePacketSink.h"
#include "inet/queueing/contract/IPacketQueue.h"
#include <map>

using namespace omnetpp;

namespace inet {
namespace blackbox {
/**
 * TODO - Generated class
 */
class BlackBoxInterface : public MacProtocolBase, public queueing::IActivePacketSink
{
    static std::map<MacAddress, opp_component_ptr<NetworkInterface>> interfaces;
    static std::map<L3Address, opp_component_ptr<NetworkInterface>> interfacesIpAddr;
  public:
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;

    // IActivePacketSink:
    virtual queueing::IPassivePacketSource *getProvider(const cGate *gate) override;
    virtual void handleCanPullPacketChanged(const cGate *gate) override;
    virtual void handlePullPacketProcessed(Packet *packet, const cGate *gate, bool successful) override;

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void configureNetworkInterface() override;
};

}
} //namespace

#endif
