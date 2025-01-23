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

#include "PARRoT_map.h"

namespace inet {

Define_Module(PARRoT_map);

PARRoT_map::PARRoT_map() {
    // TODO Auto-generated constructor stub

}

PARRoT_map::~PARRoT_map() {
    // TODO Auto-generated destructor stub
}

void PARRoT_map::initialize(int stage) {

    NetworkProtocolBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        m_squNr = 0;
        ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        // Routing
        mhChirpInterval = par("mhChirpInterval");
        discount = par("discount");
        maxHops = par("maxHops");
        neighborReliabilityTimeout = par("neighborReliabilityTimeout");
        // Create reminder messages
        multiHopChirpReminder = new cMessage("multiHopChirpReminder");
        LETRangeMode = par("LETRangeMode");
        losRange = par("losRange");
        nlosRange = par("nlosRange");
        considerSmallScale = par("considerSmallScale");
        if(considerSmallScale)
        {
            std::string line;

            const char * fname_rician = par("RicianFadingFile");
            std::string filename_rician(fname_rician);
            std::ifstream file_rician(filename_rician);
            if (!file_rician.is_open()) {
                throw cRuntimeError("无法打开文件");
            }
            // 逐行读取文件
            while (std::getline(file_rician, line)) {
               std::stringstream ss(line);
               std::string value;
               std::vector<double> row;
               // 按照逗号分割每行
               while (std::getline(ss, value, ',')) {
                   row.push_back(std::stod(value));  // 将字符串转换为double并存储
               }
               // 将读取到的行存储到矩阵中
               rician_outage_vector.push_back(row[0]);
            }
            // 关闭文件
            file_rician.close();

            const char * fname_rayleigh = par("RayleighFadingFile");
            std::string filename_rayleigh(fname_rayleigh);
            std::ifstream file_rayleigh(filename_rayleigh);
            if (!file_rayleigh.is_open()) {
               throw cRuntimeError("无法打开文件");
            }
            // 逐行读取文件
            while (std::getline(file_rayleigh, line)) {
              std::stringstream ss(line);
              std::string value;
              std::vector<double> row;
              // 按照逗号分割每行
              while (std::getline(ss, value, ',')) {
                  row.push_back(std::stod(value));  // 将字符串转换为double并存储
              }
              // 将读取到的行存储到矩阵中
              rayleigh_outage_vector.push_back(row[0]);
            }
            // 关闭文件
            file_rayleigh.close();
        }

    } else if(stage == INITSTAGE_NETWORK_LAYER) {
        pathLoss = check_and_cast<physicallayer::FactoryFading*>(getModuleByPath("Net80211_aodv.radioMedium.pathLoss"));
        mobility = check_and_cast<IMobility*>(getContainingNode(this)->getSubmodule("mobility"));
        arp = check_and_cast<GlobalArp*>(this->getParentModule()->getSubmodule("arp"));

        cModule* topModule = getModuleByPath("Net80211_aodv");
        int numServers = topModule->par("numServers");
        int numClients = topModule->par("numClients");
        int numRouters = topModule->par("numRouters");
        char mob_path_str[100];
        char ip_path_str[100];
        for(int i = 0; i < numServers; i ++)
        {
            sprintf(mob_path_str, "Net80211_aodv.server[%d].mobility", i);
            sprintf(ip_path_str, "Net80211_aodv.server[%d].wlan[0]", i);
            ExtendedBonnMotionMobility* mob = check_and_cast<ExtendedBonnMotionMobility*>(getModuleByPath(mob_path_str));
            NetworkInterface *ie = check_and_cast<NetworkInterface*>(getModuleByPath(ip_path_str));
            Ipv4Address addr = ie->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
            _globalMob.insert({addr, mob});
        }
        for (int i = 0; i < numClients; i ++)
        {
            sprintf(mob_path_str, "Net80211_aodv.client[%d].mobility", i);
            sprintf(ip_path_str, "Net80211_aodv.client[%d].wlan[0]", i);
            ExtendedBonnMotionMobility* mob = check_and_cast<ExtendedBonnMotionMobility*>(getModuleByPath(mob_path_str));
            NetworkInterface *ie = check_and_cast<NetworkInterface*>(getModuleByPath(ip_path_str));
            Ipv4Address addr = ie->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
            _globalMob.insert({addr, mob});
        }
        for (int i = 0; i < numRouters; i ++)
        {
            sprintf(mob_path_str, "Net80211_aodv.router[%d].mobility", i);
            sprintf(ip_path_str, "Net80211_aodv.router[%d].wlan[0]", i);
            ExtendedBonnMotionMobility* mob = check_and_cast<ExtendedBonnMotionMobility*>(getModuleByPath(mob_path_str));
            NetworkInterface *ie = check_and_cast<NetworkInterface*>(getModuleByPath(ip_path_str));
            Ipv4Address addr = ie->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
            _globalMob.insert({addr, mob});
        }
    }
    else
    {
        if(stage != 1)
            EV << stage << std::endl;
    }
}

void PARRoT_map::start() {
    /* Search the 80211 interface */
    int num_80211 = 0;
    broadcastDelay = &par("broadcastDelay");
    NetworkInterface *ie;
    NetworkInterface *i_face;
    const char *name;
    for (int i = 0; i < ift->getNumInterfaces(); i++) {
        ie = ift->getInterface(i);
        name = ie->getInterfaceName();
        if (strstr(name, "wlan") != nullptr) {
            i_face = ie;
            num_80211++;
            interfaceId = i;
        }
    }

    if (num_80211 == 1) {
        interface80211ptr = i_face;
    }
    else {
        throw cRuntimeError("Router has found %i 80211 interfaces", num_80211);
    }

    m_selfIpv4Address = interface80211ptr->getProtocolData<Ipv4InterfaceData>()->getIPAddress();

    // Schedule update reminder
    scheduleAt(simTime() + uniform(0.0, par("maxJitter")), multiHopChirpReminder);

}

void PARRoT_map::stop() {

}
void PARRoT_map::finish() {
}

void PARRoT_map::handleStartOperation(LifecycleOperation *operation)
{
    start();
//    helloTimer->resched(hello_ival());
//    tcTimer->resched(hello_ival());
//    scheduleEvent();
}

void PARRoT_map::handleUpperPacket(Packet *packet)
{
    // application data packet
    handleDataFromUpperLayer(packet);
}

void PARRoT_map::handleLowerPacket(Packet *packet)
{
    if(strstr(packet->getName(), "multiHopChirpMap") != nullptr)
    {
        recv_parrot_map(packet);
    }
    // data
    else
    {
        handleDataFromLowerLayer(packet);
    }
}

//void
//PARRoT_map::handleSelfMessage(cMessage *msg)
//{
//    checkTimer(msg);
//    scheduleEvent();  // 调度队列（按时间排序）的下一个timer
//}

void PARRoT_map::handleSelfMessage(cMessage *msg) {
    if (msg == multiHopChirpReminder) {
        sendMultiHopChirp();
        scheduleAt(simTime() + mhChirpInterval + broadcastDelay->doubleValue(), multiHopChirpReminder);
    }
//    else if (msg == destinationReminder) {
//        // Erase handled Event
//        if(!destinationsToUpdate.empty()){
//            Ipv4Address target = destinationsToUpdate.begin()->second;
//            destinationsToUpdate.erase(destinationsToUpdate.begin());
//            for (std::map<SimTime, Ipv4Address>::iterator it = destinationsToUpdate.begin(); it != destinationsToUpdate.end(); it++){
//                if(it->second == target){
//                    destinationsToUpdate.erase(it);
//                }
//            }
//            refreshRoutingTable(target);
//        }
//        // Schedule next event
//        if(!destinationsToUpdate.empty()){
//            if(destinationReminder->isScheduled()){
//                cancelEvent(destinationReminder);
//            }
//            scheduleAt(destinationsToUpdate.begin()->first, destinationReminder);
//        }
//    }
    else {
        delete msg;
    }
}

void PARRoT_map::setDownControlInfo(Packet *const pMsg, const MacAddress& pDestAddr)
{
    pMsg->addTagIfAbsent<MacAddressReq>()->setDestAddress(pDestAddr);
    pMsg->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&getProtocol());
    pMsg->addTagIfAbsent<DispatchProtocolInd>()->setProtocol(&getProtocol());
}



} /* namespace inet */
