//
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


#include "inet/physicallayer/wireless/common/analogmodel/unitdisk/UnitDiskMediumAnalogModelLoss.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/wireless/common/radio/packetlevel/Radio.h"

#include "inet/physicallayer/wireless/common/analogmodel/unitdisk/UnitDiskReceptionAnalogModel.h"
#include "inet/physicallayer/wireless/common/analogmodel/unitdisk/UnitDiskTransmissionAnalogModel.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IArrival.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadioMedium.h"
#include "inet/physicallayer/wireless/common/radio/packetlevel/Reception.h"

namespace inet {
namespace physicallayer {

std::map<int, UnitDiskMediumAnalogModelLoss::Links> UnitDiskMediumAnalogModelLoss::uniLinks;
std::map<int, UnitDiskMediumAnalogModelLoss::Links> UnitDiskMediumAnalogModelLoss::lossLinks;

Define_Module(UnitDiskMediumAnalogModelLoss);

void UnitDiskMediumAnalogModelLoss::initialize(int stage)
{
    UnitDiskMediumAnalogModelLoss::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {

    }
    else if (stage == INITSTAGE_PHYSICAL_LAYER_NEIGHBOR_CACHE) {

             double numUniLink = par("perUniLinks").doubleValue();
             double numLossLinks = par("perLosLinks").doubleValue();
             if (numUniLink == 0 && numLossLinks == 0)
                 return;
             if (numUniLink > 100|| numLossLinks > 100)
                 throw cRuntimeError("perUniLinks %f or perLosLinks %f too big", numUniLink, numLossLinks);
             if (par("forceUni"))
                 if (numUniLink > 50)
                     throw cRuntimeError("perUniLinks %f with forceUni too big", numUniLink);
             if (!uniLinks.empty() || !lossLinks.empty())
                 return;
             // Extract the connection list
             cTopology topo("topo");
             topo.extractByProperty("networkNode");

             cTopology::Node *destNode = topo.getNode(0);
              cModule *host = destNode->getModule();

              auto transmitter = check_and_cast<ITransmitter*>(host->getSubmodule("transmitter"));

             communicationRange = transmitter->getMaxCommunicationRange();

             std::deque<std::pair<int, int>> links;
             std::deque<std::pair<int, int>> erased;
             for (int i = 0; i < topo.getNumNodes(); i++) {
                 cTopology::Node *destNode = topo.getNode(i);
                 cModule *host = destNode->getModule();
                 auto mod = check_and_cast<IMobility*>(
                         host->getSubmodule("mobility"));
                 auto cord1 = mod->getCurrentPosition();
                 for (int j = i + 1; j < topo.getNumNodes(); j++) {
                     cTopology::Node *destNodeAux = topo.getNode(j);
                     cModule *host2 = destNodeAux->getModule();
                     auto mod2 = check_and_cast<IMobility*>(
                             host2->getSubmodule("mobility"));
                     auto cord2 = mod2->getCurrentPosition();
                     if (cord1.distance(cord2) < communicationRange.get()) {
                         links.push_back(
                                 std::make_pair(host->getId(), host2->getId()));
                         links.push_back(
                                 std::make_pair(host2->getId(), host->getId()));
                     }
                 }
             }
             numUniLink /= 100;
             numUniLink *= links.size(); //
             numUniLink = std::floor(numUniLink);

             while (numUniLink > 0) {
                 int index = intuniform(0, links.size() - 1);
                 auto link = links[index];
                 if (par("forceUni")) {
                     auto itAux = std::find(erased.begin(), erased.end(),
                             std::make_pair(link.second, link.first));
                     if (itAux != erased.end())
                         continue;
                     erased.push_back(link);
                 }

                 links.erase(links.begin() + index);
                 auto it = uniLinks.find(link.first);
                 if (it == uniLinks.end()) {
                     UnitDiskMediumAnalogModelLoss::Links l;
                     l.push_back(link.second);
                     uniLinks[link.first] = l;
                 }
                 else
                     it->second.push_back(link.second);
                 numUniLink--;
             }
             numLossLinks /= 100;
             numLossLinks *= links.size(); //
             numLossLinks = std::floor(numLossLinks);

             while (numLossLinks > 0) {
                 int index = intuniform(0, links.size() - 1);
                 auto link = links[index];
                 links.erase(links.begin() + index);
                 auto it = lossLinks.find(link.first);
                 if (it == lossLinks.end()) {
                     UnitDiskMediumAnalogModelLoss::Links l;
                     l.push_back(link.second);
                     lossLinks[link.first] = l;
                 }
                 else
                     it->second.push_back(link.second);
                 numLossLinks--;
             }
         }
}

bool UnitDiskMediumAnalogModelLoss::checkLinkLoss(const IRadio *radio, const ITransmission *transmission) const
{

    int txId = transmission->getTransmitterRadio()->getId();
    int rxId = radio->getId();


     auto uniLinkIt = uniLinks.find(txId);
     auto lossLinkIt = lossLinks.find(txId);

     if (uniLinkIt == uniLinks.end() && lossLinkIt == lossLinks.end())
         return false;
     if (uniLinkIt != uniLinks.end()) {
         auto it = std::find(uniLinkIt->second.begin(), uniLinkIt->second.end(), rxId);
         if (it != uniLinkIt->second.end()) {
             return true;
         }
     }

     if (lossLinkIt != lossLinks.end()) {
         auto it = std::find(lossLinkIt->second.begin(), lossLinkIt->second.end(), rxId);
         if (it != lossLinkIt->second.end()) {
             double pr = dblrand();
             if (par("errorProb").doubleValue() > pr)
                 return true;
         }
     }
     return false;
}

std::ostream& UnitDiskMediumAnalogModelLoss::printToStream(std::ostream& stream, int level, int evFlags) const
{
    return stream << "UnitDiskMediumAnalogModelLoss";
}

const IReception *UnitDiskMediumAnalogModelLoss::computeReception(const IRadio *receiverRadio, const ITransmission *transmission, const IArrival *arrival) const
{
    const IRadioMedium *radioMedium = receiverRadio->getMedium();
    const UnitDiskTransmissionAnalogModel *analogModel = check_and_cast<const UnitDiskTransmissionAnalogModel *>(transmission->getAnalogModel());
    const simtime_t receptionStartTime = arrival->getStartTime();
    const simtime_t receptionEndTime = arrival->getEndTime();
    const Coord& receptionStartPosition = arrival->getStartPosition();
    const Coord& receptionEndPosition = arrival->getEndPosition();
    const Quaternion& receptionStartOrientation = arrival->getStartOrientation();
    const Quaternion& receptionEndOrientation = arrival->getEndOrientation();
    m distance = m(transmission->getStartPosition().distance(receptionStartPosition));
    double obstacleLoss = radioMedium->getObstacleLoss() ? radioMedium->getObstacleLoss()->computeObstacleLoss(Hz(NaN), transmission->getStartPosition(), receptionStartPosition) : 1;
    ASSERT(obstacleLoss == 0 || obstacleLoss == 1);
    UnitDiskReceptionAnalogModel::Power power;

    if (checkLinkLoss(receiverRadio, transmission))
        power = UnitDiskReceptionAnalogModel::POWER_UNDETECTABLE;
    else if (obstacleLoss == 0)
        power = UnitDiskReceptionAnalogModel::POWER_UNDETECTABLE;
    else if (distance <= analogModel->getCommunicationRange())
        power = UnitDiskReceptionAnalogModel::POWER_RECEIVABLE;
    else if (distance <= analogModel->getInterferenceRange())
        power = UnitDiskReceptionAnalogModel::POWER_INTERFERING;
    else if (distance <= analogModel->getDetectionRange())
        power = UnitDiskReceptionAnalogModel::POWER_DETECTABLE;
    else
        power = UnitDiskReceptionAnalogModel::POWER_UNDETECTABLE;
    auto receptionAnalogModel = new UnitDiskReceptionAnalogModel(analogModel->getPreambleDuration(), analogModel->getHeaderDuration(), analogModel->getDataDuration(), power);
    return new Reception(receiverRadio, transmission, receptionStartTime, receptionEndTime, receptionStartPosition, receptionEndPosition, receptionStartOrientation, receptionEndOrientation, receptionAnalogModel);
}

} // namespace physicallayer
} // namespace inet

