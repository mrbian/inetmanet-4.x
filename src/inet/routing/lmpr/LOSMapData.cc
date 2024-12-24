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

#include "LMPR.h"
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include "inet/physicallayer/wireless/common/contract/packetlevel/SignalTag_m.h"
#include "inet/physicallayer/pathloss/LOSCondTag_m.h"

namespace inet {

void LMPR::addMapData(Packet *packet)
{
    const auto losCond = packet->findTag<LOSCondTag>();
    const auto sinrInd = packet->findTag<SnirInd>();
    if(losCond && sinrInd)
    {
        MapDataEle d;
        d.txX = losCond->getTxX();
        d.txY = losCond->getTxY();
        d.rxX = losCond->getRxX();
        d.rxY = losCond->getRxY();
        d.sinr = sinrInd->getMinimumSnir();
        d.losCond = losCond->getLosCondFlag();
        mapDataSet.push_back(d);
    }
}

void LMPR::storeMapDataToCSV()
{
    std::string baseName = getEnvir()->getConfig()->substituteVariables("${resultdir}/../LOSMapDataFiles/${configname}-${iterationvarsf}#${repetition}");
    if (!std::filesystem::exists(baseName)) {
        if (std::filesystem::create_directory(baseName)) {
            std::cout << "Folder " << baseName << " created successfully" << std::endl;
        } else {
            std::cerr << "Cannot create folder" << baseName << "." << std::endl;
            return;
        }
    }

    cModule *node = findContainingNode(this);
    std::string nodeName = node->getFullName();
    std::string fileName = nodeName + ".csv";
    std::string fullPath = baseName + "/" + fileName;
    std::ofstream file(fullPath);
    if (file.is_open()) {
      std::cout << "File " << fullPath << " is opened successfully" << std::endl;
      for(auto ele = mapDataSet.begin(); ele != mapDataSet.end(); ele ++)
      {
          file << ele->txX << ","
                  << ele->txY << ","
                  << ele->rxX << ","
                  << ele->rxY << ","
                  << ele->sinr << ","
                  << ele->losCond << std::endl;
      }
      file.close();
    } else {
      std::cerr << "Cannot create or open " << fullPath << "." << std::endl;
    }
    return;
}


} /* namespace inet */
