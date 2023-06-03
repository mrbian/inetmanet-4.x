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

#ifndef INET_FLORA_LORAPHY_LORASCALARRECEPTIONANALOGMODEL_H_
#define INET_FLORA_LORAPHY_LORASCALARRECEPTIONANALOGMODEL_H_

#include "inet/physicallayer/wireless/common/analogmodel/scalar/ScalarReceptionAnalogModel.h"

namespace inet {
namespace flora {
using namespace physicallayer;
class LoraScalarReceptionAnalogModel : public  ScalarReceptionAnalogModel {
    int LoRaSF;
    int LoRaCR;
public:
    LoraScalarReceptionAnalogModel(const simtime_t preambleDuration, simtime_t headerDuration, simtime_t dataDuration, Hz centerFrequency, Hz bandwidth, W power, int sf, int cr);
    virtual ~LoraScalarReceptionAnalogModel();
    Hz getLoRaCF() const { return this->getCenterFrequency(); }
    int getLoRaSF() const { return LoRaSF; }
    Hz getLoRaBW() const { return this->getBandwidth(); }
    double getLoRaCR() const { return LoRaCR; }

};

} /* namespace flora */
} /* namespace inet */

#endif /* INET_FLORA_LORAPHY_LORASCALARRECEPTIONANALOGMODEL_H_ */
