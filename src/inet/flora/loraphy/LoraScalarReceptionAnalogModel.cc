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

#include "inet/flora/loraphy/LoraScalarReceptionAnalogModel.h"

namespace inet {
namespace flora {

LoraScalarReceptionAnalogModel::LoraScalarReceptionAnalogModel(const simtime_t preambleDuration, simtime_t headerDuration, simtime_t dataDuration, Hz centerFrequency, Hz bandwidth, W power, int sf, int cr) :
    ScalarReceptionAnalogModel(preambleDuration, headerDuration, dataDuration, centerFrequency, bandwidth, power)
{
    LoRaSF = sf;
    LoRaCR = cr;
}


LoraScalarReceptionAnalogModel::~LoraScalarReceptionAnalogModel() {
    // TODO Auto-generated destructor stub
}

} /* namespace flora */
} /* namespace inet */
