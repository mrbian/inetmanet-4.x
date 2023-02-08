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

package inet.flora.loraphy;

import inet.physicallayer.wireless.common.pathloss.FreeSpacePathLoss;
// default values from "Do LoRa Low-Power Wide-Area Networks Scale?"
// Values from paper  "On the Coverage of LPWANs: Range Evaluation and Channel Attenuation Model for LoRa Technology"
// Empirical
// d0 = 1000m
//                 boat      car
//  PL_d0_db    126.43      128.95
//   gamma       1.76       2.32
//   sigma         8        7.8

module LoRaLogNormalShadowing extends FreeSpacePathLoss
{
    parameters:
        //parameters taken from paper LoRaSim
        double d0  @unit(m) = default(40m); // 1000, sigma boat 1.72, car 2.32, PL0 boat 126.43, 
        double PL_d0_db = default(127.41); // Path loss intercept
        double gamma = default(2.08);      // Path loss exponent
        double sigma = default(3.57);    //     
        @class(LoRaLogNormalShadowing);
}
