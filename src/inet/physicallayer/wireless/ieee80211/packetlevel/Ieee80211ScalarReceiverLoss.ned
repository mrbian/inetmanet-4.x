//
// Copyright (C) 2013 OpenSim Ltd
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

package inet.physicallayer.wireless.ieee80211.packetlevel;

//
// This receiver model receives an IEEE 802.11 transmission successfully if the
// minimum of the signal to noise and interference ratio over the duration of
// the reception is sufficiently high. It uses one of the IEEE 802.11 specific
// error models to compute the error rate based on this value and the used
// operation mode.
//
// See also ~Ieee80211ScalarTransmitter, ~Ieee80211ScalarRadio, and
// ~Ieee80211ScalarRadioMedium.
//
module Ieee80211ReceiverLoss extends Ieee80211Receiver
{
    parameters:
        double perUniLinks = default(0);
        double perLosLinks = default(0);
        bool  forceUni = default(true);
        double errorProb = default(0);
        @class(Ieee80211ScalarReceiverLoss);
}

