//
// Copyright (C) 2005 Georg Lutz, Institut fuer Telematik, University of Karlsruhe
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

package inet.mobility.single;

import inet.mobility.single.RandomWaypointMobility;

//
// Implements the Random Waypoint mobility model.
//
// The node moves in line segments. For each line segment, a random destination
// position (distributed uniformly over the constraint area or volume) and a 
// random speed is chosen. You can define a speed as a variate from which a new
// value will be drawn for each line segment; it is customary to specify it as
// uniform(minSpeed, maxSpeed). When the node reaches the target position,
// it waits for the time waitTime which can also be defined as a variate.
// After this time the the algorithm calculates a new random position, etc.
//
// This model was originally written by Georg Lutz (GeorgLutz AT gmx DOT de) for
// his diploma thesis "Effizientes Modell fuer Funkverbindungen in 4G-Netzen fuer
// OMNeT++" (Efficient model for radio links in 4G networks for OMNeT++)
// at Institut fuer Telematik, Universitaet Karlsruhe (Institute for
// Telematics, University Karlsruhe, Germany), 2005-06-21.
//
// @author Georg Lutz
//
simple RandomWaypointMobility2 extends RandomWaypointMobility
{
    parameters:
        string physicalEnvironmentModule = default("physicalEnvironment");
        string Active @enum("ON", "OFF") = default("ON");
        @class(RandomWaypointMobility2);
}

