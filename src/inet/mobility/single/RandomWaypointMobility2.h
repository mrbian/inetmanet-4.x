//
// Copyright (C) 2005 Georg Lutz, Institut fuer Telematik, University of Karlsruhe
// Copyright (C) 2005 Andras Varga
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

#ifndef __INET_RANDOMWPMOBILITY2_H
#define __INET_RANDOMWPMOBILITY2_H

#include "inet/common/INETDefs.h"
#include "inet/common/IVisitor.h"
#include "inet/common/ModuleRefByPar.h"
#include "inet/mobility/single/RandomWaypointMobility.h"
#include "inet/environment/contract/IPhysicalEnvironment.h"

namespace inet {

/**
 * Random Waypoint mobility model. See NED file for more info.
 *
 * @author Georg Lutz (georglutz AT gmx DOT de), Institut fuer Telematik,
 *  Universitaet Karlsruhe, http://www.tm.uka.de, 2004-2005
 * @author Andras Varga (generalized, ported to LineSegmentsMobilityBase)
 */
class INET_API RandomWaypointMobility2 : public RandomWaypointMobility
{
  protected:
    physicalenvironment::IPhysicalEnvironment* physicalEnvironment = nullptr;
    /** @brief Initializes mobility model parameters.*/
    virtual void initialize(int stage) override;
    virtual void setTargetPosition() override;
    virtual bool isObstacle(const physicalenvironment::IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const;
    class Visitor : public IVisitor
    {
    protected:
        const RandomWaypointMobility2 *obstacles = nullptr;
        const Coord transmissionPosition;
        const Coord receptionPosition;
        mutable bool isObstacleFound_ = false;
    public:
        Visitor(RandomWaypointMobility2 *, const Coord& endPosition, const Coord& initialPosition);
        void visit(const cObject *object) const override;
        bool isObstacleFound() const { return isObstacleFound_; }
    };


  public:
    RandomWaypointMobility2();
};

} // namespace inet

#endif // ifndef __INET_RANDOMWPMOBILITY_H

