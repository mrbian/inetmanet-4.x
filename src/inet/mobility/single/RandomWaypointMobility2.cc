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

#include "inet/mobility/single/RandomWaypointMobility2.h"
#include "inet/common/geometry/common/RotationMatrix.h"

namespace inet {

Define_Module(RandomWaypointMobility2);

void RandomWaypointMobility2::Visitor::visit(const cObject *object) const
{
    if (!isObstacleFound_)
        isObstacleFound_ = obstacles->isObstacle(check_and_cast<const physicalenvironment::IPhysicalObject *>(object), transmissionPosition, receptionPosition);
}

RandomWaypointMobility2::Visitor::Visitor(RandomWaypointMobility2 *mod, const Coord& endPosition, const Coord& initialPosition):
        obstacles(mod),
        transmissionPosition(endPosition),
        receptionPosition(initialPosition)
{

}

RandomWaypointMobility2::RandomWaypointMobility2():RandomWaypointMobility()
{
}

void RandomWaypointMobility2::initialize(int stage)
{
    RandomWaypointMobility::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        if (strcmp(par("Active").stringValue(), "OFF") == 0)  {
            stationary = true;
        }
        physicalEnvironment = findModuleFromPar<physicalenvironment::IPhysicalEnvironment>(par("physicalEnvironmentModule"), this->getSimulation()->getSystemModule());
    }
}


bool RandomWaypointMobility2::isObstacle(const physicalenvironment::IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const
{
    const ShapeBase *shape = object->getShape();
    const Coord& position = object->getPosition();
    const Quaternion& orientation = object->getOrientation();
    RotationMatrix rotation(orientation.toEulerAngles());
    const LineSegment lineSegment(rotation.rotateVectorInverse(transmissionPosition - position), rotation.rotateVectorInverse(receptionPosition - position));
    Coord intersection1, intersection2, normal1, normal2;
    bool hasIntersections = shape->computeIntersection(lineSegment, intersection1, intersection2, normal1, normal2);
    bool isObstacle = hasIntersections && intersection1 != intersection2;
    return isObstacle;
}

void RandomWaypointMobility2::setTargetPosition()
{
    if (physicalEnvironment == nullptr) {
        RandomWaypointMobility::setTargetPosition();
        return;
    }

    if (nextMoveIsWait) {
        simtime_t waitTime = waitTimeParameter->doubleValue();
        nextChange = simTime() + waitTime;
        nextMoveIsWait = false;
    }
    else {
        bool obs;
        do {
            targetPosition = getRandomPosition();
            Visitor visitor(this, targetPosition, this->getCurrentPosition());
            // check obstacles, if it cross an obstacle, select other
            physicalEnvironment->visitObjects(&visitor, LineSegment(targetPosition, this->getCurrentPosition()));
            obs = visitor.isObstacleFound();
        } while(obs);


        double speed = speedParameter->doubleValue();
        double distance = lastPosition.distance(targetPosition);
        simtime_t travelTime = distance / speed;
        nextChange = simTime() + travelTime;
        nextMoveIsWait = hasWaitTime;
    }
}


} // namespace inet

