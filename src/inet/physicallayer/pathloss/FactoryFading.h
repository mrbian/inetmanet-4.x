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

#ifndef INET_PHYSICALLAYER_PATHLOSS_FACTORYFADING_H_
#define INET_PHYSICALLAYER_PATHLOSS_FACTORYFADING_H_

#include "inet/physicallayer/wireless/common/pathloss/FreeSpacePathLoss.h"
#include "inet/common/geometry/common/Coord.h"

namespace inet {
namespace physicallayer {

typedef struct {
    double xmin;
    double ymin;
    double xmax;
    double ymax;
} Block;

class INET_API  FactoryFading : public FreeSpacePathLoss {
public:
    virtual void initialize(int stage) override;
    std::vector<Block> blockages;

    std::string blockageFile;

    double m_los_sigma = 4;
    double m_los_alpha = 31.84;
    double m_los_beta = 21.5;
    double m_los_gamma = 19.0;

    double m_nlos_sigma = 5.7;
    double m_nlos_alpha = 33;
    double m_nlos_beta = 25.5;
    double m_nlos_gamma = 20.0;

    std::map<int, double> shadowing_fading_cache;
    std::map<int, bool> los_condition_cache;
    std::map<int, double> distance_cache;

    bool loadObstacle;
    bool loadShadowFading;
    bool loadSmallScaleFading;
    std::vector<std::vector<double>> shadow_fading_matrix;

    static std::map<std::pair<int, int>, LOSCond*>  transmissionLOSCondCache;

public:
    FactoryFading();
    virtual ~FactoryFading();

    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags) const override;
    virtual double computePathLoss(const ITransmission *transmission, const IArrival *arrival) const override;
    virtual double computePathLoss(const IRadio *receiverRadio, const ITransmission *transmission, const IArrival *arrival) const override;
    virtual m computeRange(mps propagationSpeed, Hz frequency, double loss) const override;

    double computePathLoss(double frequency, Coord ptx, Coord prx, bool is_nlos) const;
    double computeShadowFading(Coord ptx, Coord prx) const;
    double computeSmallScaleFading(bool is_nlos) const;
    bool checkNlos(Coord ptx, Coord prx) const;
    bool CheckLineRectIntersect(double x1, double y1, double x2, double y2, double xmin, double ymin, double xmax, double ymax) const;
    int ComputeCode(double x, double y, double xmin, double ymin, double xmax, double ymax) const;

    virtual LOSCond* getLOSCondByTxId(int tranmission_id, int rx_id) const override;

};

} /* namespace physicallayer */
} /* namespace inet */

#endif /* INET_PHYSICALLAYER_PATHLOSS_FACTORYFADING_H_ */
