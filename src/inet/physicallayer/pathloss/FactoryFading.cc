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

#include "FactoryFading.h"

#include <fstream>
#include <sstream>

#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadioMedium.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/IRadioSignal.h"

#include "inet/physicallayer/wireless/common/analogmodel/scalar/ScalarTransmitterAnalogModel.h"

namespace inet {
namespace physicallayer {

Define_Module(FactoryFading);

FactoryFading::FactoryFading() {
    // TODO Auto-generated constructor stub

}
FactoryFading::~FactoryFading() {
    // TODO Auto-generated destructor stub
}

void
FactoryFading::initialize(int stage)
{
    FreeSpacePathLoss::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        bool loadObstacle = par("loadObstacle");
        if(loadObstacle)
        {
            const char *fname = par("blockageFile");
            std::string filename(fname);
            std::fstream r;
            char cstr[256];
            r.open(filename.c_str(), std::ios::in);
            if(r.is_open()){
                while(r.getline(cstr, sizeof(cstr))){
                    Block block;

                    std::string str(cstr);
                    size_t pos = 0;
                    std::string delimiter = ",";
                    pos = str.find(delimiter);
                    str.erase(0, pos + delimiter.length());
                    block.xmin = std::stod(str);

                    pos = str.find(delimiter);
                    str.erase(0, pos + delimiter.length());
                    block.ymin = std::stod(str);

                    pos = str.find(delimiter);
                    str.erase(0, pos + delimiter.length());
                    block.xmax = std::stod(str);

                    pos = str.find(delimiter);
                    str.erase(0, pos + delimiter.length());
                    block.ymax = std::stod(str);

                    blockages.push_back(block);
                }
            }else{
                throw;
            }

            r.close();
        }
    }
}

double
FactoryFading::computePathLoss(const ITransmission *transmission, const IArrival *arrival) const
{
    auto analogModel = check_and_cast<const INarrowbandSignalAnalogModel *>(transmission->getAnalogModel());
    Hz centerFrequency = Hz(analogModel->getCenterFrequency());
    double freq = centerFrequency.get();
    Coord ptx = transmission->getStartPosition();
    Coord prx = arrival->getStartPosition();
    int tx_id = transmission->getTransmitterRadioId();
    return computePathLoss(freq, ptx, prx, tx_id);
}

m
FactoryFading::computeRange(mps propagationSpeed, Hz frequency, double loss) const
{
    // use LOS model to compute range for mediumCache
    double freq = frequency.get();
    m maxRange = m(pow(10, (-10*log10(loss) - m_los_alpha - m_los_gamma*log10(freq/1e9))/m_los_beta));
    return maxRange;
}


double
FactoryFading::computePathLoss(double frequency, Coord ptx, Coord prx, int tx_id) const
{
    double distance = ptx.distance(prx);
    double lossDb;
    bool is_nlos = checkNlos(ptx, prx);
    frequency = 5.9*1e9; // todo: disable
//    double shadowFadingDb = getShadowingFading(tx_id, rx_id, distance, is_nlos);
    if(is_nlos)
    {
        lossDb = m_nlos_alpha + m_nlos_beta*log10(distance) + m_los_gamma*log10(frequency/1e9);
    }
    else
    {
        lossDb = m_los_alpha + m_los_beta*log10(distance) + m_los_gamma*log10(frequency/1e9);
    }
    return pow(10, -lossDb/10);
}

bool
FactoryFading::checkNlos(Coord ptx, Coord prx) const
{
//    return true; // todo: disable
    double x1 = ptx.x;
    double y1 = ptx.y;
    double x2 = prx.x;
    double y2 = prx.y;
    for (auto it = blockages.begin(); it != blockages.end(); ++it) {
        if(CheckLineRectIntersect(x1, y1, x2, y2, it->xmin, it->ymin, it->xmax, it->ymax))
            return true;
    }
    return false;
}

double
FactoryFading::getShadowingFading(int tx_id, int rx_id, double distance, bool is_nlos) const
{
//    UnorderedPair node_pair = UnorderedPair(tx_id, rx_id);
//    auto it_sf_cache = shadowing_fading_cache.find(node_pair);
//    if(it_sf_cache == shadowing_fading_cache.end())
//    {
//        shadowing_fading_cache.insert(make_pair(node_pair, normal(0, m_los_sigma)));
//    }
//    auto it_los_cond_cache = los_condition_cache.find(node_pair);
//    if(it_los_cond_cache == los_condition_cache.end())
//    {
//        los_condition_cache.insert(make_pair(node_pair, is_nlos));
//    }
//    auto it_dist_cache = distance_cache.find(node_pair);
//    if(it_dist_cache == distance_cache.end())
//    {
//        distance_cache.insert(make_pair(node_pair, distance));
//    }

}

bool
FactoryFading::CheckLineRectIntersect(double x1, double y1, double x2, double y2, double xmin, double ymin, double xmax, double ymax) const
{
    int LEFT = 1;
    int RIGHT = 2;
    int BOTTOM = 4;
    int TOP = 8;

    int code1 = ComputeCode(x1, y1, xmin, ymin, xmax, ymax);
    int code2 = ComputeCode(x2, y2, xmin, ymin, xmax, ymax);

    if((code1 | code2) == 0)
        return true;
    else
    {
        if((code1 & code2) != 0)
            return false;
        else
        {
            int codeout = code1;
            if(code1 == 0)
                codeout = code2;
            double k, xout, yout;
            if((x2-x1) == 0)
                k = std::numeric_limits<double>::infinity();
            else
                k = (y2-y1)/(x2-x1);
            if(codeout & LEFT)
            {
                xout = xmin;
                yout = y1+k*(xout-x1);
                if(yout >= ymin && yout <= ymax)
                    return true;
            }
            if(codeout & RIGHT)
            {
                xout = xmax;
                yout = y1+k*(xout-x1);
                if(yout >= ymin && yout <= ymax)
                    return true;
            }
            if(codeout & BOTTOM)
            {
                yout = ymin;
                xout = x1 + (yout-y1)/k;
                if(xout >= xmin && xout <= xmax)
                    return true;
            }
            if(codeout & TOP)
            {
                yout = ymax;
                xout = x1+(yout-y1)/k;
                if(xout >= xmin && xout <= xmax)
                    return true;
            }
        }
    }
    return false;
}

int
FactoryFading::ComputeCode(double x, double y, double xmin, double ymin, double xmax, double ymax) const
{
    int INSIDE = 0;
    int LEFT = 1;
    int RIGHT = 2;
    int BOTTOM = 4;
    int TOP = 8;

    int code = INSIDE;
    if(x < xmin)
        code = code | LEFT;
    else if(x > xmax)
        code = code | RIGHT;
    if(y < ymin)
        code = code | BOTTOM;
    else if(y > ymax)
        code = code | TOP;

    return code;
}

std::ostream& FactoryFading::printToStream(std::ostream& stream, int level, int evFlags) const
{
    stream << "FactoryFading";
    return stream;
}




} /* namespace physicallayer */
} /* namespace inet */
