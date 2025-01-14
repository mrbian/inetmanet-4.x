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
#include "inet/physicallayer/pathloss/LOSCondTag_m.h"

namespace inet {
namespace physicallayer {

Define_Module(FactoryFading);

std::map<std::pair<int, int>, LOSCond*> FactoryFading::transmissionLOSCondCache;

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
        loadObstacle = par("loadObstacle");
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

        loadShadowFading = par("loadShadowFading");
        if(loadShadowFading)
        {
            const char * fname = par("shadowFadingFile");
            std::string filename(fname);
            std::ifstream file(filename);
            if (!file.is_open()) {
                throw cRuntimeError("无法打开文件");
            }
           std::string line;
           // 逐行读取文件
           while (std::getline(file, line)) {
               std::stringstream ss(line);
               std::string value;
               std::vector<double> row;
               // 按照逗号分割每行
               while (std::getline(ss, value, ',')) {
                   row.push_back(std::stod(value));  // 将字符串转换为double并存储
               }
               // 将读取到的行存储到矩阵中
               shadow_fading_matrix.push_back(row);
           }
           // 关闭文件
           file.close();
        }

        loadSmallScaleFading = par("loadSmallScaleFading");
    }
}

double
FactoryFading::computePathLoss(const ITransmission *transmission, const IArrival *arrival) const
{
    throw cRuntimeError("Should not use common FactoryFading::computePathLoss!");
}

double
FactoryFading::computePathLoss(const IRadio *receiverRadio, const ITransmission *transmission, const IArrival *arrival) const
{
    auto analogModel = check_and_cast<const INarrowbandSignalAnalogModel *>(transmission->getAnalogModel());
    Hz centerFrequency = Hz(analogModel->getCenterFrequency());
    double freq = centerFrequency.get();

    freq = 5.9e9;

    // check los cond
    Coord ptx = transmission->getStartPosition();
    Coord prx = arrival->getStartPosition();
    bool is_nlos = checkNlos(ptx, prx);

    // cache los cond
    int tx_id = transmission->getId();
    int rx_id = receiverRadio->getId();
    LOSCond* cond = new LOSCond();
    cond->txX = ptx.x;
    cond->txY = ptx.y;
    cond->rxX = prx.x;
    cond->rxY = prx.y;
    cond->losCondFlag = is_nlos ? 0 : 1;
    transmissionLOSCondCache.insert(std::make_pair(std::make_pair(tx_id, rx_id), cond));

    // path loss
    double PL_dB = computePathLoss(freq, ptx, prx, is_nlos);
    // shadow fading
    double SF_dB = computeShadowFading(ptx, prx);
    // small scale fading
    double SSF_dB = computeSmallScaleFading(is_nlos);
    // total loss in dB
    double lossDb = PL_dB + SF_dB + SSF_dB;
    return pow(10, lossDb/10);
}

m
FactoryFading::computeRange(mps propagationSpeed, Hz frequency, double loss) const
{
    // use LOS model to compute range for mediumCache
    double freq = frequency.get();
    m maxRange = m(pow(10, (-10*log10(loss) - m_los_alpha - m_los_gamma*log10(freq/1e9))/m_los_beta));
    return maxRange;
}

LOSCond* FactoryFading::getLOSCondByTxId(int tranmission_id, int rx_id) const
{
    std::pair<int, int> keyToFind = std::make_pair(tranmission_id, rx_id);
    auto it = transmissionLOSCondCache.find(keyToFind);
    if(it != transmissionLOSCondCache.end())
    {
        return it->second;
    }
    return nullptr;
}


double
FactoryFading::computePathLoss(double frequency, Coord ptx, Coord prx, bool is_nlos) const
{
    double distance = ptx.distance(prx);
    double lossDb;
    if(is_nlos)
    {
        lossDb = m_nlos_alpha + m_nlos_beta*log10(distance) + m_nlos_gamma*log10(frequency/1e9);
    }
    else
    {
        lossDb = m_los_alpha + m_los_beta*log10(distance) + m_los_gamma*log10(frequency/1e9);
    }

    return -lossDb;
}

double
FactoryFading::computeShadowFading(Coord ptx, Coord prx) const
{
    double lossDb = 0;
    if(loadShadowFading)
    {
        int tx_x_idx = floor(ptx.x/5);
        int tx_y_idx = floor(ptx.y/5);
        int rx_x_idx = floor(prx.x/5);
        int rx_y_idx = floor(prx.y/5);
        lossDb = shadow_fading_matrix[tx_x_idx*80+tx_y_idx][rx_x_idx*80+rx_y_idx];
    }
    return lossDb;
}

double FactoryFading::computeSmallScaleFading(bool is_nlos) const
{
    double lossDb = 0;
    if(loadSmallScaleFading)
    {
        double doppler_fading = -0.0436; // 20m/s, dB
        double multipath_fading = 0;
        if(is_nlos)
        {
            double sigma = 1;
            double real_part = sigma * normal(0,1);
            double imag_part =  sigma * normal(0,1);
            double amplitude_h = real_part*real_part + imag_part*imag_part;
            multipath_fading = 10*log10(amplitude_h);
        }
        else
        {
            double nu = 1;
            double sigma = 1;
            double los_component = nu;
            double real_part = los_component + sigma * normal(0,1);
            double imag_part = sigma * normal(0,1);
            double amplitude_h = real_part*real_part + imag_part*imag_part;
            multipath_fading = 10*log10(amplitude_h);
        }
        lossDb = doppler_fading + multipath_fading;
    }
    return lossDb;
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
