#ifndef __AhpTopsis_H
#define __AhpTopsis_H

#include <algorithm>
#include <map>
#include <vector>
#include <cmath>
#include <deque>
#include "inet/common/INETDefs.h"
#include "inet/networklayer/contract/IL3AddressType.h"

namespace inet {
namespace inetmanet {

#ifndef NaN
#define qNaN        std::numeric_limits<double>::quiet_NaN()
#define NaN         qNaN
#endif

#define OLDLIST

//static const int wlength = 4*3+1; // 13
#ifdef TWOLIST
static const int wlength = 6; // 13
#elif defined(OLDLIST)
static const int wlength = 6; // 13
#else
static const int wlength = 7; // 13
#endif

enum AhpWeightComputeType
{
	Satrii,
	Sotooni,
	Hesabi,
	Hendesi 
};

class AhpWeightCompute2
{
private:
	double w[wlength][wlength];

	void runHendesi(double weight[], int length) {
		for (int i = 0; i < length; i++) 
			weight[i] = 1;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				weight[i]*=w[i][j];
			}
		}
		for (int i = 0; i < length; i++) {
			weight[i]=pow(weight[i],(double)1/3);
		}
		double normalizer=0;
		for (int i = 0; i < length; i++) {
			normalizer+=weight[i];
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)weight[i]/normalizer;
		}
	}
	void runSotooni(double weight[], int length) {
		for (int i = 0; i < length; i++) 
			weight[i] = 0;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				weight[i]+=w[j][i];
			}
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)1/weight[i];
		}
		double normalizer=0;
		for (int i = 0; i < length; i++) {
			normalizer+=weight[i];
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)weight[i]/normalizer;
		}
	}
	void runSatrii(double weight[], int length) {
		for (int i = 0; i < length; i++) 
			weight[i] = 0;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				weight[i]+=w[i][j];
			}
		}
		double normalizer=0;
		for (int i = 0; i < length; i++) {
			normalizer+=weight[i];
		}
		for (int i = 0; i < length; i++) {
			weight[i]=(double)weight[i]/normalizer;
		}
	}
	void runHesabi(double weight[], int length) {
		double c[wlength];
		for (int i = 0; i < length; i++) {
			double k = 0;
			for (int j = 0; j < length; j++) {
				k += w[j][i];
			}
			c[i] = k;
		}
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < length; j++) {
				w[i][j] = (double) w[i][j] / c[j];
			}
		}
		for (int i = 0; i < length; i++) {
			double k = 0;
			for (int j = 0; j < length; j++) {
				k += w[i][j];
			}
			weight[i] = (double) k / wlength;
		}
	}
public:	
	AhpWeightCompute2(double q[wlength][wlength]) {
		for (int i = 0; i < wlength; i++) {
			for (int j = 0; j < wlength; j++) {
				w[i][j]=q[i][j];
			}
		}
	}
	void run(double weight[], int length, 
		AhpWeightComputeType type = AhpWeightComputeType::Hendesi) {
		switch(type){
			case AhpWeightComputeType::Sotooni:
				return runSotooni(weight, length);
			case AhpWeightComputeType::Satrii:
				return runSatrii(weight, length);
			case AhpWeightComputeType::Hesabi:
				return runHesabi(weight, length);
			default:
				return runHendesi(weight, length);
		}
	}
};

class TopsisLoadNgNeigbors2 {
public:
    struct Cost {
        L3Address address;
        L3Address nextAddress;
        L3Address commonAddress;
        char addrSrt[16];
        bool stable = false;
        double etx = NaN;
        double neighbors = NaN;
        double snir = NaN;
        double recPower = NaN;
        double energy = NaN;
        double numHops = NaN;
        double delay = NaN;
        bool operator ==  (const Cost &a) {
            if (a.address == this->address && a.nextAddress == this->nextAddress)
                return true;
            return false;
        }
    };

    struct CostAndRange {
        Cost cost;
        double range;
        int index;
    };

private:
    /*
    double q[wlength][wlength]={
        //Energy Pos.X Pos.Y Pos.Z Speed.X Speed.Y Speed.Z ...
        {1,6.0/7,3.0/2,4,5,6,7,8,9,10,11,12,13},
        {1,1,3/2,4,5,6,7,8,9,10,11,12,13},
        {1,6/7,1,4,5,6,7,8,9,10,11,12,13},
        {1,6/7,3/2,1,5,6,7,8,9,10,11,12,13},
        {1,6/7,3/2,4,1,6,7,8,9,10,11,12,13},
        {1,6/7,3/2,4,5,1,7,8,9,10,11,12,13},
        {1,6/7,3/2,4,5,6,1,8,9,10,11,12,13},
        {1,6/7,3/2,4,5,6,7,1,9,10,11,12,13},
        {1,6/7,3/2,4,5,6,7,8,1,10,11,12,13},
        {1,6/7,3/2,4,5,6,7,8,9,1,11,12,13},
        {1,6/7,3/2,4,5,6,7,8,9,10,1,12,13},
        {1,6/7,3/2,4,5,6,7,8,9,10,11,1,13},
        {1,6/7,3/2,4,5,6,7,8,9,10,11,12,1},
    };
*/

/*    double q[wlength][wlength]={
            //Remaining Energy, receiver power, hop count, number of neighbors, SNIR, ETX
            {1, 2, 3, 4, 5, 6},
            {1.0/2.0, 2.0/2.0, 3.0/2.0, 4.0/2.0, 5.0/2.0, 6.0/2.0},
            {1.0/3.0, 2.0/3.0, 2.0/2.0, 4.0/3.0, 5.0/3.0, 6.0/3.0},
            {1.0/4.0, 2.0/4.0, 3.0/4.0, 4.0/4.0, 5.0/4.0, 6.0/4.0},
            {1.0/5.0, 2.0/5.0, 3.0/5.0, 4.0/5.0, 5.0/5.0, 6.0/5.0},
            {1.0/6.0, 2.0/6.0, 3.0/6.0, 4.0/6.0, 5.0/6.0, 6.0/6.0},
        }; */
    /*double q[wlength][wlength]={
            //Remaining Energy, SNIR, receiver power, number of neighbors, Hop count, ETX
            {1, 2, 5, 6, 8, 9},
            {1.0/2.0, 1.0/1.0, 5.0/2.0, 6.0/2.0, 8.0/2.0, 9.0/2.0},
            {1.0/5.0, 2.0/5.0, 1.0/1.0, 6.0/5.0, 8.0/5.0, 9.0/5.0},
            {1.0/6.0, 1.0/3.0, 5.0/6.0, 1.0/1.0, 8.0/6.0, 9.0/6.0},
            {1.0/8.0, 1.0/4.0, 5.0/8.0, 6.0/8.0, 1.0/1.0, 9.0/8.0},
            {1.0/9.0, 2.0/9.0, 5.0/9.0, 6.0/9.0, 8.0/9.0, 1.0/1.0},
        };*/
#ifdef TWOLIST
    double q[wlength][wlength]={
            //Etx, HC, number of neighbors, Rec power, snir, remained energy
            {1.0,     2.0,     4.0,     5.0,     6.0,     9.0},
            {1.0/2.0, 1.0/1.0, 3.0,     4.0,     5.0,     8.0},
            {1.0/4.0, 1.0/3.0, 1.0/1.0, 1.5,     2.0,     6.0},
            {1.0/5.0, 1.0/4.0, 1.0/1.5, 1.0/1.0, 1.5,     5.0},
            {1.0/6.0, 1.0/5.0, 1.0/2.0, 1.0/1.5, 1.0/1.0, 2.0},
            {1.0/9.0, 1.0/8.0, 1.0/6.0, 1.0/5.0, 1.0/2.0, 1.0/1.0},
    };

#elif defined(OLDLIST)
    double q[wlength][wlength]={
            //Etx, HC, number of neighbors, Rec power, snir, remained energy
            {1.0,     2.0,     4.0,     5.0,     6.0,     9.0},
            {1.0/2.0, 1.0/1.0, 3.0,     4.0,     5.0,     8.0},
            {1.0/4.0, 1.0/3.0, 1.0/1.0, 1.5,     2.0,     6.0},
            {1.0/5.0, 1.0/4.0, 1.0/1.5, 1.0/1.0, 1.5,     5.0},
            {1.0/6.0, 1.0/5.0, 1.0/2.0, 1.0/1.5, 1.0/1.0, 2.0},
            {1.0/9.0, 1.0/8.0, 1.0/6.0, 1.0/5.0, 1.0/2.0, 1.0/1.0},
    };
#else

    double q[wlength][wlength]={
            //Etx, HC, number of neighbors, Rec power, snir, remained energy
            {1, 8, 12, 16, 20, 24, 36},
            {1.0/8.0, 1, 8, 12, 16, 20, 32},
            {1.0/12.0, 1.0/8.0, 1, 10 , 6, 8, 24},
            {1.0/16.0, 1.0/12.0, 1.0/10, 1, 8, 6, 20},
            {1.0/20.0, 1.0/16.0, 1.0/6.0, 1.0/8.0, 1, 8, 12},
            {1.0/24.0, 1.0/20.0, 1.0/8.0, 1.0/6.0, 1.0/8.0, 1, 8},
            {1.0/36.0, 1.0/32.0, 1.0/24.0, 1.0/20.0, 1.0/12.0, 1.0/8.0, 1.0/1.0},
        };
#endif
    std::deque<Cost> listNodes;
    std::vector<std::pair<int, double>> sortedlist;
    std::vector<std::pair<int, double>> sortedlistStable;
    std::vector<int> posStable;
    std::vector<int> pos;
    bool change = true;
public:
    virtual void setExperiment(int experiment = 1){}

    virtual void setChange(bool p) {change = p;}
    virtual unsigned int numElements() const {return listNodes.size();}
    virtual void clear() {listNodes.clear(); change = true;}
    virtual Cost addCost(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower,const double &power,const double &hops, const double &delay,const bool &stable , L3Address common = L3Address());
    virtual void addCost(const Cost &);
    virtual Cost addCostNoCheck(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower, const double &power, const double &hops, const double &delay, const bool &stable, L3Address common = L3Address());

    virtual void runTopsis(std::vector<std::pair<double, Cost>> &); // return the list ordered based in range.
    virtual void runTopsisRoutes(std::map<L3Address, Cost> &results, bool = false); // return only the best entry per destination
    virtual void runTopsisRoutes(std::map<L3Address, Cost> &results, std::vector<int> &, bool = false);
    virtual void runTopsisRoutes(std::deque<CostAndRange> &results);
};

}
}

#endif // ifndef __AhpTopsis_H
