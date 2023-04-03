#include <algorithm>
#include <map>
#include <vector>
#include <cmath>
#include <math.h>
#include "inet/routing/extras/LoadNg/AhpTopsis.h"

namespace inet {
namespace inetmanet {

int searchPosition(const TopsisLoadNgNeigbors::ListPos positions[],const int &size, const TopsisLoadNgNeigbors::ListPos &objective) {
    for (int i = 0; i < size; i ++) {
        if (positions[i] == objective)
            return i;
    }
    return -1;
}

void Topsis(double q[wlength][wlength], 
	std::vector<std::vector<double>> list,
	std::vector<std::pair<int, double>>& sortedlist)
{
    int size = list.size();
    AhpWeightCompute ahp(q);
    double weight[wlength];
    ahp.run(weight,wlength,AhpWeightComputeType::Hendesi);
    double normalize[wlength];
    for (int i = 0; i < wlength; i++) {
        normalize[i] = 0;
        for (int k = 0; k < size; k++) {
            normalize[i] += pow(list[k][i], 2);
        }
        normalize[i] = sqrt(normalize[i]);
    }
    double best[wlength];
    double worse[wlength];
    for (int i = 0; i < wlength; i++) {
        best[i] = DBL_MAX;
        worse[i] = 0;
    }
    for (int i = 0; i < size; i++) {
        for (int k = 0; k < wlength; k++) {
            if (normalize[k] != 0)
                list[i][k] = (double) list[i][k] * weight[k] / normalize[k];
            else if (list[i][k] == 0)
                list[i][k] = 0;
            else
                throw cRuntimeError("Topsis tries to normalize by 0");
            if (list[i][k] > worse[k])
                worse[k] = list[i][k];
            if (list[i][k] < best[k])
                best[k] = list[i][k];
        }
    }
    std::vector<double> sPlus;
    sPlus.resize(size);
    std::vector<double> sMinus;
    sMinus.resize(size);
    for (int i = 0; i < size; i++) {
        sPlus[i] = sMinus[i] = 0;
        for (int k = 0; k < wlength; k++) {
            sPlus[i] += pow(list[i][k]- best[k], 2);
            if (isnan(sPlus[i]))
                throw cRuntimeError("Results is not a number");
            sMinus[i] += pow(list[i][k]- worse[k], 2);
            if (isnan(sMinus[i]))
                throw cRuntimeError("Results is not a number");
        }
        sPlus[i] = sqrt(sPlus[i]);
        sMinus[i] = sqrt(sMinus[i]);
    }
    for (int i = 0; i < size; i++) {
        double val = (double)sMinus[i]/(sPlus[i]+sMinus[i]+1e-10);
        if (isnan(val))
            throw cRuntimeError("Results is not a number");

        sortedlist.push_back(std::make_pair(i, val));
    }
    std::sort(sortedlist.begin(), sortedlist.end(),
            [=](std::pair<int, double>& a, std::pair<int, double>& b)
            {
        return a.second > b.second;
            }
    );
}

TopsisLoadNgNeigbors::Cost TopsisLoadNgNeigbors::addCost(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower, const double &energy, const double &hops, const double &delay, L3Address common)
{
    // first search in the list
    Cost cost;
    cost.address = dest;
    cost.nextAddress = next;
    cost.etx = etx;
    cost.neighbors = neighbors;
    cost.snir = snir;
    cost.recPower = recPower;
    cost.energy = energy;
    cost.numHops = hops;
    cost.delay = delay;
    cost.commonAddress = common;
    memset(cost.addrSrt,0,sizeof(cost.addrSrt));
    memcpy(cost.addrSrt,dest.str().c_str(),dest.str().size());
    addCost(cost);
    return cost;
}

void TopsisLoadNgNeigbors::addCost(const TopsisLoadNgNeigbors::Cost &cost)
{
    // first search in the list
    auto it = std::find(listNodes.begin(), listNodes.end(), cost);
    if (it != listNodes.end()) {
        // check 3 hops possibility
        if (it->commonAddress == cost.commonAddress)
            (*it) = cost;
        else
            listNodes.push_back(cost); // alternate path
    }
    else
        listNodes.push_back(cost);
    change = true;
    sortedlist.clear();
}


TopsisLoadNgNeigbors::Cost TopsisLoadNgNeigbors::addCostNoCheck(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower, const double &energy, const double &hops, const double &delay, L3Address common)
{
    // first search in the list
    Cost cost;
    cost.address = dest;
    cost.nextAddress = next;
    cost.etx = etx;
    cost.neighbors = neighbors;
    cost.snir = snir;
    cost.recPower = recPower;
    cost.energy = energy;
    cost.numHops = hops;
    cost.delay = delay;
    cost.commonAddress = common;
    listNodes.push_back(cost);
    change = true;
    sortedlist.clear();
    return cost;
}

void TopsisLoadNgNeigbors::runTopsis(std::vector<std::pair<double, Cost>> &results) {
    if (!change && !sortedlist.empty()) {
        results.clear();
        for (const auto &elem: sortedlist)  {
            results.push_back(std::make_pair(elem.second, listNodes[elem.first]));
        }
        return;
    }
    change = false;


    std::vector<std::vector<double>> list;
    for (auto elem : listNodes) {
        std::vector<double> v;
        v.resize(wlength);


        //Etx, HC, number of neighbors, Rec power, snir, remained energy

        // Etx
        auto p = searchPosition(pos,wlength, ETX);
        if (p < 0 || p >= wlength) {
            throw cRuntimeError("Topsis, Invalid cost position");
        }
        if (!isnan(elem.etx))
            v[p] = elem.etx;
        else
            v[p]= 1; // Max
        // hop count
        p = searchPosition(pos,wlength, HC);
        if (p < 0 || p >= wlength) {
            throw cRuntimeError("Topsis, Invalid cost position");
        }
        if (!isnan(elem.numHops))
            v[p] = elem.numHops;
        else
            v[p] = 1; // Max

        // number of neighbors
        p = searchPosition(pos,wlength, NN);
        if (p < 0 || p >= wlength) {
            throw cRuntimeError("Topsis, Invalid cost position");
        }

        if (!isnan(elem.neighbors))
            v[p]= elem.neighbors;
        else
            v[p] = 1;

        // RECEIVED POWER
        p = searchPosition(pos,wlength, RECP);
        if (p < 0 || p >= wlength) {
            throw cRuntimeError("Topsis, Invalid cost position");
        }
        if (!isnan(elem.recPower))
            v[p] = (1/(elem.recPower+1e-10));
        else
            v[p] = (1/(511+1e-10)); // Max


        // SNIR
        p = searchPosition(pos,wlength, SNIR);
        if (p < 0 || p >= wlength) {
            throw cRuntimeError("Topsis, Invalid cost position");
        }

        if (!isnan(elem.snir))
            v[p] = (1/(elem.snir+1e-10));
        else
            v[p] = (1/(511+1e-10));

        // ENERGY
        p = searchPosition(pos,wlength, ENERGY);
        if (p < 0 || p >= wlength) {
            throw cRuntimeError("Topsis, Invalid cost position");
        }

        if (!isnan(elem.energy))
            v[p] = (1/(elem.energy+1e-10));
        else
            v[p] = (1/1e10);

        list.push_back(v);
    }

    Topsis(q,list,sortedlist);
    results.clear();
    for (const auto &elem: sortedlist)  {
        results.push_back(std::make_pair(elem.second, listNodes[elem.first]));
    }
}

void TopsisLoadNgNeigbors::runTopsisRoutes(std::map<L3Address, Cost> &results, bool printData)
{
    std::vector<int> index;
    runTopsisRoutes(results, index, printData);
}

void TopsisLoadNgNeigbors::runTopsisRoutes(std::map<L3Address, Cost> &results, std::vector<int> &index, bool printData)
{

    results.clear();
    index.clear();

    if (listNodes.empty())
        return;

    if (!change && !sortedlist.empty()) {
        for (const auto &elem: sortedlist)  {
            auto it = results.find(listNodes[elem.first].address);
            if (it == results.end())
                results.insert(std::make_pair(listNodes[elem.first].address, listNodes[elem.first]));
            index.push_back(elem.first);
        }
        if (printData) {
            FILE * f = fopen("Val", "a");
            fprintf(f, "=============== Input Data ============================\n");
            for (const auto &elem: sortedlist)  {
                fprintf(f, "ID %s Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f Next %s\n",listNodes[elem.first].address.str().c_str(), elem.second, listNodes[elem.first].energy, listNodes[elem.first].snir,
                        listNodes[elem.first].recPower, listNodes[elem.first].neighbors, listNodes[elem.first].numHops, listNodes[elem.first].etx,  listNodes[elem.first].nextAddress.str().c_str());
            }
            fclose(f);
        }
        if (printData) {
            FILE * f = fopen("Val", "a");
            fprintf(f, "=============== Results Data ============================\n");
            for (const auto &elem: results)  {
                fprintf(f, "Id %s Next %s - Etx %f, num Hopos %f  \n", elem.first.str().c_str(), elem.second.nextAddress.str().c_str(), elem.second.etx, elem.second.numHops);
            }
            fclose(f);
        }
        return;
    }
    change = false;

    if (listNodes.size() > 1) {
        std::vector<std::vector<double>> list;
        for (auto elem : listNodes) {
            std::vector<double> v;
            //Etx, HC, number of neighbors, Rec power, snir, remained energy
            v.resize(wlength);
            // Etx
            auto p = searchPosition(pos,wlength, ETX);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }
            if (!isnan(elem.etx))
                v[p] = elem.etx;
            else
                v[p]= 1; // Max
            // hop count
            p = searchPosition(pos,wlength, HC);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }
            if (!isnan(elem.numHops))
                v[p] = elem.numHops;
            else
                v[p] = 1; // Max

            // number of neighbors
            p = searchPosition(pos,wlength, NN);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }

            if (!isnan(elem.neighbors))
                v[p]= elem.neighbors;
            else
                v[p] = 1;

            // RECEIVED POWER
            p = searchPosition(pos,wlength, RECP);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }
            if (!isnan(elem.recPower))
                v[p] = (1/(elem.recPower+1e-10));
            else
                v[p] = (1/(511+1e-10)); // Max


            // SNIR
            p = searchPosition(pos,wlength, SNIR);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }

            if (!isnan(elem.snir))
                v[p] = (1/(elem.snir+1e-10));
            else
                v[p] = (1/(511+1e-10));

            // ENERGY
            p = searchPosition(pos,wlength, ENERGY);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }

            if (!isnan(elem.energy))
                v[p] = (1/(elem.energy+1e-10));
            else
                v[p] = (1/1e10);

            list.push_back(v);
        }
        Topsis(q,list,sortedlist);
    }
    else {
        sortedlist.push_back(std::make_pair(0,1.0));
    }

    for (auto elem : sortedlist) {
        index.push_back(elem.first);
    }

#ifdef PRINTRESULTS
    FILE * f = fopen("Val", "a");
    for (const auto &elem: sortedlist)  {
        fprintf(f, "Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f \n", elem.second, listNodes[elem.first].energy, listNodes[elem.first].snir,
                listNodes[elem.first].recPower, listNodes[elem.first].neighbors, listNodes[elem.first].numHops, listNodes[elem.first].etx);
        auto it = results.find(listNodes[elem.first].address);
        if (it == results.end())
            results.insert(std::make_pair(listNodes[elem.first].address, listNodes[elem.first]));
        else {
            auto cost = it->second;
            auto cost2 = listNodes[elem.first];
            EV_DEBUG << "Already in list ignore \n";
        }
    }
    fprintf(f, "===========================================\n");
    fclose(f);
#else
    if (printData) {
        FILE * f = fopen("Val", "a");
        fprintf(f, "=============== Input Data ============================\n");
        for (const auto &elem: sortedlist)  {
            fprintf(f, "ID %s Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f Next %s\n",listNodes[elem.first].address.str().c_str(), elem.second, listNodes[elem.first].energy, listNodes[elem.first].snir,
                    listNodes[elem.first].recPower, listNodes[elem.first].neighbors, listNodes[elem.first].numHops, listNodes[elem.first].etx,  listNodes[elem.first].nextAddress.str().c_str());
        }
        fclose(f);
    }
    for (const auto &elem: sortedlist)  {
        auto it = results.find(listNodes[elem.first].address);
        if (it == results.end()) {
            // before to insert check next address
            if (listNodes[elem.first].address == listNodes[elem.first].nextAddress)
                results.insert(std::make_pair(listNodes[elem.first].address, listNodes[elem.first]));
            else {
                auto it2 = results.find(listNodes[elem.first].nextAddress);
//                auto cost = listNodes[elem.first];
                if (it2 == results.end())
                    throw cRuntimeError("Next address is not found in the list");
                results.insert(std::make_pair(listNodes[elem.first].address, listNodes[elem.first]));
            }
        }
        else {
//            auto cost = it->second;
//            auto cost2 = listNodes[elem.first];
            // check indirection
            if (it->first != it->second.nextAddress) // at lest one hop
                EV_DEBUG << "Already in list ignore \n";
        }
    }
    if (printData) {
        FILE * f = fopen("Val", "a");
        fprintf(f, "=============== Results Data ============================\n");
        for (const auto &elem: results)  {
            fprintf(f, "Id %s Next %s - Etx %f, num Hopos %f  \n", elem.first.str().c_str(), elem.second.nextAddress.str().c_str(), elem.second.etx, elem.second.numHops);
        }
        fclose(f);
    }

#endif
}

void TopsisLoadNgNeigbors::runTopsisRoutes(std::deque<CostAndRange> &results)
{

    results.clear();
    if (listNodes.empty())
        return;
    if (!change && !sortedlist.empty()) {
        for (const auto &elem: sortedlist)  {
            CostAndRange res;
            res.cost = listNodes[elem.first];
            res.index = elem.first;
            res.range = elem.second;
            results.push_back(res);
        }
        return;
    }
    change = false;

    if (listNodes.size() > 1) {
        std::vector<std::vector<double>> list;
        for (auto elem : listNodes) {
            std::vector<double> v;
            v.resize(wlength);

            // Etx
            auto p = searchPosition(pos,wlength, ETX);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }
            if (!isnan(elem.etx))
                v[p] = elem.etx;
            else
                v[p]= 1; // Max
            // hop count
            p = searchPosition(pos,wlength, HC);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }
            if (!isnan(elem.numHops))
                v[p] = elem.numHops;
            else
                v[p] = 1; // Max

            // number of neighbors
            p = searchPosition(pos,wlength, NN);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }

            if (!isnan(elem.neighbors))
                v[p]= elem.neighbors;
            else
                v[p] = 1;

            // RECEIVED POWER
            p = searchPosition(pos,wlength, RECP);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }
            if (!isnan(elem.recPower))
                v[p] = (1/(elem.recPower+1e-10));
            else
                v[p] = (1/(511+1e-10)); // Max


            // SNIR
            p = searchPosition(pos,wlength, SNIR);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }

            if (!isnan(elem.snir))
                v[p] = (1/(elem.snir+1e-10));
            else
                v[p] = (1/(511+1e-10));

            // ENERGY
            p = searchPosition(pos,wlength, ENERGY);
            if (p < 0 || p >= wlength) {
                throw cRuntimeError("Topsis, Invalid cost position");
            }

            if (!isnan(elem.energy))
                v[p] = (1/(elem.energy+1e-10));
            else
                v[p] = (1/1e10);

            list.push_back(v);
        }
        Topsis(q,list,sortedlist);
    }
    else {
        sortedlist.push_back(std::make_pair(0,1.0));
    }

    for (const auto &elem: sortedlist)  {
        CostAndRange res;
        res.cost = listNodes[elem.first];
        res.index = elem.first;
        res.range = elem.second;
        results.push_back(res);
    }
}

void TopsisLoadNgNeigbors::setExperiment(int experiment) {
    if (experiment == 1)
        memcpy(q,q1,sizeof(q));
    else
        memcpy(q,q2,sizeof(q));

    switch (experiment) {
        case 1:
            memcpy(pos,pos1,sizeof(pos));
            break;
        case 2:
            memcpy(pos,pos2,sizeof(pos));
            break;
        case 3:
            memcpy(pos,pos3,sizeof(pos));
            break;
        case 4:
            memcpy(pos,pos4,sizeof(pos));
            break;
        case 5:
            memcpy(pos,pos5,sizeof(pos));
            break;
        case 6:
            memcpy(pos,pos6,sizeof(pos));
            break;
        case 7:
            memcpy(pos,pos7,sizeof(pos));
            break;
        case 8:
            memcpy(pos,pos8,sizeof(pos));
            break;
        case 9:
            memcpy(pos,pos9,sizeof(pos));
            break;
        case 10:
            memcpy(pos,pos10,sizeof(pos));
            break;
        case 11:
            memcpy(pos,pos11,sizeof(pos));
            break;
        default:
            throw cRuntimeError("Position not found");
    }
}


}
}
