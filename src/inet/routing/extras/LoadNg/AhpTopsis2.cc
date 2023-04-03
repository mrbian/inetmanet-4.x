#include <algorithm>
#include <map>
#include <vector>
#include <cmath>
#include <math.h>

#include "AhpTopsis2.h"

namespace inet {
namespace inetmanet {


static void Topsis(double q[wlength][wlength],
    std::vector<std::vector<double>> list,
    std::vector<std::pair<int, double>>& sortedlist)
{
    int size = list.size();
    AhpWeightCompute2 ahp(q);
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


TopsisLoadNgNeigbors2::Cost TopsisLoadNgNeigbors2::addCost(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower, const double &energy, const double &hops, const double &delay, const bool &stable, L3Address common)
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
    cost.stable = stable;
    cost.commonAddress = common;
    memset(cost.addrSrt,0,sizeof(cost.addrSrt));
    memcpy(cost.addrSrt,dest.str().c_str(),dest.str().size());
    addCost(cost);
    return cost;
}

#ifdef TWOLIST


void TopsisLoadNgNeigbors2::addCost(const TopsisLoadNgNeigbors2::Cost &cost)
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
    sortedlistStable.clear();
    pos.clear();
    posStable.clear();
}


TopsisLoadNgNeigbors2::Cost TopsisLoadNgNeigbors2::addCostNoCheck(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower, const double &energy, const double &hops, const double &delay, const bool &stable, L3Address common)
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
    cost.stable = stable;
    cost.commonAddress = common;
    listNodes.push_back(cost);
    change = true;
    sortedlist.clear();
    sortedlistStable.clear();
    pos.clear();
    posStable.clear();
    return cost;
}

void TopsisLoadNgNeigbors2::runTopsis(std::vector<std::pair<double, Cost>> &results) {
    if (!change && (!sortedlist.empty() || !sortedlistStable.empty())) {
        results.clear();
        for (const auto &elem: sortedlist)  {
            results.push_back(std::make_pair(elem.second, listNodes[elem.first]));
        }
        return;
    }
    change = false;


    std::vector<std::vector<double>> listStable;
    std::vector<std::vector<double>> list;

    sortedlist.clear();
    pos.clear();
    sortedlistStable.clear();
    posStable.clear();

    for (unsigned int i = 0 ; i <  listNodes.size(); i++) {
        std::vector<double> v;
        //Etx, HC, number of neighbors, Rec power, snir, remained energy
        if (!isnan(listNodes[i].etx))
            v.push_back(listNodes[i].etx);
        else
            v.push_back(1); // Max

        if (!isnan(listNodes[i].numHops))
            v.push_back(listNodes[i].numHops);
        else
            v.push_back(1); // Max

        if (!isnan(listNodes[i].neighbors))
            v.push_back(listNodes[i].neighbors);
        else
            v.push_back(1);

        if (!isnan(listNodes[i].recPower))
            v.push_back(1/(listNodes[i].recPower+1e-10));
        else
            v.push_back(1/(511+1e-10)); // Max

        if (!isnan(listNodes[i].snir))
            v.push_back(1/(listNodes[i].snir+1e-10));
        else
            v.push_back(1/(511+1e-10));

        if (!isnan(listNodes[i].energy))
            v.push_back(1/(listNodes[i].energy+1e-10));
        else
            v.push_back(1/1e10);

        if (listNodes[i].stable)  {
            listStable.push_back(v);
            posStable.push_back(i);
        }
        else {
            list.push_back(v);
            pos.push_back(i);
        }
    }

    if (!list.empty())
        Topsis(q,list,sortedlist);
    if (!listStable.empty())
        Topsis(q,listStable,sortedlistStable);

    results.clear();
    for (const auto &elem: sortedlistStable)  {
        results.push_back(std::make_pair(elem.second, listNodes[posStable[elem.first]]));
    }
    for (const auto &elem: sortedlist)  {
        results.push_back(std::make_pair(elem.second, listNodes[pos[elem.first]]));
    }
}

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::map<L3Address, Cost> &results, bool printData)
{
    std::vector<int> index;
    runTopsisRoutes(results, index, printData);
}

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::map<L3Address, Cost> &results, std::vector<int> &index, bool printData)
{

    results.clear();
    index.clear();

    if (listNodes.empty())
        return;

    if (!change && (!sortedlist.empty() || !sortedlistStable.empty())) {
        for (const auto &elem: sortedlistStable)  {
            int id = posStable[elem.first];
            auto it = results.find(listNodes[id].address);
            if (it == results.end())
                results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
            index.push_back(id);
        }
        for (const auto &elem: sortedlist)  {
            int id = pos[elem.first];
            auto it = results.find(listNodes[id].address);
            if (it == results.end())
                results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
            index.push_back(id);
        }
        if (printData) {
            FILE * f = fopen("Val", "a");
            fprintf(f, "=============== Input Data ============================\n");
            for (const auto &elem: sortedlistStable)  {
                int id = posStable[elem.first];
                fprintf(f, "ID %s Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f Next %s Stable = true \n",listNodes[id].address.str().c_str(), elem.second, listNodes[id].energy, listNodes[id].snir,
                        listNodes[id].recPower, listNodes[id].neighbors, listNodes[id].numHops, listNodes[id].etx,  listNodes[id].nextAddress.str().c_str());
            }

            for (const auto &elem: sortedlist)  {
                int id = pos[elem.first];
                fprintf(f, "ID %s Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f Next %s Stable = false \n",listNodes[id].address.str().c_str(), elem.second, listNodes[id].energy, listNodes[id].snir,
                        listNodes[id].recPower, listNodes[id].neighbors, listNodes[id].numHops, listNodes[id].etx,  listNodes[id].nextAddress.str().c_str());
            }
            fclose(f);
        }
        if (printData) {
            FILE * f = fopen("Val", "a");
            fprintf(f, "=============== Results Data ============================\n");
            for (const auto &elem: results)  {
                fprintf(f, "Id %s Next %s - Etx %f, num Hopos %f  stable = %i \n", elem.first.str().c_str(), elem.second.nextAddress.str().c_str(), elem.second.etx, elem.second.numHops, elem.second.stable);
            }
            fclose(f);
        }
        return;
    }
    change = false;
    sortedlist.clear();
    pos.clear();
    sortedlistStable.clear();
    posStable.clear();

    if (listNodes.size() > 1) {

        std::vector<std::vector<double>> listStable;
        std::vector<std::vector<double>> list;

        posStable.clear();
        pos.clear();

        for (unsigned int i = 0; i < listNodes.size(); i++) {
            std::vector<double> v;
            //Etx, HC, number of neighbors, Rec power, snir, remained energy
            if (!isnan(listNodes[i].etx))
                v.push_back(listNodes[i].etx);
            else
                v.push_back(1); // Max

            if (!isnan(listNodes[i].numHops))
                v.push_back(listNodes[i].numHops);
            else
                v.push_back(1); // Max

            if (!isnan(listNodes[i].neighbors))
                v.push_back(listNodes[i].neighbors);
            else
                v.push_back(1);

            if (!isnan(listNodes[i].recPower))
                v.push_back(1/(listNodes[i].recPower+1e-10));
            else
                v.push_back(1/(511+1e-10)); // Max

            if (!isnan(listNodes[i].snir))
                v.push_back(1/(listNodes[i].snir+1e-10));
            else
                v.push_back(1/(511+1e-10));

            if (!isnan(listNodes[i].energy))
                v.push_back(1/(listNodes[i].energy+1e-10));
            else
                v.push_back(1/1e10);

            if (listNodes[i].stable)  {
                listStable.push_back(v);
                posStable.push_back(i);
            }
            else {
                list.push_back(v);
                pos.push_back(i);
            }
        }
        if (!list.empty())
            Topsis(q,list,sortedlist);
        if (!listStable.empty())
            Topsis(q,listStable,sortedlistStable);
    }
    else {
        sortedlist.clear();
        sortedlistStable.clear();
        pos.clear();
        posStable.clear();
        if (listNodes.front().stable) {
            sortedlistStable.push_back(std::make_pair(0,1.0));
            posStable.push_back(0);
        }
        else {
            sortedlist.push_back(std::make_pair(0,1.0));
            pos.push_back(0);
        }
    }

    for (auto elem : sortedlistStable) {
        int id = posStable[elem.first];
        index.push_back(id);
    }

    for (auto elem : sortedlist) {
        int id = pos[elem.first];
        index.push_back(id);
    }

#ifdef PRINTRESULTS
    FILE * f = fopen("Val", "a");
    for (const auto &elem: sortedlistStable)  {
        int id = posStable[elem.first];
        fprintf(f, "Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f stable = true \n", elem.second, listNodes[id].energy, listNodes[id].snir,
                listNodes[id].recPower, listNodes[id].neighbors, listNodes[id].numHops, listNodes[id].etx);
        auto it = results.find(listNodes[id].address);
        if (it == results.end())
            results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
        else {
            auto cost = it->second;
            auto cost2 = listNodes[id];
            EV_DEBUG << "Already in list ignore \n";
        }
    }
    for (const auto &elem: sortedlist)  {
        int id = pos[elem.first];
        fprintf(f, "Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f stable = false \n", elem.second, listNodes[id].energy, listNodes[id].snir,
                listNodes[id].recPower, listNodes[id].neighbors, listNodes[id].numHops, listNodes[id].etx);
        auto it = results.find(listNodes[id].address);
        if (it == results.end())
            results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
        else {
            auto cost = it->second;
            auto cost2 = listNodes[id];
            EV_DEBUG << "Already in list ignore \n";
        }
    }
    fprintf(f, "===========================================\n");
    fclose(f);
#else
    if (printData) {
        FILE * f = fopen("Val", "a");
        fprintf(f, "=============== Input Data ============================\n");
        for (const auto &elem: sortedlistStable)  {
            int id = posStable[elem.first];
            fprintf(f, "ID %s Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f Next %s stable = true\n",listNodes[id].address.str().c_str(), elem.second, listNodes[id].energy, listNodes[id].snir,
                    listNodes[id].recPower, listNodes[id].neighbors, listNodes[id].numHops, listNodes[id].etx,  listNodes[id].nextAddress.str().c_str());
        }
        for (const auto &elem: sortedlist)  {
            int id = pos[elem.first];
            fprintf(f, "ID %s Range %f -  Remaining E = %f Snir = %f Power = %f neig = %f, num hops=%f, ETX = %f Next %s stable = false\n",listNodes[id].address.str().c_str(), elem.second, listNodes[id].energy, listNodes[id].snir,
                    listNodes[id].recPower, listNodes[id].neighbors, listNodes[id].numHops, listNodes[id].etx,  listNodes[id].nextAddress.str().c_str());
        }
        fclose(f);
    }
    for (const auto &elem: sortedlistStable)  {
        int id = posStable[elem.first];
        auto it = results.find(listNodes[id].address);
        if (it == results.end()) {
            // before to insert check next address
            if (listNodes[id].address == listNodes[id].nextAddress)
                results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
            else {
                auto it2 = results.find(listNodes[id].nextAddress);
                auto cost = listNodes[id];
                if (it2 == results.end())
                    throw cRuntimeError("Next address is not found in the list");
                results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
            }
        }
        else {
            auto cost = it->second;
            auto cost2 = listNodes[id];
            // check indirection
            if (it->first != it->second.nextAddress) // at lest one hop
            EV_DEBUG << "Already in list ignore \n";
        }
    }
    for (const auto &elem: sortedlist)  {
        int id = pos[elem.first];
        auto it = results.find(listNodes[id].address);
        if (it == results.end()) {
            // before to insert check next address
            if (listNodes[id].address == listNodes[id].nextAddress)
                results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
            else {
                auto it2 = results.find(listNodes[id].nextAddress);
                auto cost = listNodes[id];
                if (it2 == results.end())
                    throw cRuntimeError("Next address is not found in the list");
                results.insert(std::make_pair(listNodes[id].address, listNodes[id]));
            }
        }
        else {
            auto cost = it->second;
            auto cost2 = listNodes[id];
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

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::deque<CostAndRange> &results)
{

    results.clear();
    if (listNodes.empty())
        return;
    if (!change && (!sortedlist.empty() || !sortedlistStable.empty())) {
        for (const auto &elem: sortedlistStable)  {
            CostAndRange res;
            int index = posStable[elem.first];
            res.cost = listNodes[index];
            res.index = index;
            res.range = elem.second;
            results.push_back(res);
        }
        for (const auto &elem: sortedlist)  {
            CostAndRange res;
            int index = pos[elem.first];
            res.cost = listNodes[index];
            res.index = index;
            res.range = elem.second;
            results.push_back(res);
        }
        return;
    }
    change = false;
    sortedlist.clear();
    pos.clear();
    sortedlistStable.clear();
    posStable.clear();

    if (listNodes.size() > 1) {
        posStable.clear();
        pos.clear();
        std::vector<std::vector<double>> listStable;
        std::vector<std::vector<double>> list;
        for (unsigned int i = 0; i < listNodes.size(); i++) {
            std::vector<double> v;

            if (!isnan(listNodes[i].etx))
                v.push_back(listNodes[i].etx);
            else
                v.push_back(1); // Max

            if (!isnan(listNodes[i].numHops))
                v.push_back(listNodes[i].numHops);
            else
                v.push_back(1); // Max

            if (!isnan(listNodes[i].neighbors))
                v.push_back(listNodes[i].neighbors);
            else
                v.push_back(1);

            if (!isnan(listNodes[i].recPower))
                v.push_back(1/(listNodes[i].recPower+1e-10));
            else
                v.push_back(1/(511+1e-10)); // Max

            if (!isnan(listNodes[i].snir))
                v.push_back(1/(listNodes[i].snir+1e-10));
            else
                v.push_back(1/(511+1e-10));

            if (!isnan(listNodes[i].energy))
                v.push_back(1/(listNodes[i].energy+1e-10));
            else
                v.push_back(1/1e10);

            if (listNodes[i].stable)  {
                listStable.push_back(v);
                posStable.push_back(i);
            }
            else {
                list.push_back(v);
                pos.push_back(i);
            }
        }
        if (!list.empty())
            Topsis(q,list,sortedlist);
        if (!listStable.empty())
            Topsis(q,listStable,sortedlistStable);
    }
    else {
        sortedlist.clear();
        sortedlistStable.clear();
        pos.clear();
        posStable.clear();
        if (listNodes.front().stable) {
            sortedlistStable.push_back(std::make_pair(0,1.0));
            posStable.push_back(0);
        }
        else {
            sortedlist.push_back(std::make_pair(0,1.0));
            pos.push_back(0);
        }
    }

    for (const auto &elem: sortedlistStable)  {
        CostAndRange res;
        int id = posStable[elem.first];
        res.cost = listNodes[id];
        res.index = id;
        res.range = elem.second;
        results.push_back(res);
    }

    for (const auto &elem: sortedlist)  {
        CostAndRange res;
        int id = pos[elem.first];
        res.cost = listNodes[id];
        res.index = id;
        res.range = elem.second;
        results.push_back(res);
    }
}
#elif defined(OLDLIST)

void TopsisLoadNgNeigbors2::addCost(const TopsisLoadNgNeigbors2::Cost &cost)
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


TopsisLoadNgNeigbors2::Cost TopsisLoadNgNeigbors2::addCostNoCheck(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower, const double &energy, const double &hops, const double &delay, const bool &stable, L3Address common)
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
    cost.stable = stable;
    cost.commonAddress = common;
    listNodes.push_back(cost);
    change = true;
    sortedlist.clear();
    return cost;
}

void TopsisLoadNgNeigbors2::runTopsis(std::vector<std::pair<double, Cost>> &results) {
    if (!change && (!sortedlist.empty() || !sortedlistStable.empty())) {
        results.clear();
        for (const auto &elem: sortedlist)  {
            results.push_back(std::make_pair(elem.second, listNodes[elem.first]));
        }
        return;
    }
    change = false;

    std::vector<std::vector<double>> list;
    sortedlist.clear();

    for (unsigned int i = 0 ; i <  listNodes.size(); i++) {
        std::vector<double> v;
        //Etx, HC, number of neighbors, Rec power, snir, remained energy
        if (!isnan(listNodes[i].etx))
            v.push_back(listNodes[i].etx);
        else
            v.push_back(1); // Max

        if (!isnan(listNodes[i].numHops))
            v.push_back(listNodes[i].numHops);
        else
            v.push_back(1); // Max

        if (!isnan(listNodes[i].neighbors))
            v.push_back(listNodes[i].neighbors);
        else
            v.push_back(1);

        if (!isnan(listNodes[i].recPower))
            v.push_back(1/(listNodes[i].recPower+1e-10));
        else
            v.push_back(1/(511+1e-10)); // Max

        if (!isnan(listNodes[i].snir))
            v.push_back(1/(listNodes[i].snir+1e-10));
        else
            v.push_back(1/(511+1e-10));

        if (!isnan(listNodes[i].energy))
            v.push_back(1/(listNodes[i].energy+1e-10));
        else
            v.push_back(1/1e10);

        list.push_back(v);
    }

    if (!list.empty())
        Topsis(q,list,sortedlist);

    results.clear();
    for (const auto &elem: sortedlist)  {
        results.push_back(std::make_pair(elem.second, listNodes[elem.first]));
    }
}

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::map<L3Address, Cost> &results, bool printData)
{
    std::vector<int> index;
    runTopsisRoutes(results, index, printData);
}

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::map<L3Address, Cost> &results, std::vector<int> &index, bool printData)
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
             if (!isnan(elem.etx))
                 v.push_back(elem.etx);
             else
                 v.push_back(1);

             if (!isnan(elem.numHops))
                 v.push_back(elem.numHops);
             else
                 v.push_back(1);

             if (!isnan(elem.neighbors))
                 v.push_back(elem.neighbors);
             else
                 v.push_back(1);

             if (!isnan(elem.recPower))
                 v.push_back(-elem.recPower);
             else
                 v.push_back(-511);

             if (!isnan(elem.snir))
                 v.push_back(-elem.snir);
             else
                 v.push_back(-511);

             if (!isnan(elem.energy))
                 v.push_back(-elem.energy);
             else
                 v.push_back(-1e300);

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
                // auto cost = listNodes[elem.first];
                 if (it2 == results.end())
                     throw cRuntimeError("Next address is not found in the list");
                 results.insert(std::make_pair(listNodes[elem.first].address, listNodes[elem.first]));
             }
         }
         else {
             //auto cost = it->second;
             //auto cost2 = listNodes[elem.first];
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

 void TopsisLoadNgNeigbors2::runTopsisRoutes(std::deque<CostAndRange> &results)
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

             if (!isnan(elem.etx))
                 v.push_back(elem.etx);
             else
                 v.push_back(1);

             if (!isnan(elem.numHops))
                 v.push_back(elem.numHops);
             else
                 v.push_back(1);

             if (!isnan(elem.neighbors))
                 v.push_back(elem.neighbors);
             else
                 v.push_back(1);

             if (!isnan(elem.recPower))
                 v.push_back(-elem.recPower);
             else
                 v.push_back(-511);

             if (!isnan(elem.snir))
                 v.push_back(-elem.snir);
             else
                 v.push_back(-511);

             if (!isnan(elem.energy))
                 v.push_back(-elem.energy);
             else
                 v.push_back(-1e300);


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

#else

void TopsisLoadNgNeigbors2::addCost(const TopsisLoadNgNeigbors2::Cost &cost)
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


TopsisLoadNgNeigbors2::Cost TopsisLoadNgNeigbors2::addCostNoCheck(const L3Address &dest, const L3Address &next, const double &etx, const double &neighbors, const double &snir, const double &recPower, const double &energy, const double &hops, const double &delay, const bool &stable, L3Address common)
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
    cost.stable = stable;
    cost.commonAddress = common;
    listNodes.push_back(cost);
    change = true;
    sortedlist.clear();
    return cost;
}


void TopsisLoadNgNeigbors2::runTopsis(std::vector<std::pair<double, Cost>> &results) {
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
        //Etx, HC, number of neighbors, Rec power, snir, remained energy
        if (elem.stable)
            v.push_back(0.0);
        else
            v.push_back(1.0);

        if (!isnan(elem.etx))
            v.push_back(elem.etx);
        else
            v.push_back(1); // Max

        if (!isnan(elem.numHops))
            v.push_back(elem.numHops);
        else
            v.push_back(1); // Max

        if (!isnan(elem.neighbors))
            v.push_back(elem.neighbors);
        else
            v.push_back(1);

        if (!isnan(elem.recPower))
            v.push_back(-elem.recPower);
        else
            v.push_back(-511); // Max

        if (!isnan(elem.snir))
            v.push_back(-elem.snir);
        else
            v.push_back(-511);

        if (!isnan(elem.energy))
            v.push_back(-elem.energy);
        else
            v.push_back(-1e300);

        list.push_back(v);
    }

    Topsis(q,list,sortedlist);
    results.clear();
    for (const auto &elem: sortedlist)  {
        results.push_back(std::make_pair(elem.second, listNodes[elem.first]));
    }
}

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::map<L3Address, Cost> &results, bool printData)
{
    std::vector<int> index;
    runTopsisRoutes(results, index, printData);
}

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::map<L3Address, Cost> &results, std::vector<int> &index, bool printData)
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
            if (elem.stable)
                v.push_back(0.0);
            else
                v.push_back(1.0);

            if (!isnan(elem.etx))
                v.push_back(elem.etx);
            else
                v.push_back(1);

            if (!isnan(elem.numHops))
                v.push_back(elem.numHops);
            else
                v.push_back(1);

            if (!isnan(elem.neighbors))
                v.push_back(elem.neighbors);
            else
                v.push_back(1);

            if (!isnan(elem.recPower))
                v.push_back(-elem.recPower);
            else
                v.push_back(-511);

            if (!isnan(elem.snir))
                v.push_back(-elem.snir);
            else
                v.push_back(-511);

            if (!isnan(elem.energy))
                v.push_back(-elem.energy);
            else
                v.push_back(-1e300);

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
                auto cost = listNodes[elem.first];
                if (it2 == results.end())
                    throw cRuntimeError("Next address is not found in the list");
                results.insert(std::make_pair(listNodes[elem.first].address, listNodes[elem.first]));
            }
        }
        else {
            auto cost = it->second;
            auto cost2 = listNodes[elem.first];
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

void TopsisLoadNgNeigbors2::runTopsisRoutes(std::deque<CostAndRange> &results)
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
            if (elem.stable)
                v.push_back(0.0);
            else
                v.push_back(1.0);

            if (!isnan(elem.etx))
                v.push_back(elem.etx);
            else
                v.push_back(1);

            if (!isnan(elem.numHops))
                v.push_back(elem.numHops);
            else
                v.push_back(1);

            if (!isnan(elem.neighbors))
                v.push_back(elem.neighbors);
            else
                v.push_back(1);

            if (!isnan(elem.recPower))
                v.push_back(-elem.recPower);
            else
                v.push_back(-511);

            if (!isnan(elem.snir))
                v.push_back(-elem.snir);
            else
                v.push_back(-511);

            if (!isnan(elem.energy))
                v.push_back(-elem.energy);
            else
                v.push_back(-1e300);


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


#endif

}
}
