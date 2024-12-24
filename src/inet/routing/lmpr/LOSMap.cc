#include "LMPR.h"

namespace inet
{

double LMPR::getRealCommRange(Coord PosA, Coord PosB)
{
    if(setAutoRange)
    {
        double range;
        bool nlos_cond = pathLoss->checkNlos(PosA, PosB);
        nlos_cond = uniform(0,1) <= losMapError ? !nlos_cond : nlos_cond;
        if(nlos_cond)
        {
//            range = 32.49;
            range = 51;
        }
        else
        {
//            range = 76.35;
            range = 130.4;
        }
        return range;
    }
    else
    {
//        return maxRangeForLET;
        return 130.4;
    }
}


double LMPR::getRangeForLET(Coord PosA, Coord PosB)
{
    if(setAutoLETRange)
    {
        double range;
        bool nlos_cond = pathLoss->checkNlos(PosA, PosB);
        nlos_cond = uniform(0,1) <= losMapError ? !nlos_cond : nlos_cond;
        if(nlos_cond)
        {
//            range = 32.49;
            range = 51;
        }
        else
        {
//            range = 76.35;
            range = 130.4;
        }
        return range;
    }
    else
    {
        return maxRangeForLET;
    }

}


}
