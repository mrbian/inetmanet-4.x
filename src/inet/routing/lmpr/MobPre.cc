#include "LMPR.h"

namespace inet
{

Coord LMPR::forecastSelfPosition_Optimal(double duration)
{
    ExtendedBonnMotionMobility *mob = check_and_cast<ExtendedBonnMotionMobility*>(mobility);
    Coord fut = mob->getFuturePosition(duration, simTime());
    return fut;
}

Coord LMPR::forecastNodePosition_Optimal(int idx, double duration)
{
    ExtendedBonnMotionMobility *mob = check_and_cast<ExtendedBonnMotionMobility*>(_globalMob[idx].second);
    Coord fut = mob->getFuturePosition(duration, simTime());
    return fut;
}

// obstacle, prediction error must be consider
Coord LMPR::forecastNodeLocation_by_ThreePos(NodeInfo* node_info, simtime_t t)
{
//    Ipv4Address addr;
//    double time_offset;
//    Trajectory trs;
//    int last_bcast_seqno;
//    simtime_t bcast_seqno_reset;
//    simtime_t last_seen;
    double timeOffset = t.dbl() - node_info->time_abstract;
    Trajectory trs = node_info->trs;
    Coord P1, P2, P3, v1, v2;
    P1.x = trs.x1;
    P1.y = trs.y1;
    P2.x = trs.x2;
    P2.y = trs.y2;
    P3.x = trs.x3;
    P3.y = trs.y3;
    v1 = (P2 - P1)/predictDuration;
    v2 = (P3 - P2)/predictDuration;
    Coord furPos;
    if(timeOffset <= predictDuration)
    {
        furPos = P1 + v1 * predictDuration;
    }
    else if(timeOffset > predictDuration && timeOffset <= predictDuration*2)
    {
        furPos = P2 + v1 * (timeOffset - predictDuration);
    }
    else
    {
        furPos = P3 + v2 * (timeOffset - 2*predictDuration);
    }
    return furPos;
}


}
