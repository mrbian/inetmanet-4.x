/*
 * Prels_brains.cc
 *
 *  Created on: Dec 30, 2024
 *      Author: bman
 */

#include "Prels.h"

namespace inet
{

#ifndef state_
#define state_ (*state_ptr)
#endif

simsignal_t mobInfoAverLapseSignal = cComponent::registerSignal("mobInfoAverLapse");
simsignal_t mobInfoMaxLapseSignal = cComponent::registerSignal("mobInfoMaxLapse");
simsignal_t mobInfoMinLapseSignal = cComponent::registerSignal("mobInfoMinLapse");
simsignal_t mobInfoCountSignal = cComponent::registerSignal("mobInfoCount");

void
Prels::rtable_computation()
{
    double max_lapse = 0;
    double min_lapse = 100;
    double total_lapse = 0;
    double now = CURRENT_TIME;
    for(auto it = state_.nodeset().begin(); it != state_.nodeset().end(); ++it)
    {
        Prels_node_tuple * node_tuple = (*it);
        double update_time = node_tuple->mob_update_time();
        double lapse = now - update_time;
        if(lapse > max_lapse)
        {
            max_lapse = lapse;
        }
        if(lapse < min_lapse)
        {
            min_lapse = lapse;
        }
        total_lapse += lapse;
    }
    int count = state_.nodeset().size();
    emit(mobInfoCountSignal, count);
    double aver_lapse = total_lapse / state_.nodeset().size();
    if(now > 10)
    {
        emit(mobInfoAverLapseSignal, aver_lapse);
        emit(mobInfoMaxLapseSignal, max_lapse);
        emit(mobInfoMinLapseSignal, min_lapse);
    }
}



}


