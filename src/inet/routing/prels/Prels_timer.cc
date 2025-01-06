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

#include "Prels_timer.h"
#include "Prels.h"

namespace inet {

void Prels_Timer::removeTimer()
{
    removeQueueTimer();
}

Prels_Timer::Prels_Timer(Prels* agent) : cOwnedObject("PrelsTimer")
{
    agent_ = agent;
}

Prels_Timer::~Prels_Timer()
{
    removeTimer();
}

Prels_Timer::Prels_Timer() : cOwnedObject("PrelsTimer")
{
    agent_ = dynamic_cast <Prels*> (this->getOwner());
    if (agent_==nullptr)
        throw cRuntimeError("timer ower is bad");
}

void Prels_Timer::removeQueueTimer()
{
    TimerMultiMap::iterator it;
    for (it=agent_->getTimerMultimMap()->begin(); it != agent_->getTimerMultimMap()->end();  ) {
        if (it->second == this)
            agent_->getTimerMultimMap()->erase(it++);
        else
            ++it;
    }
}

void Prels_Timer::resched(double time)
{
    removeQueueTimer();
    if (simTime() + time <= simTime())
        throw cRuntimeError("Prels_Timer::resched message timer in the past");
    // Search first
    agent_->getTimerMultimMap()->insert(std::pair<simtime_t, Prels_Timer *>(simTime()+time, this));
    agent_->scheduleEvent();
}

void Prels_Timer::resched(simtime_t time)
{
    removeQueueTimer();
    if (time <= simTime())
        throw cRuntimeError("Prels_Timer::resched message timer in the past");
    agent_->getTimerMultimMap()->insert(std::pair<simtime_t, Prels_Timer *>(time, this));
}

bool Prels_Timer::isScheduled()
{
    TimerMultiMap::iterator it;
    for (it=agent_->getTimerMultimMap()->begin() ; it != agent_->getTimerMultimMap()->end(); ++it ) {
        if (it->second==this) {
            return true;
        }
    }
    return false;
}

std::ostream& operator<<(std::ostream& os, const Prels_Timer& e)
{
    os << e.getClassName() << " ";
    return os;
};

void
Prels_SendPktTimer::expire()
{
    auto ag = agent_;
    ag->send_pkt();
    removeTimer();
    delete this;
}


void
Prels_HelloTimer::expire()
{
    auto ag = agent_;
    ag->send_hello();
    // agent_->scheduleAt(simTime()+agent_->hello_ival_- JITTER,this);
    simtime_t next = simTime() + ag->hello_ival() - ag->jitter();
    this->resched(next);
}


void
Prels_TcTimer::expire()
{
    auto ag = agent_;
    if (ag->mprselset().size() > 0)
        ag->send_tc();
    // agent_->scheduleAt(simTime()+agent_->tc_ival_- JITTER,this);
    simtime_t next = simTime() + ag->tc_ival() - ag->jitter();
    this->resched(next);
}

Prels_NbTupleTimer::Prels_NbTupleTimer(Prels* agent, Prels_nb_tuple* tuple) : Prels_Timer(agent)
{
    agent_ = agent;
    tuple_ = tuple;
}

void
Prels_NbTupleTimer::expire()
{
    Prels_nb_tuple* tuple = dynamic_cast<Prels_nb_tuple*> (tuple_);
    double time = tuple->lost_time();
    if (time < SIMTIME_DBL(simTime()))
    {
        removeTimer();
        delete this;
    }
    else
    {
        simtime_t next = simTime()+DELAY_T(time);
        this->resched(next);
    }
}

Prels_NbTupleTimer::~Prels_NbTupleTimer()
{
    removeTimer();
    if (!tuple_)
        return;
    Prels_nb_tuple* tuple = dynamic_cast<Prels_nb_tuple*> (tuple_);
    tuple->asocTimer = nullptr;
    auto ag = agent_;
    if (ag->state_ptr == nullptr)
        return;
    ag->rm_nb_tuple(tuple);
    delete tuple_;
}




} /* namespace inet */
