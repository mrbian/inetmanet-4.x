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

#ifndef INET_ROUTING_PRELS_PRELSTIMER_H_
#define INET_ROUTING_PRELS_PRELSTIMER_H_

#include <map>
#include <vector>
#include <assert.h>
#include "inet/common/INETUtils.h"
#include "inet/routing/prels/Prels_repositories.h"

namespace inet {

#ifndef CURRENT_TIME
#define CURRENT_TIME    SIMTIME_DBL(simTime())
#endif

#ifndef CURRENT_TIME_T
#define CURRENT_TIME_T  SIMTIME_DBL(simTime())
#endif

#ifndef DELAY(time)
#define DELAY(time) (((time) < (CURRENT_TIME)) ? (0.000001) : \
    (time - CURRENT_TIME + 0.000001))
#endif

#ifndef DELAY_T(time)
#define DELAY_T(time) (((time) < (CURRENT_TIME_T)) ? (0.000001) : \
    (time - CURRENT_TIME_T + 0.000001))
#endif


class Prels;
class Prels_Timer :  public cOwnedObject {
public:
    Prels*  agent_ = nullptr; ///< agent which created the timer.
public:
  virtual void expire() = 0;
  virtual void removeQueueTimer();
  virtual void removeTimer();
  virtual void resched(double time);
  virtual void resched(simtime_t time);
  virtual bool isScheduled();

public:
  Prels_Timer(Prels* agent);
  Prels_Timer();
  virtual ~Prels_Timer();
};

typedef std::multimap <simtime_t, Prels_Timer*> TimerMultiMap;


class Prels_SendPktTimer : public Prels_Timer
{
public:
    Prels_SendPktTimer(Prels* agent) : Prels_Timer(agent) { agent_ = agent; }
    Prels_SendPktTimer() : Prels_Timer() {}
    virtual void expire() override;
};

class Prels_HelloTimer: public Prels_Timer
{
public:
    Prels_HelloTimer(Prels* agent) : Prels_Timer(agent) { agent_ = agent; }
    Prels_HelloTimer() : Prels_Timer() {}
    virtual void expire() override;
};

class Prels_TcTimer: public Prels_Timer
{
public:
    Prels_TcTimer(Prels* agent) : Prels_Timer(agent) { agent_ = agent; }
    Prels_TcTimer() : Prels_Timer() {}
    virtual void expire() override;
};

class Prels_NbTupleTimer : public Prels_Timer
{
public:
    Prels_nb_tuple*   tuple_ = nullptr;
  public:
    Prels_NbTupleTimer(Prels* agent, Prels_nb_tuple* tuple);

    void setTuple(Prels_nb_tuple* tuple) {tuple_ = tuple; tuple->asocTimer = this;}
    ~Prels_NbTupleTimer();
    virtual void expire() override;
};



} /* namespace inet */

#endif /* INET_ROUTING_PRELS_PRELSTIMER_H_ */
