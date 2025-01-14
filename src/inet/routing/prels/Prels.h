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

#ifndef INET_ROUTING_PRELS_PRELS_H_
#define INET_ROUTING_PRELS_PRELS_H_

#include <map>
#include <vector>
#include <assert.h>
#include "inet/common/INETUtils.h"
#include "inet/routing/prels/Prels_repositories.h"
#include "inet/routing/prels/Prels_state.h"
#include "inet/routing/prels/PrelsPkt_m.h"
#include "inet/routing/prels/Prels_hdr_m.h"
#include "inet/routing/prels/Prels_timer.h"

#include "inet/networklayer/base/NetworkProtocolBase.h"
#include "inet/networklayer/contract/INetworkProtocol.h"
#include "inet/networklayer/ipv4/Ipv4InterfaceData.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/mobility/single/ExtendedBonnMotionMobility.h"
#include "inet/networklayer/contract/IArp.h"
#include "inet/networklayer/arp/ipv4/GlobalArp.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/common/packet/Packet.h"
#include "inet/node/base/NodeBase.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"


namespace inet {

#ifndef CURRENT_TIME
#define CURRENT_TIME    SIMTIME_DBL(simTime())
#endif

#define PRELS_MAXJITTER      hello_ival()/4
#define PRELS_MAX_SEQ_NUM    65535

/// Not neighbor type.
#define PRELS_NOT_NEIGH      0
/// Symmetric neighbor type.
#define PRELS_SYM_NEIGH      1
/// Asymmetric neighbor type.
#define PRELS_MPR_NEIGH      2

extern simsignal_t mobInfoAverLapseSignal;
extern simsignal_t mobInfoMaxLapseSignal;
extern simsignal_t mobInfoMinLapseSignal;
extern simsignal_t mobInfoCountSignal;


class Prels: public NetworkProtocolBase, public INetworkProtocol
{
    friend class Prels_Timer;
    friend class Prels_NbTupleTimer;
    friend class Prels_HelloTimer;
    friend class Prels_TcTimer;
    friend class Prels_SendPktTimer;
    friend class Prels_NbTupleTimer;

public:
    Prels() {}
    virtual ~Prels();

protected:
    // Pointer
    INetfilter *networkProtocol = nullptr;
    NetworkInterface *interface80211ptr = nullptr;
    IInterfaceTable *ift = nullptr;
    IMobility *mobility = nullptr;
    GlobalArp* arp = nullptr;
    std::map<int, IMobility*> _globalMob;
    int interfaceId = -1;
    int self_node_id;
    Ipv4Address m_selfIpv4Address;

  double jitter() {return uniform(0,(double)PRELS_MAXJITTER);}
#define JITTER jitter()

public:
  Prels_state    *state_ptr = nullptr;
  bool configured = false;
  virtual TimerMultiMap *getTimerMultimMap() const {return timerMultiMapPtr;}
  TimerMultiMap timerMultiMap;
  TimerMultiMap *timerMultiMapPtr = &timerMultiMap;
  cMessage *timerMessagePtr = nullptr;
  void createTimerQueue();
  void scheduleEvent();
  bool checkTimer(cMessage *msg);

  Prels_HelloTimer *helloTimer = nullptr;    ///< Timer for sending HELLO messages.
  Prels_TcTimer    *tcTimer = nullptr;   ///< Timer for sending TC messages.

protected:
  // hello/tc queue for merging
  std::vector<PrelsMsg>   msgs_;

  /// Packets sequence number counter.
  uint16_t    pkt_seq_ = PRELS_MAX_SEQ_NUM;
  /// Messages sequence number counter.
  uint16_t    msg_seq_ = PRELS_MAX_SEQ_NUM;
  /// Advertised Neighbor Set sequence number.
  uint16_t    ansn_ = PRELS_MAX_SEQ_NUM;
  /// HELLO messages' emission interval.
  cPar     *hello_ival_ = nullptr;
  /// TC messages' emission interval.
  cPar     *tc_ival_ = nullptr;
  /// Increments packet sequence number and returns the new value.
  inline uint16_t pkt_seq()
  {
      pkt_seq_ = (pkt_seq_ + 1)%(PRELS_MAX_SEQ_NUM + 1);
      return pkt_seq_;
  }
  /// Increments message sequence number and returns the new value.
  inline uint16_t msg_seq()
  {
      msg_seq_ = (msg_seq_ + 1)%(PRELS_MAX_SEQ_NUM + 1);
      return msg_seq_;
  }
  inline double     hello_ival()    { return hello_ival_->doubleValue();}
  inline double     tc_ival()   { return tc_ival_->doubleValue();}

protected:
  inline mprset_t&    mprset()    { return state_ptr->mprset(); }
  inline mprselset_t& mprselset() { return state_ptr->mprselset(); }
  inline nbset_t&     nbset()     { return state_ptr->nbset(); }
  inline nb2hopset_t& nb2hopset() { return state_ptr->nb2hopset(); }
  inline dupset_t&    dupset()    { return state_ptr->dupset(); }

  virtual void        add_dup_tuple(Prels_dup_tuple*);
  virtual void        rm_dup_tuple(Prels_dup_tuple*);
  virtual void        add_nb_tuple(Prels_nb_tuple*);
  virtual void        rm_nb_tuple(Prels_nb_tuple*);
  virtual void        add_nb2hop_tuple(Prels_nb2hop_tuple*);
  virtual void        rm_nb2hop_tuple(Prels_nb2hop_tuple*);
  virtual void        add_mprsel_tuple(Prels_mprsel_tuple*);
  virtual void        rm_mprsel_tuple(Prels_mprsel_tuple*);
  virtual void        add_node_tuple(Prels_node_tuple*);
  virtual void        rm_node_tuple(Prels_node_tuple*);


protected:
  virtual void        recv_prels(Packet*);

  virtual void        CoverTwoHopNeighbors(const nsaddr_t &neighborMainAddr, nb2hopset_t & N2);
  virtual void        mpr_computation();
  virtual void        rtable_computation();

  virtual bool        process_hello(PrelsMsg&);
  virtual bool        process_tc(PrelsMsg&);

  virtual void        forward_default(PrelsMsg&, Prels_dup_tuple*, const nsaddr_t &);

  virtual void        enque_msg(PrelsMsg&, double);
  virtual void        send_hello();
  virtual void        send_tc();
  virtual void        send_pkt();

  virtual bool        populate_nbset(PrelsMsg&);
  virtual bool        populate_nb2hopset(PrelsMsg&);
  virtual void        populate_mprselset(PrelsMsg&);

  virtual int     degree(Prels_nb_tuple*);

  static bool seq_num_bigger_than(uint16_t, uint16_t);

  virtual void    recv(cMessage *p) {}

  L3Address getIfaceAddressFromIndex(int index);



protected:
  virtual int numInitStages() const override { return NUM_INIT_STAGES; }
  virtual void initialize(int stage) override;

  virtual void handleUpperPacket(Packet *packet) override;
  virtual void handleLowerPacket(Packet *packet) override;
  virtual void handleSelfMessage(cMessage *msg) override;

  void setDownControlInfo(Packet *const pMsg, const MacAddress& pDestAddr);

  const Protocol& getProtocol() const override { return Protocol::ipv4; }

  void start();
  void stop();
  void finish() override;
  virtual void handleStartOperation(LifecycleOperation *operation) override;  // application layer
  virtual void handleStopOperation(LifecycleOperation *operation) override { stop(); }
  virtual void handleCrashOperation(LifecycleOperation *operation) override { stop(); }

};

} /* namespace inet */

#endif /* INET_ROUTING_PRELS_PRELS_H_ */
