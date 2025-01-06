/*
 * Prels_Msg.h
 *
 *  Created on: Dec 27, 2024
 *      Author: bman
 */

#ifndef INET_ROUTING_PRELS_PRELS_MSG_H_
#define INET_ROUTING_PRELS_PRELS_MSG_H_


#include <string.h>
#include "inet/common/INETDefs.h"
#include "inet/networklayer/common/L3Address.h"
#include <cmath>

namespace inet {

#ifndef nsaddr_t
typedef L3Address nsaddr_t;
#endif

// Common
#define PRELS_MAX_MSGS       4
#define PRELS_MSG_HDR_SIZE 4
#define PRELS_MAX_ADDRS       64
#define PRELS_PKT_HDR_SIZE   4
/// %HELLO message
#define PRELS_HELLO_MSG      1
#define PRELS_MAX_HELLOS     12
#define PRELS_HELLO_HDR_SIZE 4
#define PRELS_HELLO_MSG_HDR_SIZE 4
/// %TC message
#define PRELS_TC_MSG     2
#define PRELS_TC_HDR_SIZE 4
#define PRELS_IFACE_ADDR_SIZE 16

typedef struct Prels_iface_address {

  /// Interface Address
  nsaddr_t  iface_address_;

  /// Node mobility info extension
  int mob_info_idx_;
  double mob_update_time_;

  inline nsaddr_t& iface_address() { return iface_address_; }
  /// mob extension
  inline int& mob_info() { return mob_info_idx_; }
  inline double& mob_update_time() { return mob_update_time_; }

} Prels_iface_address;

typedef struct Prels_hello_msg {

  /// Link code.
//  uint8_t  link_code_;
  uint8_t nbtype_code_;  // 按type分类，所以有不同的hello_msg
  /// Reserved.
  uint8_t  reserved_;
  /// Size of this link message.
  uint16_t  link_msg_size_;
  /// List of interface addresses of neighbor nodes.
  Prels_iface_address  nb_iface_addrs_[PRELS_MAX_ADDRS];
  /// Number of interface addresses contained in nb_iface_addrs_.
  int    count;

//  inline uint8_t&  link_code()    { return link_code_; }
  inline uint8_t&  nbtype_code()    { return nbtype_code_; }
  inline uint8_t&  reserved()    { return reserved_; }
  inline uint16_t&  link_msg_size()    { return link_msg_size_; }
  inline Prels_iface_address&  nb_etx_iface_addr(int i)  { return nb_iface_addrs_[i]; }
  inline nsaddr_t  & nb_iface_addr(int i)  { return nb_iface_addrs_[i].iface_address_;}

  inline uint32_t size() { return PRELS_HELLO_MSG_HDR_SIZE + count*PRELS_IFACE_ADDR_SIZE; }

} Prels_hello_msg;

typedef struct Prels_hello :cObject {

    /// Reserved.
    uint16_t    reserved_;
    /// HELLO emission interval in mantissa/exponent format.
    uint8_t htime_;
    /// Willingness of a node for forwarding packets on behalf of other nodes.
    uint8_t willingness_;
    /// List of hello_msg.
    Prels_hello_msg  hello_body_[PRELS_MAX_HELLOS];
    /// Number of hello_msg contained in hello_body_.
    int     count;
    /// Mobility extension
    int mob_info_;
    double mob_update_time_;

    inline uint16_t&    reserved()      { return reserved_; }
    inline uint8_t& htime()         { return htime_; }
    inline uint8_t& willingness()       { return willingness_; }
    inline int& mob_info()          { return mob_info_; }
    inline double& mob_update_time()  { return mob_update_time_;}

    inline Prels_hello_msg&  hello_msg(int i)    { return hello_body_[i]; }

    inline uint32_t size() {
        uint32_t sz = PRELS_HELLO_HDR_SIZE;
        for (int i = 0; i < count; i++)
            sz += hello_msg(i).size();
        return sz;
    }
} Prels_hello;


/// %TC message.
typedef struct Prels_tc :cObject{
    /// Advertised Neighbor Sequence Number.
    uint16_t    ansn_;
    /// Reserved.
    uint16_t    reserved_;
    /// List of neighbors' main addresses.
    Prels_iface_address  nb_main_addrs_[PRELS_MAX_ADDRS];
    /// Number of neighbors' main addresses contained in nb_main_addrs_.
    int     count;

    inline  uint16_t&   ansn()          { return ansn_; }
    inline  uint16_t&   reserved()      { return reserved_; }
    inline  nsaddr_t&   nb_main_addr(int i) { return nb_main_addrs_[i].iface_address_; }
    inline  Prels_iface_address& nb_etx_main_addr(int i) { return nb_main_addrs_[i]; }
    inline  uint32_t size() { return PRELS_TC_HDR_SIZE + count*PRELS_IFACE_ADDR_SIZE; }
} Prels_tc;


class   MsgBody {
public:
    Prels_hello  hello_;
    Prels_tc     tc_;

    MsgBody(MsgBody &other){
            memcpy((void*)&hello_,(void*)&other.hello_,sizeof(Prels_hello));
            memcpy((void*)&tc_,(void*)&other.tc_,sizeof(Prels_tc));
        }
    MsgBody(){
            memset((void*)&hello_,0,sizeof(Prels_hello));
            memset((void*)&tc_,0,sizeof(Prels_tc));
          }
    ~MsgBody(){}
    MsgBody & operator =  (const MsgBody &other){
            if (this==&other) return *this;
                            memcpy((void*)&hello_,(void*)&other.hello_,sizeof(Prels_hello));
                            memcpy((void*)&tc_,(void*)&other.tc_,sizeof(Prels_tc));
                return *this;
            }
    Prels_hello  * hello(){return &hello_;}
    Prels_tc * tc(){return &tc_;}
};


/// %message.
class PrelsMsg {
public:
    // 8 Byte
    uint8_t msg_type_;  ///< Message type.
    uint8_t vtime_;     ///< Validity time.
    uint16_t    msg_size_;  ///< Message size (in bytes).
    uint8_t ttl_;       ///< Time to live (in hops).
    uint8_t hop_count_; ///< Number of hops which the message has taken.
    uint16_t    msg_seq_num_;   ///< Message sequence number.

    nsaddr_t    orig_addr_; ///< Main address of the node which generated this message.

    MsgBody msg_body_;          ///< Message body.
    inline  uint8_t&    msg_type()  { return msg_type_; }
    inline  uint8_t&    vtime()     { return vtime_; }
    inline  uint16_t&   msg_size()  { return msg_size_; }
    inline  nsaddr_t    & orig_addr()   { return orig_addr_; }
    inline  void    setOrig_addr(nsaddr_t a)    {orig_addr_=a; }
    inline  uint8_t&    ttl()       { return ttl_; }
    inline  uint8_t&    hop_count() { return hop_count_; }
    inline  uint16_t&   msg_seq_num()   { return msg_seq_num_; }
    inline  Prels_hello& hello()     { return *(msg_body_.hello()); }
    inline  Prels_tc&    tc()        { return *(msg_body_.tc()); }

    std::string str() const {
        std::stringstream out;
        out << "type :"<< msg_type_ << " vtime :" << vtime_ << "orig_addr :" << orig_addr_;
        return out.str();
    }

    inline uint32_t size() {
        uint32_t sz = PRELS_MSG_HDR_SIZE;
        if (msg_type() == PRELS_HELLO_MSG)
            sz += hello().size();
        else if (msg_type() == PRELS_TC_MSG)
            sz += tc().size();
        return sz;
    }
    PrelsMsg(){}
    PrelsMsg(const PrelsMsg &other)
    {
        msg_type_=other.msg_type_;  ///< Message type.
        vtime_=other.vtime_;        ///< Validity time.
        msg_size_=other.msg_size_;  ///< Message size (in bytes).
        orig_addr_=other.orig_addr_;    ///< Main address of the node which generated this message.
        ttl_=other.ttl_;        ///< Time to live (in hops).
        hop_count_=other.hop_count_;    ///< Number of hops which the message has taken.
        msg_seq_num_=other.msg_seq_num_;    ///< Message sequence number.
        msg_body_=other.msg_body_;          ///< Message body.
    }

    PrelsMsg & operator = (const PrelsMsg &other)
    {
        if (this==&other) return *this;
        msg_type_=other.msg_type_;  ///< Message type.
        vtime_=other.vtime_;        ///< Validity time.
        msg_size_=other.msg_size_;  ///< Message size (in bytes).
        orig_addr_=other.orig_addr_;    ///< Main address of the node which generated this message.
        ttl_=other.ttl_;        ///< Time to live (in hops).
        hop_count_=other.hop_count_;    ///< Number of hops which the message has taken.
        msg_seq_num_=other.msg_seq_num_;    ///< Message sequence number.
        msg_body_=other.msg_body_;          ///< Message body.
        return *this;
    }
};

}


#endif /* INET_ROUTING_PRELS_PRELS_MSG_H_ */
