#ifndef __OLSRMsg_h__
#define __OLSRMsg_h__

#include <string.h>
#include "inet/common/INETDefs.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/routing/extras/olsr/Olrs_Etx_parameter.h"
#include <cmath>

namespace inet {

namespace inetmanet {


#ifndef nsaddr_t
typedef L3Address nsaddr_t;
#endif

/********** Message types **********/
/// %OLSR HELLO message type.
#define OLSR_HELLO_MSG      1
/// %OLSR TC message type.
#define OLSR_TC_MSG     2
/// %OLSR MID message type.
#define OLSR_MID_MSG        3


/// %OLSR HELLO message type.
#define OLSR_ETX_HELLO_MSG  OLSR_HELLO_MSG
/// %OLSR TC message type.
#define OLSR_ETX_TC_MSG     OLSR_TC_MSG
/// %OLSR MID message type.
#define OLSR_ETX_MID_MSG    OLSR_MID_MSG

/********** Packets stuff **********/

///
/// \brief Size of the addresses we are using.
///
/// You should always use this macro when interested in calculate addresses'
/// sizes. By default IPv4 addresses are supposed, but you can change to
/// IPv6 ones by recompiling with OLSR_IPv6 constant defined.
///

#ifdef OLSR_IPv6
#define ADDR_SIZE_DEFAULT   16
#else
#define ADDR_SIZE_DEFAULT   4
#endif

class OlsrAddressSize
{
    public:
      static uint32_t ADDR_SIZE;
};

/// Maximum number of messages per packet.
#define OLSR_MAX_MSGS       4
#define OLSR_ETX_MAX_MSGS OLSR_MAX_MSGS
/// Maximum number of hellos per message (4 possible link types * 3 possible nb types).
#define OLSR_MAX_HELLOS     12
#define OLSR_ETX_MAX_HELLOS OLSR_MAX_HELLOS
/// Maximum number of addresses advertised on a message.
#define OLSR_MAX_ADDRS      64
#define OLSR_ETX_MAX_ADDRS OLSR_MAX_ADDRS

/// Size (in bytes) of packet header.
#define OLSR_PKT_HDR_SIZE   4
#define OLSR_ETX_PKT_HDR_SIZE OLSR_PKT_HDR_SIZE

/// Size (in bytes) of message header.
#define OLSR_MSG_HDR_SIZE   8 + OlsrAddressSize::ADDR_SIZE
#define OLSR_ETX_MSG_HDR_SIZE OLSR_MSG_HDR_SIZE

/// Size (in bytes) of hello header.
#define OLSR_HELLO_HDR_SIZE 4
#define OLSR_ETX_HELLO_HDR_SIZE OLSR_HELLO_HDR_SIZE

/// Size (in bytes) of hello_msg header.
#define OLSR_HELLO_MSG_HDR_SIZE 4
#define OLSR_ETX_HELLO_MSG_HDR_SIZE OLSR_HELLO_MSG_HDR_SIZE

/// Size (in bytes) of tc header.
#define OLSR_TC_HDR_SIZE    4
#define OLSR_ETX_TC_HDR_SIZE OLSR_TC_HDR_SIZE


typedef struct Olsr_Etx_iface_address {

  /// Interface Address
  nsaddr_t  iface_address_;

  /// Link quality extension
  double  link_quality_;
  double  nb_link_quality_;

  /// Link delay extension
  double  link_delay_;
  double  nb_link_delay_;
  int parameter_qos_;

  // Link expiration extension
  double let_factor_;

  inline nsaddr_t& iface_address() { return iface_address_; }

  /// Link quality extension
  inline double& link_quality() { return link_quality_; }
  inline double& nb_link_quality() { return nb_link_quality_; }

  inline double  etx()  {
    double mult = (double) (link_quality() * nb_link_quality());
    double etx = 1;
    switch (parameter_qos_) {
    case OLSR_ETX_BEHAVIOR_ETX:
      return (mult < 0.01) ? 100.0 : (double) 1.0 / (double) mult;
      break;

    case OLSR_ETX_BEHAVIOR_LET:
      return (mult < 0.01) ? 100.0 : (double) let_factor_ / (double) mult;
      break;

    case OLSR_ETX_BEHAVIOR_ML:
      return mult;
      break;

    case OLSR_ETX_BEHAVIOR_NONE:
    default:
      return 0.0;
      break;
    }
  }

  /// Link delay extension
  inline double& link_delay() { return link_delay_; }
  inline double& nb_link_delay() { return nb_link_delay_; }
  inline double& let_factor() { return let_factor_; }

} Olsr_Etx_iface_address;


/// Auxiliary struct which is part of the %OLSR_ETX HELLO message (struct OLSR_ETX_hello).
typedef struct Olsr_hello_msg {

  /// Link code.
  uint8_t  link_code_;
  /// Reserved.
  uint8_t  reserved_;
  /// Size of this link message.
  uint16_t  link_msg_size_;
  /// List of interface addresses of neighbor nodes.
  Olsr_Etx_iface_address  nb_iface_addrs_[OLSR_MAX_ADDRS];
  /// Number of interface addresses contained in nb_iface_addrs_.
  int    count;

  inline uint8_t&  link_code()    { return link_code_; }
  inline uint8_t&  reserved()    { return reserved_; }
  inline uint16_t&  link_msg_size()    { return link_msg_size_; }
  inline Olsr_Etx_iface_address&  nb_etx_iface_addr(int i)  { return nb_iface_addrs_[i]; }
  inline nsaddr_t  & nb_iface_addr(int i)  { return nb_iface_addrs_[i].iface_address_;}
  inline void set_qos_behaviour(int bh) {for(int i=0;i<OLSR_MAX_ADDRS;i++) nb_iface_addrs_[i].parameter_qos_=bh;}

  inline uint32_t size() { return OLSR_HELLO_MSG_HDR_SIZE + count*OlsrAddressSize::ADDR_SIZE; }

} OLSR_ETX_hello_msg;

#if 0
/// Auxiliary struct which is part of the %OLSR HELLO message (struct OLSR_hello).
typedef struct Olsr_hello_msg {

        /// Link code.
    uint8_t link_code_;
    /// Reserved.
    uint8_t reserved_;
    /// Size of this link message.
    uint16_t    link_msg_size_;
    /// List of interface addresses of neighbor nodes.
    nsaddr_t    nb_iface_addrs_[OLSR_MAX_ADDRS];
    /// Number of interface addresses contained in nb_iface_addrs_.
    int     count;

    inline uint8_t& link_code()     { return link_code_; }
    inline uint8_t& reserved()      { return reserved_; }
    inline uint16_t&    link_msg_size()     { return link_msg_size_; }
    inline nsaddr_t&    nb_iface_addr(int i)    { return nb_iface_addrs_[i]; }

    inline uint32_t size() { return OLSR_HELLO_MSG_HDR_SIZE + count*OlsrAddressSize::ADDR_SIZE; }

} Olsr_hello_msg;
#endif

/// %OLSR HELLO message.

typedef struct Olsr_hello :cObject {

    /// Reserved.
    uint16_t    reserved_;
    /// HELLO emission interval in mantissa/exponent format.
    uint8_t htime_;
    /// Willingness of a node for forwarding packets on behalf of other nodes.
    uint8_t willingness_;
    /// List of OLSR_hello_msg.
    Olsr_hello_msg  hello_body_[OLSR_MAX_HELLOS];
    /// Number of OLSR_hello_msg contained in hello_body_.
    int     count;
    int     node_mob_info_; // mobility index for position prediction(node id)

    inline uint16_t&    reserved()      { return reserved_; }
    inline uint8_t& htime()         { return htime_; }
    inline uint8_t& willingness()       { return willingness_; }
    inline Olsr_hello_msg&  hello_msg(int i)    { return hello_body_[i]; }

    inline uint32_t size() {
        uint32_t sz = OLSR_HELLO_HDR_SIZE;
        for (int i = 0; i < count; i++)
            sz += hello_msg(i).size();
        return sz;
    }
} Olsr_hello;


/// %OLSR TC message.
typedef struct Olsr_tc :cObject{

        /// Advertised Neighbor Sequence Number.
    uint16_t    ansn_;
    /// Reserved.
    uint16_t    reserved_;
    /// List of neighbors' main addresses.
    Olsr_Etx_iface_address  nb_main_addrs_[OLSR_MAX_ADDRS];
    /// Number of neighbors' main addresses contained in nb_main_addrs_.
    int     count;

    inline  uint16_t&   ansn()          { return ansn_; }
    inline  uint16_t&   reserved()      { return reserved_; }
    inline  nsaddr_t&   nb_main_addr(int i) { return nb_main_addrs_[i].iface_address_; }
    inline  Olsr_Etx_iface_address& nb_etx_main_addr(int i) { return nb_main_addrs_[i]; }
    inline void set_qos_behaviour(int bh) {for(int i=0;i<OLSR_MAX_ADDRS;i++) nb_main_addrs_[i].parameter_qos_=bh;}
    inline  uint32_t size() { return OLSR_TC_HDR_SIZE + count*OlsrAddressSize::ADDR_SIZE; }
} Olsr_tc;

#if 0
/// %OLSR TC message.
typedef struct Olsr_tc :cObject{

        /// Advertised Neighbor Sequence Number.
    uint16_t    ansn_;
    /// Reserved.
    uint16_t    reserved_;
    /// List of neighbors' main addresses.
    nsaddr_t    nb_main_addrs_[OLSR_MAX_ADDRS];
    /// Number of neighbors' main addresses contained in nb_main_addrs_.
    int     count;

    inline  uint16_t&   ansn()          { return ansn_; }
    inline  uint16_t&   reserved()      { return reserved_; }
    inline  nsaddr_t&   nb_main_addr(int i) { return nb_main_addrs_[i]; }

    inline  uint32_t size() { return OLSR_TC_HDR_SIZE + count*OlsrAddressSize::ADDR_SIZE; }

} Olsr_tc;
#endif

/// %OLSR MID message.
typedef struct Olsr_mid :cObject{

    /// List of interface addresses.
    nsaddr_t    iface_addrs_[OLSR_MAX_ADDRS];
    /// Number of interface addresses contained in iface_addrs_.
    int     count;

    inline nsaddr_t & iface_addr(int i) { return iface_addrs_[i]; }
    void    setIface_addr(int i,nsaddr_t a) {iface_addrs_[i]=a; }

    inline uint32_t size()          { return count*OlsrAddressSize::ADDR_SIZE; }

} Olsr_mid;

#define MidSize sizeof (OLSR_mid)
#define HelloSize sizeof (OLSR_hello)
#define TcSize sizeof(OLSR_tc)

#define MAXBODY (HelloSize > TcSize ?  (HelloSize > MidSize? HelloSize: MidSize):(TcSize > MidSize? TcSize:MidSize))


class   MsgBody {
public:
    Olsr_hello  hello_;
    Olsr_tc     tc_;
    Olsr_mid    mid_;


    MsgBody(MsgBody &other){
            memcpy((void*)&hello_,(void*)&other.hello_,sizeof(Olsr_hello));
            memcpy((void*)&tc_,(void*)&other.tc_,sizeof(Olsr_tc));
            memcpy((void*)&mid_,(void*)&other.mid_,sizeof(Olsr_mid));
        }
    MsgBody(){
            memset((void*)&hello_,0,sizeof(Olsr_hello));
            memset((void*)&tc_,0,sizeof(Olsr_tc));
            memset((void*)&mid_,0,sizeof(Olsr_mid));
          }
    ~MsgBody(){}
    MsgBody & operator =  (const MsgBody &other){
            if (this==&other) return *this;
                            memcpy((void*)&hello_,(void*)&other.hello_,sizeof(Olsr_hello));
                            memcpy((void*)&tc_,(void*)&other.tc_,sizeof(Olsr_tc));
                            memcpy((void*)&mid_,(void*)&other.mid_,sizeof(Olsr_mid));
                return *this;
            }
    Olsr_hello  * hello(){return &hello_;}
    Olsr_tc * tc(){return &tc_;}
    Olsr_mid* mid(){return &mid_;}


};

/// %OLSR message.
class OlsrMsg {
public:
    uint8_t msg_type_;  ///< Message type.
    uint8_t vtime_;     ///< Validity time.
    uint16_t    msg_size_;  ///< Message size (in bytes).
    nsaddr_t    orig_addr_; ///< Main address of the node which generated this message.
    uint8_t ttl_;       ///< Time to live (in hops).
    uint8_t hop_count_; ///< Number of hops which the message has taken.
    uint16_t    msg_seq_num_;   ///< Message sequence number.

    MsgBody msg_body_;          ///< Message body.
    inline  uint8_t&    msg_type()  { return msg_type_; }
    inline  uint8_t&    vtime()     { return vtime_; }
    inline  uint16_t&   msg_size()  { return msg_size_; }
    inline  nsaddr_t    & orig_addr()   { return orig_addr_; }
    inline  void    setOrig_addr(nsaddr_t a)    {orig_addr_=a; }
    inline  uint8_t&    ttl()       { return ttl_; }
    inline  uint8_t&    hop_count() { return hop_count_; }
    inline  uint16_t&   msg_seq_num()   { return msg_seq_num_; }
    inline  Olsr_hello& hello()     { return *(msg_body_.hello()); }
    inline  Olsr_tc&    tc()        { return *(msg_body_.tc()); }
    inline  Olsr_mid&   mid()       { return *(msg_body_.mid()); }

    std::string str() const {
        std::stringstream out;
        out << "type :"<< msg_type_ << " vtime :" << vtime_ << "orig_addr :" << orig_addr_;
        return out.str();
    }


    inline uint32_t size() {
        uint32_t sz = OLSR_MSG_HDR_SIZE;
        if (msg_type() == OLSR_HELLO_MSG)
            sz += hello().size();
        else if (msg_type() == OLSR_TC_MSG)
            sz += tc().size();
        else if (msg_type() == OLSR_MID_MSG)
            sz += mid().size();
        return sz;
    }
    OlsrMsg(){}
    OlsrMsg(const OlsrMsg &other)
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

    OlsrMsg & operator = (const OlsrMsg &other)
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
}

#endif
