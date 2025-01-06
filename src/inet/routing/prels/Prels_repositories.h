/*
 * Prels_repositories.h
 *
 *  Created on: Dec 27, 2024
 *      Author: bman
 */

#ifndef INET_ROUTING_PRELS_PRELS_REPOSITORIES_H_
#define INET_ROUTING_PRELS_PRELS_REPOSITORIES_H_

#include <string.h>
#include <set>
#include <vector>
#include "inet/common/INETDefs.h"
#include "inet/networklayer/common/L3Address.h"

namespace inet{

typedef L3Address nsaddr_t;

/// A Neighbor Tuple.
typedef struct Prels_nb_tuple : public cObject
{
    /// Main address of a neighbor node.
    nsaddr_t nb_main_addr_;
    /// Neighbor Type and Link Type at the four less significative digits.
    uint8_t status_;
    /// A value between 0 and 7 specifying the node's willingness to carry traffic on behalf of other nodes.
    uint8_t willingness_;
    //cMessage *asocTimer;
    cObject *asocTimer;
    // nb type
    uint8_t nb_type;
    // time when this tuple is lost
    double lost_time_;

    inline nsaddr_t & nb_main_addr()    { return nb_main_addr_; }
    void    setNb_main_addr(const nsaddr_t &a)  { nb_main_addr_ = a; }
    inline double & lost_time() { return lost_time_; }

    inline uint8_t& getStatus() { return status_; }
    inline uint8_t& willingness()   { return willingness_; }

    Prels_nb_tuple() {asocTimer = nullptr;}
    Prels_nb_tuple(const Prels_nb_tuple * e)
    {
        memcpy((void*)this, (void*)e, sizeof(Prels_nb_tuple));
        asocTimer = nullptr;
    }
    virtual Prels_nb_tuple *dup() const {return new Prels_nb_tuple(this);}
    ~Prels_nb_tuple()
    {
        status_ = 0;
    }
} Prels_nb_tuple;

/// A 2-hop Tuple.
typedef struct Prels_nb2hop_tuple : public cObject
{
    /// Main address of a neighbor.
    nsaddr_t    nb_main_addr_;
    /// Main address of a 2-hop neighbor with a symmetric link to nb_main_addr.
    nsaddr_t    nb2hop_addr_;
    /// Time at which this tuple expires and must be removed.
    double      time_;
    //cMessage *asocTimer;
    cObject *asocTimer;

    inline nsaddr_t & nb_main_addr()    { return nb_main_addr_; }
    inline nsaddr_t & nb2hop_addr() { return nb2hop_addr_; }
    void    setNb_main_addr(const nsaddr_t &a)  { nb_main_addr_ = a; }
    void    setNb2hop_addr(const nsaddr_t &a)   { nb2hop_addr_ = a; }

    inline double&      time()      { return time_; }

    Prels_nb2hop_tuple() {asocTimer = nullptr;}
    Prels_nb2hop_tuple(const Prels_nb2hop_tuple * e)
    {
        memcpy((void*)this, (void*)e, sizeof(Prels_nb2hop_tuple));
        asocTimer = nullptr;
    }
    virtual Prels_nb2hop_tuple *dup() const {return new Prels_nb2hop_tuple(this);}

} Prels_nb2hop_tuple;

/// An MPR-Selector Tuple.
typedef struct Prels_mprsel_tuple : public cObject
{
    /// Main address of a node which have selected this node as a MPR.
    nsaddr_t    main_addr_;
    /// Time at which this tuple expires and must be removed.
    double      time_;
    // cMessage *asocTimer;
    cObject *asocTimer;

    inline nsaddr_t & main_addr()   { return main_addr_; }
    void    setMain_addr(const nsaddr_t &a) {main_addr_ = a; }
    inline double&      time()      { return time_; }

    Prels_mprsel_tuple() {asocTimer = nullptr;}
    Prels_mprsel_tuple(const Prels_mprsel_tuple * e)
    {
        memcpy((void*)this, (void*)e, sizeof(Prels_mprsel_tuple));
        asocTimer = nullptr;
    }
    virtual Prels_mprsel_tuple *dup() const {return new Prels_mprsel_tuple(this);}


} Prels_mprsel_tuple;

/// An Multi-Hop Node Tuple.
typedef struct Prels_node_tuple : public cObject
{
    /// Main address of a node which have selected this node as a MPR.
    nsaddr_t    main_addr_;
    /// Mob info
    int mob_info_idx_;
    double mob_update_time_;
    /// Time at which this tuple expires and must be removed.
    double      time_;
    // cMessage *asocTimer;
    cObject *asocTimer;

    inline nsaddr_t & main_addr()   { return main_addr_; }
    void    setMain_addr(const nsaddr_t &a) {main_addr_ = a; }
    inline double&      time()      { return time_; }
    inline double&     mob_update_time() { return mob_update_time_;}
    inline int&        mob_info()   { return mob_info_idx_; }

    Prels_node_tuple() {asocTimer = nullptr;}
    Prels_node_tuple(const Prels_node_tuple * e)
    {
        memcpy((void*)this, (void*)e, sizeof(Prels_node_tuple));
        asocTimer = nullptr;
    }
    virtual Prels_node_tuple *dup() const {return new Prels_node_tuple(this);}

} Prels_node_tuple;



/// The type "list of interface addresses"
typedef std::vector<nsaddr_t> addr_list_t;

/// A Duplicate Tuple
typedef struct Prels_dup_tuple : public cObject
{
    /// Originator address of the message.
    nsaddr_t    addr_;
    /// Message sequence number.
    uint16_t    seq_num_;
    /// Indicates whether the message has been retransmitted or not.
    bool        retransmitted_;
    /// List of interfaces which the message has been received on.
    addr_list_t iface_list_;
    /// Time at which this tuple expires and must be removed.
    double      time_;
    // cMessage *asocTimer;
    cObject *asocTimer;

    inline nsaddr_t & getAddr()     { return addr_; }
    void    setAddr(const nsaddr_t &a)  {addr_ = a; }

    inline uint16_t&    seq_num()   { return seq_num_; }
    inline bool&        retransmitted() { return retransmitted_; }
    inline addr_list_t& iface_list()    { return iface_list_; }
    inline double&      time()      { return time_; }

    Prels_dup_tuple() {asocTimer = nullptr;}
    Prels_dup_tuple(const Prels_dup_tuple * e)
    {
        memcpy((void*)this, (void*)e, sizeof(Prels_dup_tuple));
        asocTimer = nullptr;
    }
    virtual Prels_dup_tuple *dup() const {return new Prels_dup_tuple(this);}

} Prels_dup_tuple;

typedef std::set<nsaddr_t>          mprset_t;   ///< MPR Set type.
typedef std::vector<Prels_mprsel_tuple*>     mprselset_t;    ///< MPR Selector Set type.
typedef std::vector<Prels_nb_tuple*>     nbset_t;    ///< Neighbor Set type.
typedef std::vector<Prels_nb2hop_tuple*>     nb2hopset_t;    ///< 2-hop Neighbor Set type.
typedef std::vector<Prels_node_tuple*> nodeset_t;
typedef std::vector<Prels_dup_tuple*>        dupset_t;   ///< Duplicate Set type.

}


#endif /* INET_ROUTING_PRELS_PRELS_REPOSITORIES_H_ */
