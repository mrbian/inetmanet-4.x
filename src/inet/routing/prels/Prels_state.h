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

#ifndef INET_ROUTING_PRELS_PRELSSTATE_H_
#define INET_ROUTING_PRELS_PRELSSTATE_H_

#include "inet/common/INETDefs.h"
#include "inet/routing/prels/Prels_repositories.h"

namespace inet {

class Prels_state : public cObject
{
public:
    nbset_t     nbset_;     ///< Neighbor Set (RFC 3626, section 4.3.1).
    nb2hopset_t nb2hopset_; ///< 2-hop Neighbor Set (RFC 3626, section 4.3.2).
    mprset_t    mprset_;    ///< MPR Set (RFC 3626, section 4.3.3).
    mprselset_t mprselset_; ///< MPR Selector Set (RFC 3626, section 4.3.4).
    nodeset_t  nodeset_;
    dupset_t    dupset_;    ///< Duplicate Set (RFC 3626, section 3.4).

    inline  mprset_t&       mprset()    { return mprset_; }
    inline  mprselset_t&        mprselset() { return mprselset_; }
    inline  nbset_t&        nbset()     { return nbset_; }
    inline  nb2hopset_t&        nb2hopset() { return nb2hopset_; }
    inline  nodeset_t&      nodeset() { return nodeset_; }
    inline  dupset_t&       dupset()    { return dupset_; }

    Prels_mprsel_tuple*  find_mprsel_tuple(const nsaddr_t &);
    void            erase_mprsel_tuple(Prels_mprsel_tuple*);
    bool            erase_mprsel_tuples(const nsaddr_t &);
    void            insert_mprsel_tuple(Prels_mprsel_tuple*);

    Prels_nb_tuple*      find_nb_tuple(const nsaddr_t &);
    Prels_nb_tuple*      find_sym_nb_tuple(const nsaddr_t &);
    Prels_nb_tuple*      find_nb_tuple(const nsaddr_t &, uint8_t);
    void            erase_nb_tuple(Prels_nb_tuple*);
    void            erase_nb_tuple(const nsaddr_t &);
    void            insert_nb_tuple(Prels_nb_tuple*);

    Prels_nb2hop_tuple*  find_nb2hop_tuple(const nsaddr_t &, const nsaddr_t &);
    void            erase_nb2hop_tuple(Prels_nb2hop_tuple*);
    bool            erase_nb2hop_tuples(const nsaddr_t &);
    bool            erase_nb2hop_tuples(const nsaddr_t &, const nsaddr_t &);
    void            insert_nb2hop_tuple(Prels_nb2hop_tuple*);

    bool            find_mpr_addr(const nsaddr_t&);
    void            insert_mpr_addr(const nsaddr_t&);
    void            clear_mprset();

    Prels_dup_tuple*     find_dup_tuple(const nsaddr_t &, uint16_t);
    void            erase_dup_tuple(Prels_dup_tuple*);
    void            insert_dup_tuple(Prels_dup_tuple*);

    Prels_node_tuple*     find_node_tuple(const nsaddr_t &);
    void            erase_node_tuple(Prels_node_tuple*);
    void            insert_node_tuple(Prels_node_tuple*);

    void            clear_all();

public:
    Prels_state();
    virtual ~Prels_state();
    Prels_state(const Prels_state &);
    virtual Prels_state * dup() const { return new Prels_state(*this); }
};

} /* namespace inet */

#endif /* INET_ROUTING_PRELS_PRELSSTATE_H_ */
