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

#include "Prels_state.h"

namespace inet {

/********** MPR Selector Set Manipulation **********/

Prels_mprsel_tuple*
Prels_state::find_mprsel_tuple(const nsaddr_t &main_addr)
{
    for (auto it = mprselset_.begin(); it != mprselset_.end(); it++)
    {
        Prels_mprsel_tuple* tuple = *it;
        if (tuple->main_addr() == main_addr)
            return tuple;
    }
    return nullptr;
}

void
Prels_state::erase_mprsel_tuple(Prels_mprsel_tuple* tuple)
{
    for (auto it = mprselset_.begin(); it != mprselset_.end(); it++)
    {
        if (*it == tuple)
        {
            mprselset_.erase(it);
            break;
        }
    }
}

bool
Prels_state::erase_mprsel_tuples(const nsaddr_t & main_addr)
{
    bool topologyChanged = false;
    for (auto it = mprselset_.begin(); it != mprselset_.end();)
    {
        Prels_mprsel_tuple* tuple = *it;
        if (tuple->main_addr() == main_addr)
        {
            it = mprselset_.erase(it);
            topologyChanged = true;
            if (mprselset_.empty())
                break;
        }
        else
            it++;

    }
    return topologyChanged;
}

void
Prels_state::insert_mprsel_tuple(Prels_mprsel_tuple* tuple)
{
    mprselset_.push_back(tuple);
}

/********** Neighbor Set Manipulation **********/

Prels_nb_tuple*
Prels_state::find_nb_tuple(const nsaddr_t & main_addr)
{
    for (auto it = nbset_.begin(); it != nbset_.end(); it++)
    {
        Prels_nb_tuple* tuple = *it;
        if (tuple->nb_main_addr() == main_addr)
            return tuple;
    }
    return nullptr;
}

Prels_nb_tuple*
Prels_state::find_nb_tuple(const nsaddr_t & main_addr, uint8_t willingness)
{
    for (auto it = nbset_.begin(); it != nbset_.end(); it++)
    {
        Prels_nb_tuple* tuple = *it;
        if (tuple->nb_main_addr() == main_addr && tuple->willingness() == willingness)
            return tuple;
    }
    return nullptr;
}

void
Prels_state::erase_nb_tuple(Prels_nb_tuple* tuple)
{
    for (auto it = nbset_.begin(); it != nbset_.end(); it++)
    {
        if (*it == tuple)
        {
            nbset_.erase(it);
            break;
        }
    }
}

void
Prels_state::erase_nb_tuple(const nsaddr_t & main_addr)
{
    for (auto it = nbset_.begin(); it != nbset_.end(); it++)
    {
        Prels_nb_tuple* tuple = *it;
        if (tuple->nb_main_addr() == main_addr)
        {
            it = nbset_.erase(it);
            break;
        }
    }
}

void
Prels_state::insert_nb_tuple(Prels_nb_tuple* tuple)
{
    nbset_.push_back(tuple);
}

/********** Neighbor 2 Hop Set Manipulation **********/

Prels_nb2hop_tuple*
Prels_state::find_nb2hop_tuple(const nsaddr_t & nb_main_addr, const nsaddr_t & nb2hop_addr)
{
    for (auto it = nb2hopset_.begin(); it != nb2hopset_.end(); it++)
    {
        Prels_nb2hop_tuple* tuple = *it;
        if (tuple->nb_main_addr() == nb_main_addr && tuple->nb2hop_addr() == nb2hop_addr)
            return tuple;
    }
    return nullptr;
}

void
Prels_state::erase_nb2hop_tuple(Prels_nb2hop_tuple* tuple)
{
    for (auto it = nb2hopset_.begin(); it != nb2hopset_.end(); it++)
    {
        if (*it == tuple)
        {
            nb2hopset_.erase(it);
            break;
        }
    }
}

bool
Prels_state::erase_nb2hop_tuples(const nsaddr_t & nb_main_addr, const nsaddr_t & nb2hop_addr)
{
    bool returnValue = false;
    for (auto it = nb2hopset_.begin(); it != nb2hopset_.end();)
    {
        Prels_nb2hop_tuple* tuple = *it;
        if (tuple->nb_main_addr() == nb_main_addr && tuple->nb2hop_addr() == nb2hop_addr)
        {
            it = nb2hopset_.erase(it);
            returnValue = true;
            if (nb2hopset_.empty())
                break;
        }
        else
            it++;

    }
    return returnValue;
}

bool
Prels_state::erase_nb2hop_tuples(const nsaddr_t & nb_main_addr)
{
    bool topologyChanged = false;
    for (auto it = nb2hopset_.begin(); it != nb2hopset_.end();)
    {
        Prels_nb2hop_tuple* tuple = *it;
        if (tuple->nb_main_addr() == nb_main_addr)
        {
            it = nb2hopset_.erase(it);
            topologyChanged = true;
            if (nb2hopset_.empty())
                break;
        }
        else
            it++;

    }
    return topologyChanged;
}

void
Prels_state::insert_nb2hop_tuple(Prels_nb2hop_tuple* tuple)
{
    nb2hopset_.push_back(tuple);
}

/********** MPR Set Manipulation **********/

bool
Prels_state::find_mpr_addr(const nsaddr_t & addr)
{
    auto it = mprset_.find(addr);
    return (it != mprset_.end());
}

void
Prels_state::insert_mpr_addr(const nsaddr_t & addr)
{
    mprset_.insert(addr);
}

void
Prels_state::clear_mprset()
{
    mprset_.clear();
}

/********** Duplicate Set Manipulation **********/

Prels_dup_tuple*
Prels_state::find_dup_tuple(const nsaddr_t & addr, uint16_t seq_num)
{
    for (auto it = dupset_.begin(); it != dupset_.end(); it++)
    {
        Prels_dup_tuple* tuple = *it;
        if (tuple->getAddr() == addr && tuple->seq_num() == seq_num)
            return tuple;
    }
    return nullptr;
}

void
Prels_state::erase_dup_tuple(Prels_dup_tuple* tuple)
{
    for (auto it = dupset_.begin(); it != dupset_.end(); it++)
    {
        if (*it == tuple)
        {
            dupset_.erase(it);
            break;
        }
    }
}

void
Prels_state::insert_dup_tuple(Prels_dup_tuple* tuple)
{
    dupset_.push_back(tuple);
}

/********** Node Set Manipulation **********/
Prels_node_tuple*
Prels_state::find_node_tuple(const nsaddr_t & addr)
{
    for (auto it = nodeset_.begin(); it != nodeset_.end(); it++)
    {
        Prels_node_tuple* tuple = *it;
        if (tuple->main_addr() == addr)
            return tuple;
    }
    return nullptr;
}

void
Prels_state::erase_node_tuple(Prels_node_tuple* tuple)
{
    for (auto it = nodeset_.begin(); it != nodeset_.end(); it++)
    {
        if (*it == tuple)
        {
            nodeset_.erase(it);
            break;
        }
    }
}

void
Prels_state::insert_node_tuple(Prels_node_tuple* tuple)
{
    nodeset_.push_back(tuple);
}



void Prels_state::clear_all()
{
    for (auto it = nbset_.begin(); it != nbset_.end(); it++)
        delete (*it);
    nbset_.clear();
    for (auto it = nb2hopset_.begin(); it != nb2hopset_.end(); it++)
        delete (*it);
    nb2hopset_.clear();

    for (auto it = mprselset_.begin(); it != mprselset_.end(); it++)
        delete (*it);
    mprselset_.clear();
    for (auto it = dupset_.begin(); it != dupset_.end(); it++)
        delete (*it);

    dupset_.clear();
    mprset_.clear();

}

Prels_state::Prels_state(const Prels_state& st)
{
    for (auto it = st.nbset_.begin(); it != st.nbset_.end(); it++)
    {
        Prels_nb_tuple* tuple = *it;
        nbset_.push_back(tuple->dup());
    }

    for (auto it = st.nb2hopset_.begin(); it != st.nb2hopset_.end(); it++)
    {
        Prels_nb2hop_tuple* tuple = *it;
        nb2hopset_.push_back(tuple->dup());
    }
    for (auto it = st.mprset_.begin(); it != st.mprset_.end(); it++)
    {
        mprset_.insert(*it);
    }

    for (auto it = st.mprselset_.begin(); it != st.mprselset_.end(); it++)
    {
        Prels_mprsel_tuple* tuple = *it;
        mprselset_.push_back(tuple->dup());
    }

    for (auto it = st.dupset_.begin(); it != st.dupset_.end(); it++)
    {
        Prels_dup_tuple* tuple = *it;
        dupset_.push_back(tuple->dup());
    }

}


Prels_state::Prels_state() {

}

Prels_state::~Prels_state() {
    clear_all();
}

} /* namespace inet */
