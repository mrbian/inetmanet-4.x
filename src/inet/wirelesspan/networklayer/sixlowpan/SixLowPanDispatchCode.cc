// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Copyright (c) 2013 Universita' di Firenze, Italy
// Author: Tommaso Pecorella <tommaso.pecorella@unifi.it>
//         Michele Muccio <michelemuccio@virgilio.it>
// Copyright (c) 2022 Universidad de Malaga, Spain
// Author: Alfonso Ariza aarizaq@uma.es
//

#include "inet/wirelesspan/networklayer/sixlowpan/SixLowPanDispatchCode.h"
namespace inet {
namespace wirelesspan {
namespace sixlowpan {

SixLowPanDispatchCode::SixLowPanDispatchCode ()
{
}

SixLowPanDispatchCode::Dispatch_e
SixLowPanDispatchCode::GetDispatchType (uint8_t dispatch)
{
  if (dispatch <= LOWPAN_NALP_N)
    {
      return LOWPAN_NALP;
    }
  else if (dispatch == LOWPAN_IPv6)
    {
      return LOWPAN_IPv6;
    }
  else if (dispatch == LOWPAN_HC1)
    {
      return LOWPAN_HC1;
    }
  else if (dispatch == LOWPAN_BC0)
    {
      return LOWPAN_BC0;
    }
  else if ((dispatch >= LOWPAN_IPHC) && (dispatch <= LOWPAN_IPHC_N))
    {
      return LOWPAN_IPHC;
    }
  else if ((dispatch >= LOWPAN_MESH) && (dispatch <= LOWPAN_MESH_N))
    {
      return LOWPAN_MESH;
    }
  else if ((dispatch >= LOWPAN_FRAG1) && (dispatch <= LOWPAN_FRAG1_N))
    {
      return LOWPAN_FRAG1;
    }
  else if ((dispatch >= LOWPAN_FRAGN) && (dispatch <= LOWPAN_FRAGN_N))
    {
      return LOWPAN_FRAGN;
    }
  return LOWPAN_UNSUPPORTED;
}

SixLowPanDispatchCode::NhcDispatch_e
SixLowPanDispatchCode::GetNhcDispatchType (uint8_t dispatch)
{
  if ((dispatch >= LOWPAN_NHC) && (dispatch <= LOWPAN_NHC_N))
    {
      return LOWPAN_NHC;
    }
  else if ((dispatch >= LOWPAN_UDPNHC) && (dispatch <= LOWPAN_UDPNHC_N))
    {
      return LOWPAN_UDPNHC;
    }
  return LOWPAN_NHCUNSUPPORTED;
}


}
}
}
