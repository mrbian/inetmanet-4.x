
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

/**
* \ingroup sixlowpan
* \brief   Dispatch header helper. This class only purpose is to interpret
* the Dispatch header into its correct type.
*
* The dispatch type is defined by a zero bit as the first bit and a one
*  bit as the second bit.
  \verbatim
                        1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |0 1| Dispatch  |  type-specific header
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  \endverbatim
*/
#ifndef __SIXLOWPANDISPATCH_CODE_H__
#define __SIXLOWPANDISPATCH_CODE_H__

#include "inet/common/INETDefs.h"

namespace inet {

namespace sixlowpan {

class SixLowPanDispatchCode
{
public:
  /**
   *  \brief Dispatch values, as defined in \RFC{4944} and \RFC{6282}
  \verbatim
     Pattern    Header Type
   +------------+------------------------------------------------+
   | 00  xxxxxx | NALP        - Not a LoWPAN frame               |
   | 01  000000 | ESC         - Additional Dispatch byte follows |
   | 01  000001 | IPv6        - Uncompressed IPv6 Addresses      |
   | 01  000010 | LOWPAN_HC1  - LOWPAN_HC1 compressed IPv6       |
   | 01  000011 | reserved    - Reserved for future use          |
   |   ...      | reserved    - Reserved for future use          |
   | 01  001111 | reserved    - Reserved for future use          |
   | 01  010000 | LOWPAN_BC0  - LOWPAN_BC0 broadcast             |
   | 01  010001 | reserved    - Reserved for future use          |
   |   ...      | reserved    - Reserved for future use          |
   | 01  1xxxxx | LOWPAN_IPHC - LOWPAN_IPHC compressed IPv6      |
   | 10  xxxxxx | MESH        - Mesh Header                      |
   | 11  000xxx | FRAG1       - Fragmentation Header (first)     |
   | 11  001000 | reserved    - Reserved for future use          |
   |   ...      | reserved    - Reserved for future use          |
   | 11  011111 | reserved    - Reserved for future use          |
   | 11  100xxx | FRAGN       - Fragmentation Header (subsequent)|
   | 11  101000 | reserved    - Reserved for future use          |
   |   ...      | reserved    - Reserved for future use          |
   | 11  111111 | reserved    - Reserved for future use          |
   +------------+------------------------------------------------+
   \endverbatim
   */
  enum Dispatch_e
  {
    LOWPAN_NALP = 0x0,
    LOWPAN_NALP_N = 0x3F,
    LOWPAN_IPv6 = 0x41,
    LOWPAN_HC1 = 0x42,
    LOWPAN_BC0 = 0x50,
    LOWPAN_IPHC = 0x60,
    LOWPAN_IPHC_N = 0x7F,
    LOWPAN_MESH = 0x80,
    LOWPAN_MESH_N = 0xBF,
    LOWPAN_FRAG1 = 0xC0,
    LOWPAN_FRAG1_N = 0xC7,
    LOWPAN_FRAGN = 0xE0,
    LOWPAN_FRAGN_N = 0xE7,
    LOWPAN_UNSUPPORTED = 0xFF
  };

  /**
   *  \brief Dispatch values for Next Header compression.
   *
   *  The dispatch values reflect the dispatch use, since
   *  some dispatch bits carry actual header compression bits.
   */
  enum NhcDispatch_e
  {
    LOWPAN_NHC = 0xE0,
    LOWPAN_NHC_N = 0xEF,
    LOWPAN_UDPNHC = 0xF0,
    LOWPAN_UDPNHC_N = 0xF7,
    LOWPAN_NHCUNSUPPORTED = 0xFF
  };

  SixLowPanDispatchCode (void);

  /**
   * \brief Get the Dispatch type.
   * \param [in] dispatch The dispatch value.
   * \return The Dispatch type.
   */
  static Dispatch_e GetDispatchType (uint8_t dispatch);

  /**
   * \brief Get the NhcDispatch type.
   * \param [in] dispatch The dispatch value.
   * \return The NhcDispatch type.
   */
  static NhcDispatch_e GetNhcDispatchType (uint8_t dispatch);

};




}
}

#endif
