//
// Copyright (C) 2004 Andras Varga
// Copyright (C) 2000 Institut fuer Telematik, Universitaet Karlsruhe
// Copyright (C) 2011 Zoltan Bojthe
// Copyright (C) 2011 Alfonso Ariza, Universidad de Malaga
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//


#ifndef __INET_UDPBASICBURSTNOTIFICATION_H
#define __INET_UDPBASICBURSTNOTIFICATION_H

#include "inet/applications/udpapp/UdpBasicBurst.h"
#include "inet/applications/common/AddressModule.h"

namespace inet {

/**
 * UDP application. See NED for more info.
 */
class INET_API UdpBasicBurstNotification : public UdpBasicBurst, protected cListener
{
  protected:
    AddressModule * addressModule;
    int outputInterface = -1;
    std::vector<int> outputInterfaceMulticastBroadcast;

  protected:
    // chooses random destination address
    virtual L3Address chooseDestAddr() override;
    virtual void processStart() override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, intval_t i, cObject *details) override;
  public:
    UdpBasicBurstNotification();
    virtual ~UdpBasicBurstNotification();
};

}

#endif

