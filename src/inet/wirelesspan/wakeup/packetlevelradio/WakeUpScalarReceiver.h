//
// Copyright (C) 2014 Florian Meier
// Copyright (C) 2013 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_WAKEUPSCALARRECEIVER_H
#define __INET_WAKEUPSCALARRECEIVER_H

#include "inet/wirelesspan/wakeup/packetlevelradio/WakeUpReceiverBase.h"
namespace inet {

namespace physicallayer {

class INET_API WakeUpScalarReceiver : public WakeUpReceiverBase
{
  protected:
    W minInterferencePower;

  public:
    WakeUpScalarReceiver();

    void initialize(int stage) override;


    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags = 0) const override;

    virtual W getMinInterferencePower() const override { return minInterferencePower; }
};

} // namespace physicallayer

} // namespace inet

#endif

