/***************************************************************************
* author:      Andreas Kuntz
*
* copyright:   (c) 2008 Institute of Telematics, University of Karlsruhe (TH)
*
* author:      Alfonso Ariza
*              Malaga university
*
*              This program is free software; you can redistribute it
*              and/or modify it under the terms of the GNU General Public
*              License as published by the Free Software Foundation; either
*              version 2 of the License, or (at your option) any later
*              version.
*              For further information see file COPYING
*              in the top level directory
***************************************************************************/

#ifndef __INET_EMPIRICNAKAGAMIFADING_H
#define __INET_EMPIRICNAKAGAMIFADING_H

#include "inet/physicallayer/wireless/common/pathloss/FreeSpacePathLoss.h"

namespace inet {

namespace physicallayer {

/**
 * This class implements the Nakagami fading model.
 */
class INET_API EmpiricNakagamiFading : public FreeSpacePathLoss
{
  protected:
    std::string pathlossFile;

  protected:
    virtual void initialize(int stage) override;

  public:
    EmpiricNakagamiFading();
    std::vector<double> values;
    virtual std::ostream& printToStream(std::ostream& stream, int level, int evFlags) const override;
    virtual double computePathLoss(mps propagationSpeed, Hz frequency, m distance) const override;
};

} // namespace physicallayer

} // namespace inet

#endif // ifndef __INET_EMPIRICNAKAGAMIFADING_H

