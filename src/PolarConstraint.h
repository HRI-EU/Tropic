/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef TROPIC_POLARCONSTRAINT_H
#define TROPIC_POLARCONSTRAINT_H

#include "ConstraintSet.h"


namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for computing 2d orientation trajectories
 *
 *         The PolarConstraint class is a 3-dimensinal constraint to create
 *         trajectories between Polar axes. Internally, it maintains three
 *         constraints that represent a Polar axis. Interpolation between
 *         two rotations is performed with a linear interpolation (LERP) of
 *         the axis components. To extract the interpolated rotations,
 *         the LERPed values are converted to the Polar angles. Therefore the
 *         interface to the class is the only place where Polar angles are used.
 */
class PolarConstraint : public ConstraintSet
{
public:

  PolarConstraint();
  PolarConstraint(xmlNode* node);
  PolarConstraint(double t, double polarPhi, double polarTheta,
                  const std::string& trajNameND, int flag=7);
  virtual ~PolarConstraint();
  virtual PolarConstraint* clone() const;
  virtual void fromXML(xmlNode* node);
  virtual void toXML(std::ostream& out, size_t indent = 0) const;
};

}   // namespace tropic

#endif   // TROPIC_POLARCONSTRAINT_H
