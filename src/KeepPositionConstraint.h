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

#ifndef TROPIC_KEEPPOSITIONCONSTRAINT_H
#define TROPIC_KEEPPOSITIONCONSTRAINT_H

#include "ConstraintSet.h"

#include <string>



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for keeping the position of a trajectory constan
 */
class KeepPositionConstraint : public ConstraintSet
{
public:

  KeepPositionConstraint();
  KeepPositionConstraint(xmlNode* node);
  KeepPositionConstraint(double t0, double t1, const std::string& trajNameND);
  KeepPositionConstraint(double t0, double t1, double horizon,
                         const std::string& trajNameND);
  virtual KeepPositionConstraint* clone() const;
  virtual ~KeepPositionConstraint();
  virtual void fromXML(xmlNode* node);
  virtual void toXML(std::ostream& out, size_t indent = 0) const;

  virtual double compute(double dt);

  virtual bool inUse() const;

  virtual void apply(std::vector<TrajectoryND*>& trajectory,
                     std::map<std::string, Trajectory1D*>& tMap,
                     bool permissive);

protected:

  double startTime;
  double endTime;
  double horizon;
  bool active;
  TrajectoryND* trajND;
  std::string trjName;

};

}   // namespace tropic


#endif   // TROPIC_KEEPPOSITIONCONSTRAINT_H
