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

#ifndef TROPIC_MULTIGOALCONSTRAINT_H
#define TROPIC_MULTIGOALCONSTRAINT_H

#include "ConstraintSet.h"



namespace tropic
{

/*! \brief Holds a set of goal constraint (flag 7) at time t corresponding to
 *         the given vector pos. Vector pos is expected to have the dimension
 *         of the positions, not the trajectorie's internal coordinates. For
 *         instance a set of Euler angles are three elements. The conversion
 *         to the trajectorie's internal coordinates is done inside this class.
 */
class MultiGoalConstraint : public ConstraintSet
{
public:
  MultiGoalConstraint(double t, const MatNd* pos,
                      std::vector<TrajectoryND*> traj);

  virtual ~MultiGoalConstraint();

  bool check(const MatNd* pos, std::vector<TrajectoryND*> traj);
};

}   // namespace tropic





#endif   // TROPIC_MULTIGOALCONSTRAINT_H
