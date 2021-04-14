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

#include "MultiGoalConstraint.h"

#include <Rcs_macros.h>


namespace tropic
{

MultiGoalConstraint::MultiGoalConstraint(double t, const MatNd* pos,
                                         std::vector<TrajectoryND*> traj) :
  ConstraintSet()
{
  RCHECK(check(pos, traj));
  setClassName("MultiGoalConstraint");

  unsigned int arrayIdx = 0;

  for (size_t i=0; i<traj.size(); ++i)
  {
    double* internalCoords = new double[traj[i]->getInternalDim()];
    traj[i]->toInternalCoords(internalCoords, &pos->ele[arrayIdx]);

    // Special case for Quaternions: Check shortest path
    if (traj[i]->getClassName() == std::string("TrajectoryEuler"))
    {
      double qPrev[4];
      traj[i]->TrajectoryND::getPositionConstraintBefore(qPrev, t);
      double dotPrd = Quat_dot(internalCoords, qPrev);

      if (dotPrd<0.0)
      {
        VecNd_constMulSelf(internalCoords, -1.0, 4);
      }
    }

    for (size_t j=0; j<traj[i]->getInternalDim(); ++j)
    {
      add(t, internalCoords[j], traj[i]->getTrajectory1D(j)->getName());
    }

    arrayIdx += traj[i]->getDim();
    delete [] internalCoords;
  }
}

MultiGoalConstraint::~MultiGoalConstraint()
{
}

bool MultiGoalConstraint::check(const MatNd* pos, std::vector<TrajectoryND*> traj)
{
  unsigned int nx = 0;

  for (size_t i=0; i<traj.size(); ++i)
  {
    nx += traj[i]->getDim();
  }

  return (nx==pos->m) ? true : false;
}

}   // namespace tropic
