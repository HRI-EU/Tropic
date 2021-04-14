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

#include "ZigZagTrajectory1D.h"
#include "Rcs_macros.h"

#include <algorithm>
#include <functional>
#include <iostream>

#define WITH_SHARED_PTR


namespace tropic
{

/*******************************************************************************
 *
 ******************************************************************************/
ZigZagTrajectory1D::ZigZagTrajectory1D(double x0, double horizon_) :
  Trajectory1D(x0, horizon_)
{
  delete this->viaHorizon;
  this->viaHorizon = NULL;
  initFromConstraints();
}

/*******************************************************************************
 *
 ******************************************************************************/
ZigZagTrajectory1D::ZigZagTrajectory1D(const ZigZagTrajectory1D& copyFromMe): Trajectory1D(copyFromMe)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
ZigZagTrajectory1D& ZigZagTrajectory1D::operator=(const ZigZagTrajectory1D& copyFromMe)
{
  RLOG(0, "=== ZigZagTrajectory1D");
  if (this == &copyFromMe)
  {
    return *this;
  }

  Trajectory1D::operator=(copyFromMe);

  return *this;
}

/*******************************************************************************
 * Pointer version of copy
 ******************************************************************************/
ZigZagTrajectory1D* ZigZagTrajectory1D::clone() const
{
  return new ZigZagTrajectory1D(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
ZigZagTrajectory1D::~ZigZagTrajectory1D()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void ZigZagTrajectory1D::initFromConstraints()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* ZigZagTrajectory1D::getClassName() const
{
  return "ZigZagTrajectory1D";
}

/*******************************************************************************
 *
 ******************************************************************************/
double ZigZagTrajectory1D::getPosition(double t) const
{
  double xt, tmp;
  computeTrajectoryPoint(xt, tmp, tmp, t);
  return xt;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ZigZagTrajectory1D::computeTrajectoryPoint(double& xt,
                                                double& xt_dot,
                                                double& xt_ddot,
                                                double t) const
{
  const Constraint1D* before = getConstraintPtrBefore(t);

  if (before==NULL)
  {
    xt = viaNow.getPosition();
    xt_dot = 0.0;
    xt_ddot = 0.0;
    return;
  }

  const Constraint1D* after = getConstraintPtrAfter(t);

  if (after==NULL)
  {
    xt = before->getPosition();
    xt_dot = 0.0;
    xt_ddot = 0.0;
    return;
  }

  const double dt = after->getTime()-before->getTime();
  const double dx = after->getPosition()-before->getPosition();

  xt = before->getPosition() + dx*(t-before->getTime())/dt;
  xt_dot = dx/dt;
  xt_ddot = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ZigZagTrajectory1D::dxdPosConstraint(MatNd* grad,
                                          const std::shared_ptr<Constraint1D> c,
                                          double t0, double t1,
                                          double dt) const
{
  return false;
}


/*******************************************************************************
 *
 ******************************************************************************/
const Constraint1D* ZigZagTrajectory1D::getConstraintPtrAfter(double t) const
{
  const Constraint1D* after1 = getGoalConstraintPtrAfter(t);
  const Constraint1D* after2 = getViaConstraintPtrAfter(t);

  if (after1==NULL)
  {
    if (after2==NULL)
    {
      return NULL;
    }
    else
    {
      return after2;
    }
  }
  else
  {
    if (after2==NULL)
    {
      return after1;
    }
    else
    {
      return after1->getTime() < after2->getTime() ? after1 : after2;
    }

  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Constraint1D* ZigZagTrajectory1D::getGoalConstraintPtrAfter(double t) const
{
  for (size_t i=0; i<viaGoal.size(); ++i)
  {
    if (viaGoal[i]->getTime() >= t)
    {
#ifndef WITH_SHARED_PTR
      return viaGoal[i];
#else
      return viaGoal[i].get();
#endif
    }
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Constraint1D* ZigZagTrajectory1D::getViaConstraintPtrAfter(double t) const
{
  for (size_t i=0; i<intermediate.size(); ++i)
  {
    if (intermediate[i]->getTime() >= t)
    {
#ifndef WITH_SHARED_PTR
      return intermediate[i];
#else
      return intermediate[i].get();
#endif
    }
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Constraint1D* ZigZagTrajectory1D::getConstraintPtrBefore(double t) const
{
  const Constraint1D* after1 = getGoalConstraintPtrBefore(t);
  const Constraint1D* after2 = getViaConstraintPtrBefore(t);


  if (after1==NULL)
  {
    if (after2==NULL)
    {
      return NULL;
    }
    else
    {
      return after2;
    }
  }
  else
  {
    if (after2==NULL)
    {
      return after1;
    }
    else
    {
      return after1->getTime() > after2->getTime() ? after1 : after2;
    }

  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Constraint1D* ZigZagTrajectory1D::getGoalConstraintPtrBefore(double t) const
{
  if (t < 0.0)
  {
    return NULL;
  }

  if (viaGoal.empty())
  {
    return &viaNow;
  }

  for (int i=viaGoal.size()-1; i>=0; i--)
  {
    if (viaGoal[i]->getTime() < t)
    {
#ifndef WITH_SHARED_PTR
      return viaGoal[i];
#else
      return viaGoal[i].get();
#endif
    }
  }

  if (viaNow.getTime() < t)
  {
    return &viaNow;
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Constraint1D* ZigZagTrajectory1D::getViaConstraintPtrBefore(double t) const
{
  if (intermediate.empty())
  {
    return NULL;
  }

  for (int i=intermediate.size()-1; i>=0; i--)
  {
    if (intermediate[i]->getTime() < t)
    {
#ifndef WITH_SHARED_PTR
      return intermediate[i];
#else
      return intermediate[i].get();
#endif
    }
  }

  return NULL;
}

}   // namespace tropic
