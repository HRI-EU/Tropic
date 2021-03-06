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

#include "ViaPointTrajectory1D.h"
#include "Rcs_macros.h"
#include "Rcs_basicMath.h"
#include "Rcs_Vec3d.h"

#include <algorithm>
#include <functional>
#include <iostream>


namespace tropic
{

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointTrajectory1D::ViaPointTrajectory1D() : Trajectory1D()
{
  this->viaSeq = new Rcs::ViaPointSequence();
  viaSeq->setTurboMode(true);
  initFromConstraints();
}

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointTrajectory1D::ViaPointTrajectory1D(double x0, double horizon_) :
  Trajectory1D(x0, horizon_)
{
  this->viaSeq = new Rcs::ViaPointSequence();
  viaSeq->setTurboMode(true);
  initFromConstraints();
}

/*******************************************************************************
 * Copy constructor doing deep copying.
 ******************************************************************************/
ViaPointTrajectory1D::ViaPointTrajectory1D(const ViaPointTrajectory1D& copyFromMe): Trajectory1D(copyFromMe)
{
  RLOG(5, "Copy-Constructor ViaPointTrajectory1D");
  this->viaSeq = copyFromMe.viaSeq->clone();
}

/*******************************************************************************
 * Copy constructor doing deep copying.
 ******************************************************************************/
ViaPointTrajectory1D& ViaPointTrajectory1D::operator=(const ViaPointTrajectory1D& copyFromMe)
{
  if (this == &copyFromMe)
  {
    return *this;
  }

  Trajectory1D::operator=(copyFromMe);

  if (this->viaSeq != NULL)
  {
    delete this->viaSeq;
  }

  this->viaSeq = copyFromMe.viaSeq->clone();

  return *this;
}

/*******************************************************************************
 * Pointer version of copy
 ******************************************************************************/
ViaPointTrajectory1D* ViaPointTrajectory1D::clone() const
{
  return new ViaPointTrajectory1D(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
ViaPointTrajectory1D::~ViaPointTrajectory1D()
{
  delete this->viaSeq;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ViaPointTrajectory1D::check() const
{
  return viaSeq->check();
}

/*******************************************************************************
 * Updates the via descriptor from the set of TrajectoryConstraint1D. The
 * init() function sorts the array to ensure an increasing time.
 ******************************************************************************/
void ViaPointTrajectory1D::initFromConstraints()
{
  // If no goal point comes after the horizon, we will add or update a horizon
  // point with the coordinates of the most previous goal point within the
  // horizon. If there is none, the coordinates of the viaNow point are used.
  auto lastGoal = viaGoal.empty() ? &viaNow : viaGoal.back().get();

  if (lastGoal->getTime()<horizon-TRAJECTORY1D_ALMOST_ZERO)
  {
    // If no goal point comes after the horizon and no horizon point exists,
    // we create a horizon point with the coordinates of the most previous
    // goal point within the horizon.
    if (this->viaHorizon==NULL)
    {
      this->viaHorizon = lastGoal->clone();
      this->viaHorizon->assignUniqueID();
    }

    // We set the horizon point's coordinates to the last goal point within the
    // horizon, but with zero velocity and acceleration.
    viaHorizon->setTime(horizon);
    viaHorizon->setPosition(lastGoal->getPosition());
    viaHorizon->setVelocity(0.0);
    viaHorizon->setAcceleration(0.0);
  }
  // If a goal point comes after the horizon, we delete the horizon point if
  // it exists.
  else
  {
    if (this->viaHorizon!=NULL)
    {
      delete this->viaHorizon;
      this->viaHorizon = NULL;
    }
  }

  size_t numRows = 1+(viaHorizon ? 1 : 0)+intermediate.size()+viaGoal.size();
  MatNd* viaDescr = MatNd_create(numRows, 6);

  viaNow.toDescriptor(viaDescr, 0);
  size_t offset = 1;

  if (viaHorizon != NULL)
  {
    viaHorizon->toDescriptor(viaDescr, offset);
    offset++;
  }

  for (size_t i=0; i<intermediate.size(); ++i)
  {
    intermediate[i]->toDescriptor(viaDescr, i+offset);
  }

  offset += intermediate.size();
  for (size_t i=0; i<viaGoal.size(); ++i)
  {
    viaGoal[i]->toDescriptor(viaDescr, i+offset);
  }

  bool success = viaSeq->init(viaDescr);

  if (success==false)
  {
    RcsLogLevel = 4;
    viaSeq->check();
    viaSeq->print();
    RFATAL("Failed to initialize via point sequence");
  }

  MatNd_destroy(viaDescr);
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* ViaPointTrajectory1D::getClassName() const
{
  return "ViaPointTrajectory1D";
}

/*******************************************************************************
 *
 ******************************************************************************/
const Rcs::ViaPointSequence* ViaPointTrajectory1D::getViaSequence() const
{
  return this->viaSeq;
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::ViaPointSequence* ViaPointTrajectory1D::getViaSequence()
{
  return this->viaSeq;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ViaPointTrajectory1D::getPosition(double t) const
{
  return viaSeq->computeTrajectoryPos(t);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ViaPointTrajectory1D::computeTrajectoryPoint(double& xt,
                                                  double& xt_dot,
                                                  double& xt_ddot,
                                                  double t) const
{
  viaSeq->computeTrajectoryPoint(xt, xt_dot, xt_ddot, t);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ViaPointTrajectory1D::dxdPosConstraint(MatNd* grad,
                                            const std::shared_ptr<Constraint1D> c,
                                            double t0, double t1,
                                            double dt) const
{
  return viaSeq->gradientDxDvia_a(grad, getPosRow(c->getID()), t0, t1, dt);
}

/*******************************************************************************
 *
 ******************************************************************************/
int ViaPointTrajectory1D::getPosRow(unsigned int constraintID) const
{
  if (viaSeq->viaDescr->n < 6)
  {
    RLOG(0, "Constraint id not contained in via descriptor");
    return -1;
  }

  for (unsigned int i=0; i<viaSeq->viaDescr->m; ++i)
  {
    size_t id_i = lround(MatNd_get(viaSeq->viaDescr, i, 5));

    if (id_i == constraintID)
    {
      unsigned int flag = lround(MatNd_get(viaSeq->viaDescr, i, 4));

      if (Math_isBitSet(flag, VIA_POS))
      {
        return i;
      }
      else
      {
        RLOG(0, "Constraint has no position component");
        return -1;
      }
    }
  }

  return -1;
}

}   // namespace tropic
