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

#ifndef TROPIC_VIAPOINTTRAJECTORY1D_H
#define TROPIC_VIAPOINTTRAJECTORY1D_H

#include "Trajectory1D.h"
#include "ViaPointSequence.h"



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for computing minimum-jerk trajectories between constraints.
 *
 *         The class is based on the ViaPointSequence() class of Rcs and
 *         computes a minimum-jerk trajectory between the given constraints.
 *         It fully supports and combination of constraints, such as on
 *         position, velocity and acceleration levels.
 */
class ViaPointTrajectory1D : public Trajectory1D
{
public:

  ViaPointTrajectory1D();
  ViaPointTrajectory1D(double x0, double horizon);
  ViaPointTrajectory1D(const ViaPointTrajectory1D& copyFromMe);
  ViaPointTrajectory1D& operator=(const ViaPointTrajectory1D& rhs);
  virtual ~ViaPointTrajectory1D();
  virtual bool check() const;
  virtual double getPosition(double t) const;
  virtual void computeTrajectoryPoint(double& xt, double& xt_dot,
                                      double& xt_ddot, double t) const;
  virtual bool initFromConstraints();
  virtual const char* getClassName() const;
  virtual const Rcs::ViaPointSequence* getViaSequence() const;
  virtual Rcs::ViaPointSequence* getViaSequence();

protected:

  virtual ViaPointTrajectory1D* clone() const;
  virtual bool dxdPosConstraint(MatNd* grad,
                                const std::shared_ptr<Constraint1D> c,
                                double t0, double t1, double dt) const;
  virtual int getPosRow(unsigned int constraintID) const;

private:

  Rcs::ViaPointSequence* viaSeq;
};

}   // namespace tropic

#endif   // TROPIC_VIAPOINTTRAJECTORY1D_H
