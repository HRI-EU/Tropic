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

#ifndef TROPIC_ZIGZAGTRAJECTORY1D_H
#define TROPIC_ZIGZAGTRAJECTORY1D_H

#include "Trajectory1D.h"



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for computing linear segments between constraints.
 *
 *         The class serves as an example how to implement a new Trajectory1D
 *         class, and for testing. It interprets each constraint as a through
 *         point (regardless if it is a position, velocity, or acceleration
 *         constraint), and computes a trajectory that linearly interpolates
 *         between these constraints. The resulting trajectories have edges on
 *         position level, which are velocity jumps, and therefore should not
 *         be applied to a real robot without any furher processing.
 */
class ZigZagTrajectory1D : public Trajectory1D
{
public:
  ZigZagTrajectory1D(double x0, double horizon);
  ZigZagTrajectory1D(const ZigZagTrajectory1D& copyFromMe);
  ZigZagTrajectory1D& operator=(const ZigZagTrajectory1D& rhs);
  virtual ~ZigZagTrajectory1D();
  virtual ZigZagTrajectory1D* clone() const;
  virtual bool initFromConstraints();
  virtual double getPosition(double t) const;
  virtual void computeTrajectoryPoint(double& xt, double& xt_dot,
                                      double& xt_ddot, double t) const;
  virtual const char* getClassName() const;
  virtual bool dxdPosConstraint(MatNd* grad,
                                const std::shared_ptr<Constraint1D> c,
                                double t0, double t1, double dt) const;

  /*! \brief Returns the pointer to the goal constraint that is the first one
   *         later than the given time. If none exists, the function returns
   *         NULL.
   *
   *  \return Pointer to the goal constraint that is later than
   *          the given time.
   */
  const Constraint1D* getGoalConstraintPtrAfter(double t) const;
  const Constraint1D* getViaConstraintPtrAfter(double t) const;
  const Constraint1D* getConstraintPtrAfter(double t) const;

  /*! \brief Returns the pointer to the goal constraint that is the first one
   *         earlier than the given time. The viaNow constraint also counts as
   *         a goal constraint. If none exists, the function returns NULL.
   *
   *  \return Pointer to the goal constraint that is earlier than
   *          the given time.
   */
  const Constraint1D* getGoalConstraintPtrBefore(double t) const;
  const Constraint1D* getViaConstraintPtrBefore(double t) const;
  const Constraint1D* getConstraintPtrBefore(double t) const;
};

}   // namespace tropic

#endif   // TROPIC_ZIGZAGTRAJECTORY1D_H
