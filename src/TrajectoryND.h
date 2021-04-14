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

#ifndef TROPIC_TRAJECTORYND_H
#define TROPIC_TRAJECTORYND_H

#include "Trajectory1D.h"
#include "ActivationPoint.h"



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for computing n-dimensional trajectories.
 *
 *         The class owns a vector of Trajectory1D classes and a few methods
 *         to handle stepping and handling activations. It is in progress,
 *         with the goal of removing the activation handling to give it the
 *         one and only job of dealing with special kinds of n-dimansional
 *         trajectories, such as Euler angles trajectories etc.
 */
class TrajectoryND
{
  friend class TrajectoryControllerBase;

public:

  TrajectoryND();
  TrajectoryND(const TrajectoryND& copyFromMe);
  TrajectoryND& operator=(const TrajectoryND& rhs);
  virtual ~TrajectoryND();

  virtual TrajectoryND* clone() const = 0;
  virtual const char* getClassName() const = 0;
  virtual unsigned int getDim() const;
  virtual unsigned int getInternalDim() const;
  virtual void step(double dt);
  virtual void computeTrajectory(MatNd* trj, double t0, double t1, double dt);
  virtual void computeVelocityTrajectory(MatNd* trj, double t0, double t1,
                                         double dt);
  virtual void computeAccelerationTrajectory(MatNd* trj, double t0, double t1,
                                             double dt);
  virtual void addConstraint(double t, const double* x, const double* x_dot,
                             const double* x_ddot, int flag);
  virtual void toInternalCoords(double* internalCoords, const double* x) const;
  virtual bool fromInternalCoords(double* x, const double* x_internal) const;
  virtual void print() const;

  bool addActivationPoint(std::shared_ptr<ActivationPoint> point);
  double getBlending() const;
  double getContinuousActivation() const;
  bool isActive() const;
  void removeConstraintsAfter(double t);
  void removeConstraintsAfterFirstGoal();
  void getPosition(double* pos) const;   // at t=0
  void getPosition(double t, double* pos) const;
  void getGoalPosition(size_t idx, double* pos);
  void getViaPosition(size_t idx, double* pos);
  double getGoalTime(size_t idx);
  unsigned int getNumberOfConstraints() const;
  size_t getNumberOfActivationPoints() const;
  void init(const double* x);
  void clear(bool clearActivations=false);
  double getTimeOfLastGoal() const;
  double getTimeOfFirstGoal() const;
  void getPositionConstraintBefore(double* x, double t) const;
  Trajectory1D* operator [](int idx);
  Trajectory1D* getTrajectory1D(int idx) const;
  const Trajectory1D* operator [](int idx) const;

  /*! \brief RTTI-based name using demangling.
   */
  std::string getTypeName() const;
  const std::string& getName() const;
  void setName(std::string name);

protected:
  void addTrajectory1D(Trajectory1D* traj1D);

private:
  std::vector<Trajectory1D*> trajVec1D;
  std::vector<std::shared_ptr<ActivationPoint>> activation;
  bool active;
  double blending;
  double continuousActivation;
  std::string name;

public:
  std::vector<double> x_curr;
  std::vector<double> x_prev;
  std::vector<double> x_pprev;
  std::vector<double> x_ppprev;
};

}   // namespace tropic



#include "TrajectoryPosND.h"
#include "TrajectoryPos3D.h"
#include "TrajectoryEuler.h"
#include "TrajectoryPolar.h"

#include "ViaPointTrajectory1D.h"
#include "ZigZagTrajectory1D.h"



namespace tropic
{
typedef TrajectoryPolar<ViaPointTrajectory1D> ViaPointTrajectoryPolar;
typedef TrajectoryPos3D<ViaPointTrajectory1D> ViaPointTrajectoryPosition;
typedef TrajectoryEuler<ViaPointTrajectory1D> ViaPointTrajectoryEuler;
typedef TrajectoryPosND<ViaPointTrajectory1D> ViaPointTrajectoryND;

typedef TrajectoryPolar<ZigZagTrajectory1D> ZigZagTrajectoryPolar;
typedef TrajectoryPos3D<ZigZagTrajectory1D> ZigZagTrajectoryPosition;
typedef TrajectoryEuler<ZigZagTrajectory1D> ZigZagTrajectoryEuler;
typedef TrajectoryPosND<ZigZagTrajectory1D> ZigZagTrajectoryND;
}

#endif   // TROPIC_TRAJECTORYND_H
