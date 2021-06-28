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

#ifndef TROPIC_TRAJECTORY1D_H
#define TROPIC_TRAJECTORY1D_H

#include "Constraint1D.h"

#include <Rcs_MatNd.h>

#define TRAJECTORY1D_ALMOST_ZERO (1.0e-6)

/*!
 *  \defgroup Tropic Trajectory classes
 *
 */

namespace tropic
{

/*! \ingroup Tropic
 *  \brief Abstract base class for one-dimensional trajectories.
 *
 *         The Trajectory1D class is responsible to maintain and organize a set
 *         of constraints, and to step them through time. The core function is
 *         the step function, which will update the "now" time point with the
 *         updated values (position, velocity and acceleration), and shift all
 *         constraints backward in time. The step function also handles internal
 *         things such as making sure that the last constraint is always one on
 *         position, velocity, and accleration level.
 *
 *         The Trajectory1D class is abstract in the sense of how to calculate
 *         the trajectory profile. The abstract methods determine which kind of
 *         trajectory method is used. Some examples (ViaPointTrajectory and
 *         the very simple ZigZagTrajectory) are examples for this. However, it
 *         is possible to add many others.
 *
 */
class Trajectory1D
{
public:

  /*! \brief Default trajectory initialized with 0 and horizon 1sec.
   */
  Trajectory1D();

  /*! \brief Trajectory initialized with x0 and the given horizon.
   */
  Trajectory1D(double x0, double horizon);

  /*! \brief Copy constructor.
   */
  Trajectory1D(const Trajectory1D& copyFromMe);

  /*! \brief Virtual destructor to enable Polymorphism.
   */
  virtual ~Trajectory1D() = 0;

  /*! \brief Assignment operator.
   */
  Trajectory1D& operator=(const Trajectory1D& rhs);

  /*! \brief Steps the trajectory in time by shifing all constraints backwards
   *         by dt.
   */
  virtual void step(double dt);

  /*! \brief Adds a constraint to the trajectory that is defined by position,
   *         velocity and acceleration. Which of these elements become active is
   *         defined with the flag.
   */
  void addConstraint(double t, double x, double x_dot, double x_ddot, int flag);

  /*! \brief Adds the constraint pointed to by the argument. It is directly
   *         added to the trajectory, no copy is made.
   */
  void addConstraint(std::shared_ptr<Constraint1D> constraint);

  /*! \brief Returns the number of goal points (with constraints on
   *         position, velocity- and acceleration-level only). This
   *         excludes the now-point, and a potential horizon point.
   */
  unsigned int getNumberOfGoals() const;

  /*! \brief Returns the number of via points (with constraints on
   *         position-level only).
   */
  unsigned int getNumberOfViaPoints() const;

  /*! \brief Returns the number of goal and via points.
   */
  unsigned int getNumberOfConstraints() const;

  /*! \brief Returns the goal point position of the idx-th via point constraint.
   *         If idx is out of range, an error will occur.
   */
  double getGoalPosition(size_t idx) const;

  /*! \brief Returns the time of the horizon point.
   */
  double getHorizonTime() const;

  /*! \brief Returns the goal point time of the idx-th via point constraint.
   *         If there is no goal, the function returns 0. If idx is out of
   *         range, an error will occur.
   */
  double getGoalTime(size_t idx) const;

  /*! \brief Returns the via point time of the idx-th via point constraint.
   *         If idx is out of range, an error will occur.
   */
  double getViaTime(size_t idx) const;

  /*! \brief Returns the via point position of the idx-th via point constraint.
   *         If idx is out of range, an error will occur.
   */
  double getViaPosition(size_t idx) const;

  /*! \brief Returns a pointer to the goal constraint at the given index.
   *
   *  \param[in] idx   Index of the intermediate constraint to be searched for
   *  \return Pointer to goal constraint, or NULL if the index is out of range.
   */
  std::shared_ptr<Constraint1D> getGoalPtr(size_t idx);

  /*! \brief Returns a pointer to the intermediate constraint at the given
   *         index.
   *
   *  \param[in] idx   Index of the intermediate constraint to be searched for
   *  \return Pointer to intermediate constraint, or NULL if the index is out
   *          of range.
   */
  std::shared_ptr<Constraint1D> getViaPtr(size_t idx);

  /*! \brief Returns a pointer to the constraint at the time horizon. If there
   *         is a full constraint later than the horizon, there is no constraint
   *         at the horizon.
   *
   *  \return Pointer to horizon constraint, or NULL if no constraint exists.
   */
  virtual Constraint1D* getHorizonPtr();

  /*! \brief Returns a pointer to the constraint at t=0. This constraint always
   *         exists, and it is a constraint on position, velocity and
   *         acceleration levels.
   *
   *  \return Pointer to t=0 constraint.
   */
  virtual Constraint1D* getNowPtr();

  /*! \brief Computes the position of the position constraint most previous or
   *         at the same point in time as the given time. If there is no
   *         position constraint, the function returns the position of the
   *         trajectory at time 0. Otherwise the returned value is the position
   *         from the closest constraint, and not the value determined the
   *         trajectory. This should make a difference if the trajectory is
   *         not exactly satisfying the constraints, and for ambiguous
   *         descriptions such as quaternions.
   *         Sticking to the stored values in the constraints will ensure
   *         that the interpolation will be done consistently, e.g. on the
   *         shortest path in quaternion space.
   *
   *  \param[in] t     Time until when the previous position constraint is
   *                   to be searched.
   *  \return Position of the previous position constraint, or the initial
   *          value at time 0 of the trajectory if no constraint exists.
   */
  double getPositionConstraintBefore(double t) const;

  /*! \brief Prints out all constraint information to stdout.
   */
  virtual void print() const;

  /*! \brief Hard initialization of the trajectory to a given position value.
   *         This function might lead to a jump in the current trajectory
   *         value and therefore should be used with care. It is meant for
   *         initialization purposes, and not for using when the trajectory
   *         is stepped and expected to be smooth.
   */
  virtual void init(double x);

  /*! \brief Same as \ref init(double x), but with additional arguments that
   *         specify velocity and acceleration for the state to be initialized.
   */
  virtual void init(double x, double x_dot, double x_ddot);

  /*! \brief Keeps the current constraint at t=0, clears all future via and
   *         goal constraints, and sets a goal constraint with the current
   *         value at the time point of the horizon. It means that the
   *         trajectory will come to a stop with the same value as when calling
   *         the clear() method at t=t_horizon.
   */
  virtual void clear();

  /*! \brief Same as \ref print(), but as ostream output.
   */
  friend std::ostream& operator<<(std::ostream& out, const Trajectory1D& traj);

  /*! \brief Removes all constraints after the given time. The function does not
   *         take care that there is a valid horizon point after removal.
   */
  virtual void removeConstraintsAfter(double t);

  /*! \brief Removes all constraints after the first goal constraint.
   */
  virtual void removeConstraintsAfterFirstGoal();

  /*! \brief Removes the given constraint.
   *
   * \return True if the constraint was erased, false if the constraint was not
   *         found in the trajectory.
   */
  virtual bool erase(std::shared_ptr<Constraint1D> constraint);

  /*! \brief Returns a deep copies instance created on the heap. Must be
   *         re-implemented in each derived class.
   */
  virtual Trajectory1D* clone() const = 0;

  /*! \brief Returns the trajectoriy position at t=0;
   */
  virtual double getPosition() const;

  /*! \brief Returns the trajectoriy velocity at t=0;
   */
  virtual double getVelocity() const;

  /*! \brief Returns the acceleration velocity at t=0;
   */
  virtual double getAcceleration() const;

  /*! \brief Returns the trajectoriy position at time t. The caller needs to
   *         ensure that the trajectory parameters have been computed so that
   *         the query at time t is valid.
   */
  virtual double getPosition(double t) const = 0;

  /*! \brief Computes the trajectoriy position, velocity and acceleration at
   *         time t. The caller needs to ensure that the trajectory parameters
   *         have been computed so that the query at time t is valid.
   */
  virtual void computeTrajectoryPoint(double& xt, double& xt_dot,
                                      double& xt_ddot, double t) const = 0;

  /*! \brief Returns the class name of the trajectory. Every non-virtual child
   *         of this class should define a unique name.
   */
  virtual const char* getClassName() const = 0;

  /*! \brief Accessor for the name given to the trajectory.  This name is used
   *         for looking up the trajectory once a ConstraintSet is applied.
   */
  const std::string& getName() const;

  /*! \brief Assigns a name to the trajectory. This name will be used for
   *         looking up the trajectory once a ConstraintSet is applied.
   */
  void setName(std::string name);

  /*! \brief Function to calculate any parameters needed prior to querying
   *         trajectory positions, velocities or accelerations. For instance
   *         when using polynomial trajectories, this function must calculate
   *         the polyynomial parameters.
   */
  virtual bool initFromConstraints() = 0;

  /*! \brief Computes the gradient of the constraint c between t0 and t1,
   *         spaced with dt. The dimension of the gradient is m x n, where
   *         m is the number of constraint elements in c(e.g. 1 for position
   *         only, 3 for position, velocity and acceleration), and n is the
   *         number of time steps (t1-t0)/dt. This method depends on the type
   *         of the trajectory profile and needs to be implemented for each
   *         specialized class derieved from this one.
   */
  virtual bool dxdPosConstraint(MatNd* grad,
                                const std::shared_ptr<Constraint1D> c,
                                double t0, double t1, double dt) const = 0;


protected:
  virtual void preStep(double dt);
  void clearConstraints();

  double horizon;
  Constraint1D viaNow;
  Constraint1D* viaHorizon;
  std::vector<std::shared_ptr<Constraint1D>> viaGoal;
  std::vector<std::shared_ptr<Constraint1D>> intermediate;
  std::string name;
};

}   // namespace tropic



#endif   // TROPIC_TRAJECTORY1D_H
