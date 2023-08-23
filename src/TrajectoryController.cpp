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

#include "TrajectoryController.h"
#include "GraphConstraint.h"

#include <Rcs_macros.h>

#include <iostream>
#include <algorithm>


namespace tropic
{

TrajectoryControllerBase::TrajectoryControllerBase() :
  controller(NULL), ownController(NULL), rootSet(), reactivationScaling(1.0)
{
}

TrajectoryControllerBase::TrajectoryControllerBase(const TrajectoryControllerBase& copyFromMe) :
  rootSet(copyFromMe.rootSet)
{
  NLOG_CPP(0, "Cloning " << copyFromMe.trajectory.size() << " trajectories");

  for (size_t i=0; i<copyFromMe.trajectory.size(); ++i)
  {
    NLOG_CPP(0, "Cloning trajectory " << i);
    TrajectoryND* ti = copyFromMe.trajectory[i]->clone();
    ti->clear();
    trajectory.push_back(ti);
  }

  this->ownController = new Rcs::ControllerBase(*(copyFromMe.getController()));
  this->controller = ownController;
  this->reactivationScaling = copyFromMe.reactivationScaling;

  // We apply the constraints permissively for cases where we have modified the
  // trajectory vector and some of the constraints are not valid any more.
  const bool permissive = true;
  rootSet.apply(trajectory, permissive);
  NLOG_CPP(0, "done rootSet.apply(trajectory)");
}

TrajectoryControllerBase& TrajectoryControllerBase::operator=(const TrajectoryControllerBase& copyFromMe)
{
  if (this == &copyFromMe)
  {
    return *this;
  }

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    delete trajectory[i];
  }
  this->trajectory.clear();

  for (size_t i=0; i<copyFromMe.trajectory.size(); ++i)
  {
    TrajectoryND* ti = copyFromMe.trajectory[i]->clone();
    ti->clear();
    trajectory.push_back(ti);
  }

  delete this->ownController;
  this->ownController = new Rcs::ControllerBase(*(copyFromMe.getController()));
  this->controller = ownController;
  this->reactivationScaling = copyFromMe.reactivationScaling;

  rootSet.clear();
  rootSet = copyFromMe.rootSet;
  rootSet.apply(trajectory);

  return *this;
}

TrajectoryControllerBase::~TrajectoryControllerBase()
{
  for (size_t i=0; i<trajectory.size(); ++i)
  {
    delete trajectory[i];
  }

  delete this->ownController;   // Ok to call on NULL
}

TrajectoryControllerBase* TrajectoryControllerBase::clone() const
{
  return new TrajectoryControllerBase(*this);
}

unsigned int TrajectoryControllerBase::getDim() const
{
  return controller->getTaskDim();
}

const char* TrajectoryControllerBase::getClassName() const
{
  return "TrajectoryController";
}

bool TrajectoryControllerBase::populateTrajectories(double horizon)
{
  RLOG(0, "populateTrajectories in base class not implemented");
  return false;
}

void TrajectoryControllerBase::addTrajectory(const Rcs::Task* task_i, double horizon)
{
  RFATAL("addTrajectoryin base class not implemented");
}

unsigned int TrajectoryControllerBase::getNumberOfConstraints() const
{
  unsigned int nc = 0;

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    nc += trajectory[i]->getNumberOfConstraints();
  }

  return nc;
}

unsigned int TrajectoryControllerBase::getNumberOfSetConstraints() const
{
  return rootSet.numConstraints();
}

unsigned int TrajectoryControllerBase::getNumberOfSets() const
{
  return rootSet.numSets();
}

void TrajectoryControllerBase::getPosition(MatNd* pos)
{
  unsigned int arrayIdx = 0;
  MatNd_reshape(pos, getDim(), 1);

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    trajectory[i]->getPosition(&pos->ele[arrayIdx]);
    arrayIdx += trajectory[i]->getDim();
  }
}

void TrajectoryControllerBase::getPosition(double t, MatNd* pos)
{
  unsigned int arrayIdx = 0;
  MatNd_reshape(pos, getDim(), 1);

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    trajectory[i]->getPosition(t, &pos->ele[arrayIdx]);
    arrayIdx += trajectory[i]->getDim();
  }
}

void TrajectoryControllerBase::add(std::shared_ptr<ConstraintSet> cSet)
{
  rootSet.add(cSet);
}

bool TrajectoryControllerBase::addAndApply(std::shared_ptr<ConstraintSet> cSet,
                                           bool permissive)
{
  if (!cSet)
  {
    return false;
  }

  rootSet.add(cSet);
  cSet->apply(getTrajectoriesRef(), permissive);

  std::vector<GraphConstraint*> graphSets;
  //RLOG(0, "Starting to collect ...");
  rootSet.findSetsOfType<GraphConstraint>(graphSets);

  // The graph sets may modify the graph. This is only permitted if the class
  // owns a controller, otherwise the member pointer is const. We check this
  // here.
  if (!graphSets.empty())
  {
    RCHECK(getInternalController());
    RCHECK(getInternalController()->getGraph());

    //RLOG(0, "Starting to print ...");
    for (size_t i=0; i<graphSets.size(); ++i)
    {
      //RLOG_CPP(0, "Set " << i << " is of class " << graphSets[i]->getClassName());
      graphSets[i]->setGraph(getInternalController()->getGraph());
    }
  }

  return true;
}

void TrajectoryControllerBase::addGoalConstraint(double t, const MatNd* pos)
{
  unsigned int arrayIdx = 0;

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    trajectory[i]->addConstraint(t, &pos->ele[arrayIdx], NULL, NULL, 7);
    arrayIdx += trajectory[i]->getDim();
  }
}

void TrajectoryControllerBase::init()
{
  MatNd* x_curr = MatNd_create(controller->getTaskDim(), 1);
  controller->computeX(x_curr);

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    const double* x_i = &x_curr->ele[controller->getTaskArrayIndex(i)];
    trajectory[i]->init(x_i);
  }

  MatNd_destroy(x_curr);
}

double TrajectoryControllerBase::getTimeOfLastGoal() const
{
  double t_last = 0.0;

  for (size_t i = 0; i<trajectory.size(); ++i)
  {
    t_last = std::max(t_last, trajectory[i]->getTimeOfLastGoal());
  }

  return t_last;
}

void TrajectoryControllerBase::clear(bool clearActivations)
{
  rootSet.clear();
  for (size_t i=0; i<trajectory.size(); ++i)
  {
    trajectory[i]->clear(clearActivations);
  }
}

void TrajectoryControllerBase::clearConstraintSet()
{
  rootSet.clear();
}

void TrajectoryControllerBase::addTrajectory(TrajectoryND* traj)
{
  trajectory.push_back(traj);
}

bool TrajectoryControllerBase::eraseTrajectory(const std::string& name)
{
  int idx = controller->getTaskIndex(name.c_str());

  if (idx==-1)
  {
    return false;
  }

  return eraseTrajectory(idx);
}

bool TrajectoryControllerBase::eraseTrajectory(size_t index)
{
  if (trajectory.empty())
  {
    return false;
  }

  if (index > trajectory.size() - 1)
  {
    RLOG_CPP(1, "Failed to erase trajectory with index " << index
             << " - should be less than " << trajectory.size());
    return false;
  }

  delete trajectory[index];
  trajectory.erase(trajectory.begin() + index);

  return true;
}

void TrajectoryControllerBase::eraseTrajectories()
{
  rootSet.clear();

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    delete trajectory[i];
  }

  trajectory.clear();
}

void TrajectoryControllerBase::print() const
{
  for (size_t i=0; i<trajectory.size(); ++i)
  {
    std::cout << "Trajectory " << i << std::endl;
    trajectory[i]->print();
  }
}

bool TrajectoryControllerBase::check() const
{
  bool success = true;

  success = checkDuplicateTrajectories();

  return success;
}

bool TrajectoryControllerBase::checkDuplicateTrajectories() const
{
  bool success = true;

  // Check for duplicate names in the 1-d trajectories
  std::map<std::string,Trajectory1D*> tMap;

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    const size_t dim = trajectory[i]->getInternalDim();

    for (size_t j=0; j<dim; ++j)
    {
      Trajectory1D* tj = trajectory[i]->getTrajectory1D(j);

      std::map<std::string, Trajectory1D*>::iterator it;
      it = tMap.find(tj->getName());
      if (it!=tMap.end())
      {
        RLOG(1, "Trajectory \"%s\" is duplicate", tj->getName().c_str());
        success = false;
      }

      tMap[tj->getName()] = tj;
    }

  }

  return success;
}

double TrajectoryControllerBase::getActivation(int idx) const
{
  return trajectory[idx]->isActive() ? 1.0 : 0.0;
}

void TrajectoryControllerBase::getActivation(MatNd* a) const
{
  MatNd_reshape(a, trajectory.size(), 1);

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    MatNd_set(a, i, 0, trajectory[i]->isActive() ? 1.0 : 0.0);
  }

}

void TrajectoryControllerBase::getContinuousActivation(MatNd* a) const
{
  for (size_t i=0; i<trajectory.size(); ++i)
  {
    MatNd_set(a, i, 0, trajectory[i]->getContinuousActivation());
  }

}

void TrajectoryControllerBase::setActivation(int idx, bool activity)
{
  trajectory[idx]->addActivationPoint(std::make_shared<ActivationPoint>(0.0, activity, 0.0));
  trajectory[idx]->active = activity;
}

void TrajectoryControllerBase::setActivation(bool activity)
{
  for (size_t i=0; i<trajectory.size(); ++i)
  {
    setActivation(i, activity);
  }
}

void TrajectoryControllerBase::setActivation(const MatNd* activation)
{
  for (size_t i=0; i<trajectory.size(); ++i)
  {
    setActivation(i, MatNd_get(activation, i, 0));
  }
}

double TrajectoryControllerBase::computeBlending() const
{
  double blending = 1.0;

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    blending = fmin(trajectory[i]->getBlending(), blending);
  }

  return blending;
}

TrajectoryND* TrajectoryControllerBase::getTrajectory(const std::string& taskName) const
{
  int idx = controller->getTaskIndex(taskName.c_str());

  if (idx==-1)
  {
    return NULL;
  }

  return trajectory[idx];
}

TrajectoryND* TrajectoryControllerBase::getTrajectory(size_t idx) const
{
  return trajectory[idx];
}

std::vector<TrajectoryND*> TrajectoryControllerBase::getTrajectories() const
{
  return trajectory;
}

std::vector<TrajectoryND*>& TrajectoryControllerBase::getTrajectoriesRef()
{
  return trajectory;
}

const Rcs::Task* TrajectoryControllerBase::getTask(const std::string& taskName) const
{
  return controller->getTask(taskName.c_str());
}

const Rcs::ControllerBase* TrajectoryControllerBase::getController() const
{
  return this->controller;
}

Rcs::ControllerBase* TrajectoryControllerBase::getInternalController()
{
  return this->ownController;
}

const ConstraintSet& TrajectoryControllerBase::getRootSet() const
{
  return this->rootSet;
}

void TrajectoryControllerBase::stepTrajectoryND(TrajectoryND* tnd,
                                                const Rcs::Task* task_i,
                                                double dt)
{
  bool activeBefore = tnd->isActive();

  // Shift all activation points backwards in time, and erase the activation
  // points that fall below time 0.
  for (int j=tnd->activation.size()-1; j>=0; j--)
  {
    tnd->activation[j]->shiftTime(-dt);

    if ((dt!=0.0) && (tnd->activation[j]->getTime()<0.0) &&
        (tnd->activation[j]->getTime()>=-dt))
    {
      tnd->active = tnd->activation[j]->switchesOn();
    }

    // Compute continuous activation in period after the activation point. It
    // has an effect on the activation in the horizon after the activation time.
    if ((tnd->activation[j]->getTime() <= 0.0) &&
        (tnd->activation[j]->getTime() > -tnd->activation[j]->getHorizon()))
    {
      // Activation point affects continuous activation
      if (tnd->active)
      {
        tnd->continuousActivation += dt/tnd->activation[j]->getHorizon();
      }
      else
      {
        tnd->continuousActivation -= dt/tnd->activation[j]->getHorizon();
      }

      tnd->continuousActivation = Math_clip(tnd->continuousActivation, 0.0, 1.0);
      // RLOG_CPP(0, "Trajectory " << tnd->getName() << ": cont. activation is "
      //          << tnd->continuousActivation);
    }

    // If an activation point falls out of time, we delete it.
    if (tnd->activation[j]->getTime()<=-tnd->activation[j]->getHorizon()-1.0e-6)
    {
      tnd->activation.erase(tnd->activation.begin()+j);
    }

  }

  // This computes the blending factor for the null space motion
  tnd->blending = 1.0;

  for (auto& aPoint : tnd->activation)
  {
    tnd->blending = fmin(tnd->blending, aPoint->computeBlending());
  }

  // Compute the trajectory state that is consistent with the current state
  // of the kinematic model. This needs to be done before stepping the
  // trajectories.
  const size_t dimTraj = tnd->getInternalDim();
  std::vector<double> x_tsk(task_i->getDim()), x_trj(dimTraj);
  task_i->computeX(x_tsk.data());
  tnd->toInternalCoords(x_trj.data(), x_tsk.data());

  // This happens only at the very first call to step()
  if (tnd->x_ppprev.empty() || tnd->x_pprev.empty() || tnd->x_prev.empty() || tnd->x_curr.empty())
  {
    tnd->x_curr = x_trj;
    tnd->x_ppprev = x_trj;
    tnd->x_pprev = x_trj;
    tnd->x_prev = x_trj;
  }

  // Store the last trajectory states
  tnd->x_ppprev = tnd->x_pprev;
  tnd->x_pprev = tnd->x_prev;
  tnd->x_prev = tnd->x_curr;
  tnd->x_curr = x_trj;


  // if (!activeBefore)
  if (!tnd->isActive() || (tnd->isActive() && (!activeBefore)))
  {
    // Compute the current trajectory velocity and acceleration according to
    // the state. We don't use the velocity functions of the tasks, but
    // rather perform finite differencing of the previous values. That's
    // computationally cheaper, more robust, and we already have polluted
    // this class with internal state.
    for (size_t j=0; j<dimTraj; ++j)
    {
      // First order backward differencing, error is O(dt)
      // double x_dot = (tnd->x_curr[j] - tnd->x_prev[j])/dt;
      // double x_dot_prev = (tnd->x_prev[j] - tnd->x_pprev[j])/dt;
      // double x_ddot = (x_dot-x_dot_prev)/dt;

      // Second order backward differencing, error is O(dt^2)
      // See http://www.math.ubc.ca/~jfeng/CHBE553/Example7/Formulae.pdf
      double x_dot = 0.0, x_ddot = 0.0;

      if (dt != 0.0)
      {
        x_dot = (3.0*tnd->x_curr[j] - 4.0*tnd->x_prev[j] + tnd->x_pprev[j])/(2.0*dt);
        x_ddot = (2.0*tnd->x_curr[j] - 5.0*tnd->x_prev[j] + 4.0*tnd->x_pprev[j] - tnd->x_ppprev[j])/(dt*dt);
      }

      // If the trajectory was not active before, the step() method of the
      // 1D trajectories was not called. We therefore set the viaNow point
      // to the graph's current state and initialize all trajectories.
      Trajectory1D* traj1d = tnd->getTrajectory1D(j);
      Constraint1D* viaNow = traj1d->getNowPtr();
      viaNow->setPosition(tnd->x_curr[j]);
      viaNow->setVelocity(reactivationScaling*x_dot);
      viaNow->setAcceleration(reactivationScaling*x_ddot);

      // If we activate a task that has no goal, we want to keep its position
      // TrajectoryConstraint1D* hrz = traj1d->getHorizonPtr();
      // if (hrz)
      // {
      //   hrz->setPosition(tnd->x_curr[j]);
      // }

      // This computes the polynomial parameters, which is essential for the
      // subsequent step() method that determines the full state at t=dt
      traj1d->initFromConstraints();
    }
  }

  // Step the horizon. We must do this after the above computation, otherwise
  // the values to re-initialize the trajectories lag behind one step and we
  // get a velocity-to-zero jump. We must also step inactive constraints,
  // otherwise their time point is not shifted.
  for (size_t j=0; j<dimTraj; ++j)
  {
    tnd->getTrajectory1D(j)->step(dt);
  }

}

// The function is called with an updated RcsGraph corresponding to the IK
// state after integration.
double TrajectoryControllerBase::step(double dt)
{
  // Computes end time and erases constraints from sets if timed out
  double endTime = rootSet.compute(dt);

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    const Rcs::Task* task_i = controller->getTask(i);
    stepTrajectoryND(trajectory[i], task_i, dt);
    //this->activation->ele[i] = trajectory[i]->isActive();
  }

  return endTime;
}

void TrajectoryControllerBase::setTurboMode(bool enable)
{
  for (size_t j=0; j<trajectory.size(); ++j)
  {
    for (size_t i = 0; i < trajectory[j]->getInternalDim(); ++i)
    {
      ViaPointTrajectory1D* viaTraj = dynamic_cast<ViaPointTrajectory1D*>(trajectory[j]->getTrajectory1D(i));
      if (viaTraj)
      {
        viaTraj->getViaSequence()->setTurboMode(enable);
      }
    }

  }

}

void TrajectoryControllerBase::reactivateWithZeroVelAcc(bool enable)
{
  if (enable)
  {
    this->reactivationScaling = 0.0;
  }
  else
  {
    this->reactivationScaling = 1.0;
  }
}

bool TrajectoryControllerBase::toXML(const std::string& xmlFileName) const
{
  return rootSet.toXML(xmlFileName);
}

void TrajectoryControllerBase::takeControllerOwnership(bool ownership)
{
  this->ownController = ownership ? (Rcs::ControllerBase*) controller : NULL;
}

}   // namespace tropic
