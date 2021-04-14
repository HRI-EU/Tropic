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

#include "TrajectoryActivationHandler.h"

#include <StackVec.h>
#include <Rcs_macros.h>



typedef Rcs::StackVec<double, 8> TaskVec;



namespace tropic
{

TrajectoryActivationHandler::TrajectoryActivationHandler(TrajectoryND* traj,
                                                         const Task* tsk) :
  trajND(traj), task(tsk), active(false)
{
}

TrajectoryActivationHandler::~TrajectoryActivationHandler()
{
}

/*****************************************************************************
 * Shift all activation points backwards in time.
 * From TrajectoryND::step()
 * \todo: Still missing blending part
 ****************************************************************************/
void TrajectoryActivationHandler::step(double dt)
{
  for (int i=activation.size()-1; i>=0; i--)
  {
    activation[i]->shiftTime(-dt);

    if (activation[i]->getTime()<0.0 && activation[i]->getTime()>-dt-1.0e-6)
    {
      this->active = activation[i]->switchesOn();
    }

    // If an activation point falls out of time, we delete it.
    if (activation[i]->getTime()<=-activation[i]->getHorizon()-1.0e-6)
    {
      activation.erase(activation.begin()+i);
    }

  }
}

/*****************************************************************************
 * \todo: This must be called after step() and before update(). See
 * TrajectoryController::step()
 * bool activeBefore = trajectory[i]->isActive();
 * trajectory[i]->step(dt);
 * this->activation->ele[i] = trajectory[i]->isActive();
 ****************************************************************************/
void TrajectoryActivationHandler::update(double dt, bool activeBefore)
{
  // bool activeBefore = trajectory[i]->isActive();
  // trajectory[i]->step(dt);
  // this->activation->ele[i] = trajectory[i]->isActive();

  if (activeBefore==true)
  {
    return;
  }

  const Rcs::Task* task_i = task;
  const unsigned int dimTask = task_i->getDim();
  const size_t dimTraj = trajND->getInternalDim();
  TaskVec x(dimTask);
  TaskVec internalCoords(dimTraj);
  task_i->computeX(x);
  trajND->toInternalCoords(internalCoords, x);


  if (x_pprev.empty())
  {
    for (size_t j=0; j<dimTraj; ++j)
    {
      x_pprev.push_back(internalCoords[j]);
    }
  }

  if (x_prev.empty())
  {
    for (size_t j=0; j<dimTraj; ++j)
    {
      x_prev.push_back(internalCoords[j]);
    }
  }

  x_pprev = x_prev;
  x_prev = x_curr;
  x_curr.clear();
  for (size_t j=0; j<dimTraj; ++j)
  {
    x_curr.push_back(internalCoords[j]);
  }

  TaskVec x_dot(dimTraj);
  TaskVec x_dot_prev(dimTraj);
  TaskVec x_ddot(dimTraj);
  for (size_t j=0; j<dimTraj; ++j)
  {
    x_dot[j] = (x_curr[j] - x_prev[j])/dt;
    x_dot_prev[j] = (x_prev[j] - x_pprev[j])/dt;
    x_ddot[j] = (x_dot[j]-x_dot_prev[j])/dt;
  }

  // If the trajectory was not active before, the step() method of the
  // 1D trajectories was not called. We therefore set the viaNow point
  // to the graph's current state and initialize all trajectories.
  for (size_t j=0; j<dimTraj; ++j)
  {
    trajND->getTrajectory1D(j)->viaNow.setPosition(x_curr[j]);
    trajND->getTrajectory1D(j)->viaNow.setVelocity(x_dot[j]);
    trajND->getTrajectory1D(j)->viaNow.setAcceleration(x_ddot[j]);
    trajND->getTrajectory1D(j)->initFromConstraints();
  }

}

bool TrajectoryActivationHandler::isActive() const
{
  return this->active;
}

}   // namespace tropic
