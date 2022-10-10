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

#include "TrajectoryND.h"
#include "Trajectory1D.h"
#include "Rcs_macros.h"

#include <typeinfo>
#include <algorithm>
#include <sstream>

#if !defined (_MSC_VER)
#include <cxxabi.h>
using __cxxabiv1::__cxa_demangle;
#endif

namespace tropic
{

/*******************************************************************************
 *
 ******************************************************************************/
TrajectoryND::TrajectoryND() : active(false), blending(1.0),
  continuousActivation(0.0)
{
}

/*******************************************************************************
 * Copy constructor doing deep copying.
 ******************************************************************************/
TrajectoryND::TrajectoryND(const TrajectoryND& copyFromMe) :
  active(copyFromMe.active), blending(copyFromMe.blending),
  continuousActivation(copyFromMe.continuousActivation), name(copyFromMe.name),
  x_curr(copyFromMe.x_curr), x_prev(copyFromMe.x_prev), x_pprev(copyFromMe.x_pprev),
  x_ppprev(copyFromMe.x_ppprev)
{

  for (size_t i=0; i<copyFromMe.trajVec1D.size(); ++i)
  {
    trajVec1D.push_back(copyFromMe.trajVec1D[i]->clone());
  }

  for (size_t i=0; i<copyFromMe.activation.size(); ++i)
  {
    activation.push_back(std::make_shared<ActivationPoint>(copyFromMe.activation[i]->getTime(),
                                                           copyFromMe.activation[i]->switchesOn(),
                                                           copyFromMe.activation[i]->getHorizon()));
  }

}

/*******************************************************************************
 * Copy constructor doing deep copying.
 ******************************************************************************/
TrajectoryND& TrajectoryND::operator=(const TrajectoryND& copyFromMe)
{
  //RLOG(0, "=== TrajectoryND");
  if (this == &copyFromMe)
  {
    return *this;
  }

  for (size_t i=0; i<copyFromMe.trajVec1D.size(); ++i)
  {
    this->trajVec1D.push_back(copyFromMe.trajVec1D[i]->clone());
  }

  this->active = copyFromMe.active;
  this->blending = copyFromMe.blending;
  this->continuousActivation = copyFromMe.continuousActivation;

  return *this;
}

/*******************************************************************************
 *
 ******************************************************************************/
TrajectoryND::~TrajectoryND()
{
  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    delete trajVec1D[i];
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int TrajectoryND::getDim() const
{
  return trajVec1D.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int TrajectoryND::getInternalDim() const
{
  return trajVec1D.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
double TrajectoryND::getContinuousActivation() const
{
  double s = Math_clip(this->continuousActivation, 0.0, 1.0);

  return 6.0*pow(s,5) - 15.0*pow(s, 4) + 10.0*pow(s, 3);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::step(double dt)
{
  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->step(dt);
  }

  // Shift all activation points backwards in time,same procedure as above.
  for (int i=activation.size()-1; i>=0; i--)
  {
    activation[i]->shiftTime(-dt);

    if ((activation[i]->getTime()<0.0) && (activation[i]->getTime()>-dt-1.0e-6))
    {
      this->active = activation[i]->switchesOn();
    }

    // Compute continuous activation in period after the activation point. It
    // has an effect on the activation in the horizon after the activation time.
    if ((activation[i]->getTime() <= 0.0) &&
        (activation[i]->getTime() > -activation[i]->getHorizon()))
    {
      // Activation point affects continuous activation
      if (this->active)
      {
        continuousActivation += dt/activation[i]->getHorizon();
      }
      else
      {
        continuousActivation -= dt/activation[i]->getHorizon();
      }

      continuousActivation = Math_clip(continuousActivation, 0.0, 1.0);
    }

    // If an activation point falls out of time, we delete it.
    if (activation[i]->getTime()<=-activation[i]->getHorizon()-1.0e-6)
    {
      std::shared_ptr<ActivationPoint> iPtr = *(activation.begin()+i);
      activation.erase(activation.begin()+i);
    }
  }


  this->blending = 1.0;

  for (size_t i=0; i<activation.size(); ++i)
  {
    this->blending = fmin(this->blending, activation[i]->computeBlending());
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::computeTrajectory(MatNd* traj,
                                     double t0,
                                     double t1,
                                     double dt)
{
  if (dt<=0.0)
  {
    RLOG(1, "dt must be > 0.0, but is %g", dt);
    if (traj != NULL)
    {
      traj->m = 0;
    }
    return;
  }

  const int nSteps = lround((t1-t0)/dt);

  if (nSteps==0)
  {
    if (traj != NULL)
    {
      traj->m = 0;
    }
    return;
  }

  traj = MatNd_realloc(traj, nSteps+1, trajVec1D.size());


  for (int i=0; i<nSteps+1; ++i)
  {
    for (size_t rows=0; rows<trajVec1D.size(); ++rows)
    {
      const double t = (double)(t1-t0)*i/nSteps+t0;
      double xt = trajVec1D[rows]->getPosition(t);
      MatNd_set(traj, i, rows, xt);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::computeVelocityTrajectory(MatNd* traj,
                                             double t0,
                                             double t1,
                                             double dt)
{
  int nSteps = lround((t1 - t0) / dt);
  traj = MatNd_realloc(traj, nSteps + 1, trajVec1D.size());

  for (int i = 0; i<nSteps + 1; i++)
  {
    for (size_t rows = 0; rows<trajVec1D.size(); ++rows)
    {
      const double t = (double)(t1 - t0)*i / nSteps + t0;
      double x=0.0, xd=0.0, xdd=0.0;
      trajVec1D[rows]->computeTrajectoryPoint(x, xd, xdd, t);
      MatNd_set(traj, i, rows, xd);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::computeAccelerationTrajectory(MatNd* traj,
                                                 double t0,
                                                 double t1,
                                                 double dt)
{
  int nSteps = lround((t1 - t0) / dt);
  traj = MatNd_realloc(traj, nSteps + 1, trajVec1D.size());

  for (int i = 0; i<nSteps + 1; i++)
  {
    for (size_t rows = 0; rows<trajVec1D.size(); ++rows)
    {
      const double t = (double)(t1 - t0)*i / nSteps + t0;
      double x=0.0, xd=0.0, xdd=0.0;
      trajVec1D[rows]->computeTrajectoryPoint(x, xd, xdd, t);
      MatNd_set(traj, i, rows, xdd);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::getPosition(double* pos) const
{
  double* internalCoords = RNSTALLOC(getInternalDim(), double);

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    internalCoords[i] = trajVec1D[i]->getPosition();
  }

  fromInternalCoords(pos, internalCoords);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::getPosition(double t, double* pos) const
{
  double* internalCoords = RNSTALLOC(getInternalDim(), double);

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    internalCoords[i] = trajVec1D[i]->getPosition(t);
  }

  fromInternalCoords(pos, internalCoords);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::getGoalPosition(size_t idx, double* pos)
{
  double* internalCoords = RNSTALLOC(getInternalDim(), double);

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    internalCoords[i] = trajVec1D[i]->getGoalPosition(idx);
  }

  fromInternalCoords(pos, internalCoords);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::getViaPosition(size_t idx, double* pos)
{
  double* internalCoords = RNSTALLOC(getInternalDim(), double);

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    internalCoords[i] = trajVec1D[i]->getViaPosition(idx);
  }

  fromInternalCoords(pos, internalCoords);
}

/*******************************************************************************
 *
 ******************************************************************************/
double TrajectoryND::getGoalTime(size_t idx)
{
  if (trajVec1D.empty())
  {
    return -1.0;
  }

  double goalTime = trajVec1D[0]->getGoalTime(idx);

  for (size_t i=1; i<trajVec1D.size(); ++i)
  {
    double tmp = trajVec1D[i]->getGoalTime(idx);
    if (tmp != goalTime)
    {
      return -1.0;
    }
  }

  return goalTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int TrajectoryND::getNumberOfConstraints() const
{
  unsigned int nc = 0;

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    nc += trajVec1D[i]->getNumberOfConstraints();
  }

  return nc;
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t TrajectoryND::getNumberOfActivationPoints() const
{
  return activation.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::addConstraint(double t, const double* x,
                                 const double* x_dot, const double* x_ddot,
                                 int flag)
{
  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->addConstraint(t,
                                x ? x[i] : 0.0,
                                x_dot ? x_dot[i] : 0.0,
                                x_ddot ? x_ddot[i] : 0.0,
                                flag);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool TrajectoryND::addActivationPoint(std::shared_ptr<ActivationPoint> point)
{
  RCHECK(point);

  int overwrite = 0;

  for (size_t i=0; i<activation.size(); ++i)
  {
    if (point->getTime() == activation[i]->getTime())
    {
      activation[i]->setSwitchOn(point->switchesOn());
      activation[i]->setHorizon(point->getHorizon());
      overwrite++;
    }
  }

  if (overwrite==0)
  {
    activation.push_back(point);
    return true;
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
double TrajectoryND::getBlending() const
{
  return this->blending;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool TrajectoryND::isActive() const
{
  return this->active;
}

/*******************************************************************************
*
******************************************************************************/
void TrajectoryND::removeConstraintsAfter(double t)
{
  for (size_t i = 0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->removeConstraintsAfter(t);
  }
}

/*******************************************************************************
*
******************************************************************************/
void TrajectoryND::removeConstraintsAfterFirstGoal()
{
  for (size_t i = 0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->removeConstraintsAfterFirstGoal();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::init(const double* x)
{
  double* internalCoords = RNSTALLOC(getInternalDim(), double);
  toInternalCoords(internalCoords, x);

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->init(internalCoords[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::clear(bool clearActivations)
{
  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->clear();
  }

  if (clearActivations)
  {
    activation.clear();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double TrajectoryND::getTimeOfLastGoal() const
{
  if (trajVec1D.empty())
  {
    return 0.0;
  }

  bool hasGoals = false;
  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    if (trajVec1D[i]->getNumberOfGoals() > 0)
    {
      hasGoals = true;
    }
  }

  if (hasGoals==false)
  {
    return 0.0;
  }


  double goalTime = 0.0;

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    unsigned int lastGoalIdx = trajVec1D[i]->getNumberOfGoals();
    goalTime = std::max(trajVec1D[i]->getGoalTime(lastGoalIdx-1), goalTime);
  }

  return goalTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
double TrajectoryND::getTimeOfFirstGoal() const
{
  if (trajVec1D.empty())
  {
    return 0.0;
  }

  bool hasGoals = false;
  double goalTime = 0.0;

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    if (trajVec1D[i]->getNumberOfGoals() > 0)
    {
      hasGoals = true;
      goalTime = trajVec1D[i]->getGoalTime(0);
      break;
    }
  }

  if (hasGoals==false)
  {
    return 0.0;
  }


  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    if (trajVec1D[i]->getNumberOfGoals()>0)
    {
      goalTime = std::min(trajVec1D[i]->getGoalTime(0), goalTime);
    }

    goalTime = std::min(trajVec1D[i]->getHorizonTime(), goalTime);
  }

  return goalTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::getPositionConstraintBefore(double* x, double t) const
{
  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    x[i] = trajVec1D[i]->getPositionConstraintBefore(t);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::print() const
{
  std::cout << "TrajectoryND: \"" << getName() << "\" with " << trajVec1D.size()
            << " sub-trajectories" << std::endl;

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->print();
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Trajectory1D* TrajectoryND::operator [](int idx)
{
  return this->trajVec1D[idx];
}

/*******************************************************************************
 *
 ******************************************************************************/
Trajectory1D* TrajectoryND::getTrajectory1D(int idx) const
{
  return this->trajVec1D[idx];
}

/*******************************************************************************
 *
 ******************************************************************************/
const Trajectory1D* TrajectoryND::operator [](int idx) const
{
  return this->trajVec1D[idx];
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::toInternalCoords(double* internalCoords,
                                    const double* x)  const
{
  VecNd_copy(internalCoords, x, getDim());
}

/*******************************************************************************
 *
 ******************************************************************************/
bool TrajectoryND::fromInternalCoords(double* x,
                                      const double* internalCoords) const
{
  VecNd_copy(x, internalCoords, getDim());
  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::addTrajectory1D(Trajectory1D* traj1D)
{
  this->trajVec1D.push_back(traj1D);
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string TrajectoryND::getTypeName() const
{
#if defined (_MSC_VER)
  std::string demStr = typeid(*this).name();
#else
  char* demangled = __cxa_demangle(typeid(*this).name(), NULL, 0, NULL);
  std::string demStr = demangled;
  free(demangled);
#endif

  return demStr;
}

/*******************************************************************************
 *
 ******************************************************************************/
const std::string& TrajectoryND::getName() const
{
  return this->name;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryND::setName(std::string newName)
{
  this->name = newName;

  for (size_t i=0; i<trajVec1D.size(); ++i)
  {
    trajVec1D[i]->setName(newName + std::string(" ") + std::to_string(i));
  }
}

}   // namespace tropic
