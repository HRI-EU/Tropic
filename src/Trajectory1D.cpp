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

#include "Trajectory1D.h"

#include <Rcs_macros.h>

#include <algorithm>
#include <iostream>
#include <string>



namespace tropic
{

/*******************************************************************************
 *
 ******************************************************************************/
Trajectory1D::Trajectory1D() : horizon(1.0), viaNow(0.0, 0, 0.0, 0.0, 7)
{
  this->viaHorizon = new Constraint1D(horizon, 0.0, 0.0, 0.0, 7);
}

/*******************************************************************************
 *
 ******************************************************************************/
Trajectory1D::Trajectory1D(double x0, double horizon_) :
  horizon(horizon_), viaNow(0.0, x0, 0.0, 0.0, 7)
{
  RCHECK(this->horizon>0.0);
  this->viaHorizon = new Constraint1D(horizon_, x0, 0.0, 0.0, 7);
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Trajectory1D::Trajectory1D(const Trajectory1D& copyFromMe) :
  horizon(copyFromMe.horizon), viaNow(copyFromMe.viaNow), viaHorizon(NULL),
  name(copyFromMe.name)
{
  clearConstraints();

  if (copyFromMe.viaHorizon != NULL)
  {
    this->viaHorizon = copyFromMe.viaHorizon->clone();
  }

  for (size_t i=0; i<copyFromMe.viaGoal.size(); ++i)
  {
    viaGoal.push_back(std::make_shared<Constraint1D>(copyFromMe.viaGoal[i]));
  }

  for (size_t i=0; i<copyFromMe.intermediate.size(); ++i)
  {
    intermediate.push_back(std::make_shared<Constraint1D>(copyFromMe.intermediate[i]));
  }
}

/*******************************************************************************
 * Assignment convenience operator
 ******************************************************************************/
Trajectory1D& Trajectory1D::operator=(const Trajectory1D& copyFromMe)
{
  if (this == &copyFromMe)
  {
    return *this;
  }

  this->horizon = copyFromMe.horizon;
  this->viaNow = copyFromMe.viaNow;
  this->name = copyFromMe.name;

  clearConstraints();

  if (copyFromMe.viaHorizon != NULL)
  {
    this->viaHorizon = copyFromMe.viaHorizon->clone();
  }

  for (size_t i=0; i<copyFromMe.viaGoal.size(); ++i)
  {
    viaGoal.push_back(std::make_shared<Constraint1D>(copyFromMe.viaGoal[i]));
  }

  for (size_t i=0; i<copyFromMe.intermediate.size(); ++i)
  {
    intermediate.push_back(std::make_shared<Constraint1D>(copyFromMe.intermediate[i]));
  }

  return *this;
}

/*******************************************************************************
 *
 ******************************************************************************/
Trajectory1D::~Trajectory1D()
{
  clearConstraints();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::addConstraint(double t, double x, double x_dot,
                                 double x_ddot, int flag)
{
  addConstraint(std::make_shared<Constraint1D>(t, x, x_dot, x_ddot, flag));
}

/*******************************************************************************
 * Sorting of the constraints is done in increasing order with respect to their
 * time. This allows us to query the last time point with the back() function.
 ******************************************************************************/
void Trajectory1D::addConstraint(std::shared_ptr<Constraint1D> constraint)
{
  if ((constraint->getFlag()&0x07)==7)   // It's a goal point
  {
    viaGoal.push_back(constraint);
    std::sort(viaGoal.begin(), viaGoal.end(), Constraint1D::lesser_sptr());
  }
  else    // It's an intermediate point
  {
    intermediate.push_back(constraint);
    std::sort(intermediate.begin(), intermediate.end(),
              Constraint1D::lesser_sptr());
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::removeConstraintsAfter(double t)
{
  for (size_t i = 0; i < viaGoal.size(); ++i)
  {
    if (viaGoal[i]->getTime() > t)
    {
      viaGoal.erase(viaGoal.begin() + i, viaGoal.end());
      break;
    }
  }

  for (size_t i = 0; i < intermediate.size(); ++i)
  {
    if (intermediate[i]->getTime() > t)
    {
      intermediate.erase(intermediate.begin() + i, intermediate.end());
      break;
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::removeConstraintsAfterFirstGoal()
{
  double t_goal = viaGoal.empty() ? getHorizonTime() : viaGoal[0]->getTime();
  removeConstraintsAfter(t_goal+1.0e-3);
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Trajectory1D::getNumberOfGoals() const
{
  return viaGoal.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Trajectory1D::getNumberOfViaPoints() const
{
  return intermediate.size();
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getGoalPosition(size_t idx) const
{
  return viaGoal[idx]->getPosition();
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Trajectory1D::getNumberOfConstraints() const
{
  return getNumberOfGoals() + getNumberOfViaPoints();
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getHorizonTime() const
{
  return this->horizon;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getGoalTime(size_t idx) const
{
  double goalTime = 0.0;

  if (viaGoal.empty())
  {
    goalTime = 0.0;
  }
  else if (idx>=viaGoal.size())
  {
    goalTime = viaGoal.back()->getTime();
  }
  else
  {
    goalTime = viaGoal[idx]->getTime();
  }

  return goalTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getViaTime(size_t idx) const
{
  double viaTime = 0.0;

  if (intermediate.empty())
  {
    viaTime = 0.0;
  }
  else if (idx>=intermediate.size())
  {
    viaTime = intermediate.back()->getTime();
  }
  else
  {
    viaTime = intermediate[idx]->getTime();
  }

  return viaTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getViaPosition(size_t idx) const
{
  return intermediate[idx]->getPosition();
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getPositionConstraintBefore(double t) const
{
  double t_iter = -1.0, pos_t = getPosition(0.0);

  for (size_t i=0; i<viaGoal.size(); ++i)
  {
    double t_tmp = viaGoal[i]->getTime();
    if ((t_tmp<t) && (t_tmp>t_iter))
    {
      t_iter = t_tmp;
      pos_t = viaGoal[i]->getPosition();
    }
  }

  for (size_t i=0; i<intermediate.size(); ++i)
  {
    double t_tmp = intermediate[i]->getTime();
    if ((t_tmp<t) && (t_tmp>t_iter))
    {
      t_iter = t_tmp;
      pos_t = intermediate[i]->getPosition();
    }
  }

  if (viaHorizon != NULL)
  {
    double t_tmp = viaHorizon->getTime();
    if ((t_tmp<t) && (t_tmp>t_iter))
    {
      t_iter = t_tmp;
      pos_t = viaHorizon->getPosition();
    }
  }

  return pos_t;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::print() const
{
  std::cout << "Trajectory1D: \"" << getName() << "\"" << std::endl;
  std::cout << "Via now: " << viaNow << std::endl;

  if (!viaGoal.empty())
  {
    std::cout << viaGoal.size() << " goals: " << std::endl;
    for (size_t i=0; i<viaGoal.size(); ++i)
    {
      std::cout << *(viaGoal[i]) << std::endl;
    }
  }

  if (!intermediate.empty())
  {
    std::cout << intermediate.size() << " intermediates: " << std::endl;
    for (size_t i=0; i<intermediate.size(); ++i)
    {
      std::cout << *(intermediate[i]) << std::endl;
    }
  }

  if (viaHorizon != NULL)
  {
    std::cout << "1 horizon point: " << *viaHorizon << std::endl;
  }
  else
  {
    std::cout << "No horizon point";
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
std::shared_ptr<Constraint1D>  Trajectory1D::getGoalPtr(size_t idx)
{
  if (idx >= viaGoal.size())
  {
    RFATAL("Index out of range: %d >= %d", (int) idx, (int) viaGoal.size());
    return NULL;
  }

  return viaGoal[idx];
}

/*******************************************************************************
 *
 ******************************************************************************/
std::shared_ptr<Constraint1D> Trajectory1D::getViaPtr(size_t idx)
{
  if (idx >= intermediate.size())
  {
    RFATAL("Index out of range: %d >= %d",
           (int) idx, (int) intermediate.size());
    return NULL;
  }

  return intermediate[idx];
}

/*******************************************************************************
 * Return the horizon via point or NULL if none
 ******************************************************************************/
Constraint1D* Trajectory1D::getHorizonPtr()
{
  return this->viaHorizon;
}

/*******************************************************************************
 * Return the via point at t=0
 ******************************************************************************/
Constraint1D* Trajectory1D::getNowPtr()
{
  return &this->viaNow;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::init(double x)
{
  init(x, 0.0, 0.0);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::init(double x, double x_dot, double x_ddot)
{
  clearConstraints();
  viaNow.set(0.0, x, x_dot, x_ddot, 7);

  // clearConstraints() has deleted the viaHorizon point, sowe must recreate it
  this->viaHorizon = new Constraint1D(horizon, x, x_dot, x_ddot, 7);

  initFromConstraints();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::clear()
{
  clearConstraints();
  this->viaHorizon = new Constraint1D(horizon, viaNow.getPosition(), 0.0, 0.0, 7);
  initFromConstraints();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::clearConstraints()
{
  if (this->viaHorizon != NULL)
  {
    delete this->viaHorizon;
    this->viaHorizon = NULL;
  }

  viaGoal.clear();
  intermediate.clear();
}

/*******************************************************************************
 *
 ******************************************************************************/
std::ostream& operator<<(std::ostream& output, const Trajectory1D& via)
{
  output << "Initial point: " << via.viaNow << std::endl;

  if (!via.viaGoal.empty())
  {
    output << via.viaGoal.size() << " goal points: " << std::endl;

    for (size_t i=0; i<via.viaGoal.size(); ++i)
    {
      output << *(via.viaGoal[i]) << std::endl;
    }
  }

  if (!via.intermediate.empty())
  {
    output << via.intermediate.size() << " intermediate points: " << std::endl;

    for (size_t i=0; i<via.intermediate.size(); ++i)
    {
      output << *(via.intermediate[i]) << std::endl;
    }
  }

  if (via.viaHorizon != NULL)
  {
    output << "1 horizon point: " << *(via.viaHorizon) << std::endl;
  }

  return output;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::preStep(double dt)
{
  // Set first via point to the state at t0+dt. We keep the time and flag.
  double pos, vel, acc;
  computeTrajectoryPoint(pos, vel, acc, dt);
  viaNow.set(0.0, pos, vel, acc, viaNow.getFlag());

  // Shift all intermediate via points backwards in time
  for (int i=intermediate.size()-1; i>=0; i--)
  {
    intermediate[i]->shiftTime(-dt);

    // If an intermediate via point coincides with the first one, we simply
    // delete it. This loop goes backwards for that reason, otherwise the
    // vector indices would become inconsistent after a deletion.
    if (intermediate[i]->getTime()-viaNow.getTime()<TRAJECTORY1D_ALMOST_ZERO ||
        intermediate[i]->getFlag()==0)
    {
      intermediate.erase(intermediate.begin()+i);
    }
  }

  // Shift all goal points backwards in time,same procedure as above.
  for (int i=viaGoal.size()-1; i>=0; i--)
  {
    viaGoal[i]->shiftTime(-dt);

    // If a goal via point coincides with the first one, we delete it.
    if (viaGoal[i]->getTime()-viaNow.getTime()<TRAJECTORY1D_ALMOST_ZERO ||
        viaGoal[i]->getFlag()==0)
    {
      viaGoal.erase(viaGoal.begin()+i);
    }
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::step(double dt)
{
  preStep(dt);
  initFromConstraints();
}

/*******************************************************************************
 * Here we do this naiive traversal since find in shared_ptr does not compare
 * the raw pointer address.
 ******************************************************************************/
bool Trajectory1D::erase(std::shared_ptr<Constraint1D> c)
{
  for (auto it = viaGoal.begin(); it != viaGoal.end(); ++it)
  {
    if ((*it).get() == c.get())
    {
      viaGoal.erase(it);
      return true;
    }
  }

  for (auto it = intermediate.begin(); it != intermediate.end(); ++it)
  {
    if ((*it).get() == c.get())
    {
      intermediate.erase(it);
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getPosition() const
{
  return viaNow.getPosition();
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getVelocity() const
{
  return viaNow.getVelocity();
}

/*******************************************************************************
 *
 ******************************************************************************/
double Trajectory1D::getAcceleration() const
{
  return viaNow.getAcceleration();
}

/*******************************************************************************
 *
 ******************************************************************************/
const std::string& Trajectory1D::getName() const
{
  return this->name;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Trajectory1D::setName(std::string newName)
{
  this->name = newName;
}

}   // namespace tropic
