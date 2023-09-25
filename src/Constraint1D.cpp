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

#include "Constraint1D.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <cmath>
#include <mutex>

static std::mutex constraintStatsMtx;


namespace tropic
{

/*******************************************************************************
 * Do some statistics on creation and deletion of constraints.
 ******************************************************************************/
size_t Constraint1D::constraintCount = 1;
static size_t constraintStats = 0;

static inline void incrementConstraintStats()
{
  std::lock_guard<std::mutex> lock(constraintStatsMtx);
  constraintStats++;
}

static inline void decrementConstraintStats()
{
  std::lock_guard<std::mutex> lock(constraintStatsMtx);
  constraintStats--;
}

static inline size_t getConstraintStats()
{
  std::lock_guard<std::mutex> lock(constraintStatsMtx);
  return constraintStats;
}

static void myExit(void)
{
  if (constraintStats != 0)
  {
    std::cerr << "[" << __FILE__ << ": " << __FUNCTION__
              << "(" << __LINE__ << ")]: " << constraintStats
              << " constraints on exit - should be 0" << std::endl;
    std::cerr << "size_t max: " << std::numeric_limits<size_t>::max()
              << std::endl;
  }
}

class PreStartRoutine
{
public:

  PreStartRoutine()
  {
    atexit(myExit);
  }

};

static PreStartRoutine onDynamicLoad;


/*******************************************************************************
 *
 ******************************************************************************/
Constraint1D::Constraint1D() :
  t(0.0), x(0.0), x_dot(0.0), x_ddot(0.0), flag(0), id(constraintCount++)
{
  incrementConstraintStats();
}

/*******************************************************************************
 *
 ******************************************************************************/
Constraint1D::Constraint1D(double t_,
                           double x_,
                           double x_dot_,
                           double x_ddot_,
                           int flag_) :
  t(t_), x(x_), x_dot(x_dot_), x_ddot(x_ddot_), flag(flag_),
  id(constraintCount++)
{
  incrementConstraintStats();
}

/*******************************************************************************
 *
 ******************************************************************************/
Constraint1D::Constraint1D(double t_,
                           double x_) :
  t(t_), x(x_), x_dot(0.0), x_ddot(0.0), flag(7), id(constraintCount++)
{
  incrementConstraintStats();
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Constraint1D::Constraint1D(const std::shared_ptr<Constraint1D> copyFromMe) :
  t(copyFromMe->t), x(copyFromMe->x), x_dot(copyFromMe->x_dot),
  x_ddot(copyFromMe->x_ddot), flag(copyFromMe->flag), id(copyFromMe->id)
{
  incrementConstraintStats();
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Constraint1D::Constraint1D(const Constraint1D& copyFromMe) :
  t(copyFromMe.t), x(copyFromMe.x), x_dot(copyFromMe.x_dot),
  x_ddot(copyFromMe.x_ddot), flag(copyFromMe.flag), id(copyFromMe.id)
{
  incrementConstraintStats();
}

/*******************************************************************************
 *
 ******************************************************************************/
Constraint1D::~Constraint1D()
{
  if (getConstraintStats()==0)
  {
    std::cerr << "[" << __FILE__ << ": " << __FUNCTION__
              << "(" << __LINE__ << ")]: Constraint underflow with id "
              << id << std::endl;
  }

  decrementConstraintStats();
}

/*******************************************************************************
 * Pointer version of copy
 ******************************************************************************/
Constraint1D* Constraint1D::clone() const
{
  return new Constraint1D(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Constraint1D::operator == (const Constraint1D& other) const
{
  return (t==other.t) && (x==other.x) && (x_dot==other.x_dot) &&
         (x_ddot==other.x_ddot) && (flag==other.flag) && (id==other.id);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Constraint1D::operator != (const Constraint1D& other) const
{
  return (t!=other.t) || (x!=other.x) || (x_dot!=other.x_dot) ||
         (x_ddot!=other.x_ddot) || (flag!=other.flag);// || (id!=other.id);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Constraint1D::operator > (const Constraint1D& other) const
{
  return (t > other.t) ? true : false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Constraint1D::operator < (const Constraint1D& other) const
{
  return (t < other.t) ? true : false;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::ostream& operator<<(std::ostream& output,
                         const Constraint1D& via)
{
  output << "t=" << via.getTime()
         << " x=" << via.getPosition()
         << " x_dot=" << via.getVelocity()
         << " x_ddot=" << via.getAcceleration()
         << " flag=" << via.getFlag()
         << " id=" << via.getID();

  return output;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Constraint1D::check() const
{
  bool success = true;

  success = std::isfinite(t) && success;
  success = std::isfinite(x) && success;
  success = std::isfinite(x_dot) && success;
  success = std::isfinite(x_ddot) && success;

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::set(double t_, double x_, double x_dot_,
                       double x_ddot_, int flag_)
{
  this->t = t_;
  this->x = x_;
  this->x_dot = x_dot_;
  this->x_ddot = x_ddot_;
  this->flag = flag_;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Constraint1D::getTime() const
{
  return this->t;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Constraint1D::getPosition() const
{
  return this->x;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Constraint1D::getVelocity() const
{
  return this->x_dot;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Constraint1D::getAcceleration() const
{
  return this->x_ddot;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Constraint1D::getFlag() const
{
  return this->flag;
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t Constraint1D::getID() const
{
  return this->id;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::setTime(double t)
{
  this->t = t;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::setFlag(int newFlag)
{
  this->flag = newFlag;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::setPosition(double x_)
{
  this->x = x_;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::setVelocity(double x_dot_)
{
  this->x_dot = x_dot_;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::setAcceleration(double x_ddot_)
{
  this->x_ddot = x_ddot_;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::shiftTime(double dt)
{
  this->t += dt;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Constraint1D::assignUniqueID()
{
  this->id = constraintCount++;
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t Constraint1D::getNumConstraints()
{
  return getConstraintStats();
}

}   // namespace tropic
