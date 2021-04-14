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

#include <Rcs_basicMath.h>
#include <Rcs_Vec3d.h>
#include <Rcs_macros.h>
#include <Rcs_utils.h>

#include <algorithm>
#include <functional>
#include <iostream>


namespace tropic
{

/*******************************************************************************
 * Do some statistics on creation and deletion of constraints.
 ******************************************************************************/
size_t Constraint1D::constraintCount = 1;
static size_t constraintStats = 0;

static void myExit(void)
{
  if (constraintStats != 0)
  {
    RLOG_CPP(0, "Error: " << constraintStats
             << " constraints on exit - should be 0");
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
  constraintStats++;
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
  constraintStats++;
}

/*******************************************************************************
 *
 ******************************************************************************/
Constraint1D::Constraint1D(double t_,
                           double x_) :
  t(t_), x(x_), x_dot(0.0), x_ddot(0.0), flag(7), id(constraintCount++)
{
  constraintStats++;
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Constraint1D::Constraint1D(const std::shared_ptr<Constraint1D> copyFromMe) :
  t(copyFromMe->t), x(copyFromMe->x), x_dot(copyFromMe->x_dot),
  x_ddot(copyFromMe->x_ddot), flag(copyFromMe->flag), id(copyFromMe->id)
{
  constraintStats++;
}

/*******************************************************************************
 * Copy constructor
 ******************************************************************************/
Constraint1D::Constraint1D(const Constraint1D& copyFromMe) :
  t(copyFromMe.t), x(copyFromMe.x), x_dot(copyFromMe.x_dot),
  x_ddot(copyFromMe.x_ddot), flag(copyFromMe.flag), id(copyFromMe.id)
{
  constraintStats++;
}

/*******************************************************************************
 *
 ******************************************************************************/
Constraint1D::~Constraint1D()
{
  constraintStats--;
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
void Constraint1D::toDescriptor(MatNd* viaDesc, unsigned int row)
{
  MatNd_set(viaDesc, row, 0, this->t);
  MatNd_set(viaDesc, row, 1, this->x);
  MatNd_set(viaDesc, row, 2, this->x_dot);
  MatNd_set(viaDesc, row, 3, this->x_ddot);
  MatNd_set(viaDesc, row, 4, this->flag & 0x07); // Mask first 3 bits only

  if (viaDesc->n > 5)
  {
    MatNd_set(viaDesc, row, 5, round(this->id));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Constraint1D::check() const
{
  bool success = true;

  success = Math_isFinite(t) && success;
  success = Math_isFinite(x) && success;
  success = Math_isFinite(x_dot) && success;
  success = Math_isFinite(x_ddot) && success;

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
  return constraintStats;
}

}   // namespace tropic
