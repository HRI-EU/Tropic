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

#include "ActivationPoint.h"

#include <Rcs_basicMath.h>
#include <Rcs_macros.h>

#include <cmath>



namespace tropic
{

ActivationPoint::ActivationPoint(double time, bool switchOn_,
                                 double horizon_) :
  t(time), switchOn(switchOn_), horizon(horizon_)
{
}

ActivationPoint::~ActivationPoint()
{
}

bool ActivationPoint::operator == (const ActivationPoint& other) const
{
  return (t==other.t) && (switchOn==other.switchOn) && (horizon==other.horizon);
}

ActivationPoint* ActivationPoint::clone() const
{
  return new ActivationPoint(t, switchOn, horizon);
}

double ActivationPoint::getTime() const
{
  return this->t;
}

bool ActivationPoint::switchesOn() const
{
  return this->switchOn;
}

double ActivationPoint::getHorizon() const
{
  return this->horizon;
}

void ActivationPoint::shiftTime(double dt)
{
  this->t += dt;
}

double ActivationPoint::computeBlending() const
{
  if (this->horizon==0.0)
  {
    return 1.0;
  }

  // Triangular blending
  double blending = Math_clip(fabs(this->t) / this->horizon, 0.0, 1.0);

  // Bassin-style blending - to do

  return blending;
}

void ActivationPoint::print() const
{
  RLOG(0, "Activation ppint at t=%f with horizon %f switches %s",
       t, horizon, switchOn ? "ON" : "OFF");
}

void ActivationPoint::setSwitchOn(bool newSwitchOn)
{
  this->switchOn = newSwitchOn;
}

void ActivationPoint::setHorizon(double newHorizon)
{
  this->horizon = newHorizon;
}

} // namespace tropic
