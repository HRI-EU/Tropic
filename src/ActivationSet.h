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

#ifndef TROPIC_ACTIVATIONSET_H
#define TROPIC_ACTIVATIONSET_H

#include "ConstraintSet.h"


namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for organizing activation points.
 *
 *         The class is a TrajectoryConstraintSet with an additional vector of
 *         activation points. It allows to additionally organize how tasks
 *         are activated and deactivated over time.
 */
class ActivationSet : public ConstraintSet
{
public:

  ActivationSet();
  ActivationSet(const ActivationSet& other);
  ActivationSet(xmlNode* node);
  virtual ~ActivationSet();
  virtual ActivationSet* clone() const;

  virtual void apply(std::vector<TrajectoryND*>& trajectory,
                     std::map<std::string, Trajectory1D*>& tMap,
                     bool permissive) override;

  virtual void clear();

  virtual void addActivation(std::shared_ptr<ActivationPoint> aPt,
                             std::string trajNameND);

  virtual void addActivation(double time, bool switchOn, double horizon,
                             std::string trajNameND);

  virtual void fromXML(xmlNode* node);
  virtual bool toXML(std::string fileName) const;
  virtual void toXML(std::ostream& outStream, size_t indent) const;
  virtual bool isEqual(const ConstraintSet& other) const;
  virtual double compute(double dt);
  virtual double getEndTime() const;
  virtual bool inUse() const;
  virtual double getStartTimeRecurse() const;

private:

  struct NamedActivationPoint
  {
    NamedActivationPoint(std::shared_ptr<ActivationPoint> constraint,
                         const std::string& trajName) :
      c(constraint), trajNameND(trajName)
    {
    }

    bool operator == (const NamedActivationPoint& other) const
    {
      if (trajNameND != other.trajNameND)
      {
        return false;
      }

      return *(c.get()) == *(other.c.get());
    }

    std::shared_ptr<ActivationPoint> c;
    std::string trajNameND;
  };

protected:


  std::vector<NamedActivationPoint> aVec;
};


}   // namespace tropic




#endif // TROPIC_ACTIVATIONSET_H
