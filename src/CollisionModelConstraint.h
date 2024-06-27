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

#ifndef TROPIC_COLLISIONMODELCONSTRAINT_H
#define TROPIC_COLLISIONMODELCONSTRAINT_H

#include <GraphConstraint.h>


namespace tropic
{

class CollisionModelConstraint : public GraphConstraint
{
public:

  CollisionModelConstraint(double t, const std::string& bdyName, bool switchOn);
  CollisionModelConstraint(double t, std::vector<std::string> names, bool switchOn);
  CollisionModelConstraint(const CollisionModelConstraint& other);
  virtual CollisionModelConstraint* clone() const;
  virtual ~CollisionModelConstraint();
  virtual double compute(double dt);
  bool inUse() const;
  double getStartTimeRecurse() const;
  double getEndTime() const;
  void fromXML(xmlNode* node);
  void toXML(std::ostream& out, size_t indent = 0) const;

protected:

  std::vector<std::string> bdyNames;
  double toggleTime;
  bool switchesOn;
  bool active;
};


}   // namespace tropic





#endif   // TROPIC_COLLISIONMODELCONSTRAINT_H
