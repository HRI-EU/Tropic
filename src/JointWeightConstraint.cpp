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


#include "JointWeightConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_body.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>

#include <algorithm>


namespace tropic
{
REGISTER_CONSTRAINT(JointWeightConstraint);

JointWeightConstraint::JointWeightConstraint(double t, const std::string& jointName_,
                                             double jointWeight_) :
  GraphConstraint(), jointName(jointName_), jointWeight(jointWeight_), jointLimitWeight(-1.0), changeTime(t), active(true)
{
  setClassName("JointWeightConstraint");
}

JointWeightConstraint::JointWeightConstraint(double t, const std::string& jointName_,
                                             double jointWeight_, double jointLimitWeight_) :
  GraphConstraint(), jointName(jointName_), jointWeight(jointWeight_), jointLimitWeight(jointLimitWeight_), changeTime(t), active(true)
{
  setClassName("JointWeightConstraint");
}

JointWeightConstraint::JointWeightConstraint(xmlNode* node) :
  GraphConstraint(), changeTime(0.0), jointWeight(1.0), jointLimitWeight(1.0), active(true)
{
  setClassName("JointWeightConstraint");
  fromXML(node);
}

JointWeightConstraint::JointWeightConstraint(const JointWeightConstraint& other) :
  GraphConstraint(other), jointName(other.jointName), changeTime(other.changeTime),
  jointWeight(other.jointWeight), active(other.active)
{
}

JointWeightConstraint* JointWeightConstraint::clone() const
{
  JointWeightConstraint* tSet = new JointWeightConstraint(changeTime, jointName, jointWeight);
  tSet->constraint = constraint;
  tSet->className = className;

  for (size_t i = 0; i < children.size(); ++i)
  {
    tSet->add(std::shared_ptr<ConstraintSet>(children[i]->clone()));
  }

  return tSet;
}

JointWeightConstraint::~JointWeightConstraint()
{
}

double JointWeightConstraint::compute(double dt)
{
  changeTime -= dt;
  //RLOG(0, "attachTime is %f", attachTime);

  if ((changeTime<0.0) && (changeTime>=-dt))
  {
    RLOG(5, "Changing joint weight of \"%s\" to %f", jointName.c_str(), jointWeight);

    RcsJoint* jnt = RcsGraph_getJointByName(graph, jointName.c_str());
    RCHECK_MSG(jnt, "%s", jointName.c_str());
    jnt->weightMetric = jointWeight;

    if (jointLimitWeight!=-1.0)
    {
      jnt->weightJL = jointLimitWeight;
    }

    this->active = false;
  }

  return GraphConstraint::compute(dt);
}

bool JointWeightConstraint::inUse() const
{
  return this->active;
}

double JointWeightConstraint::getStartTimeRecurse() const
{
  double startTime = ConstraintSet::getStartTimeRecurse();
  startTime = std::min(changeTime, startTime);
  return startTime < 0.0 ? 0.0 : startTime;
}

double JointWeightConstraint::getEndTime() const
{
  return std::max(changeTime, ConstraintSet::getEndTime());
}

void JointWeightConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  bool success = Rcs::getXMLNodePropertySTLString(node, "joint", jointName);
  success = getXMLNodePropertyDouble(node, "t", &changeTime) && success;
  success = getXMLNodePropertyDouble(node, "weightMetric", &jointWeight) && success;
  success = getXMLNodePropertyDouble(node, "weightJL", &jointLimitWeight) && success;

  active = (changeTime>0.0) ? true : false;

  node = node->children;

  while (node)
  {
    add(ConstraintFactory::create(node));
    node = node->next;
  }

}

void JointWeightConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\"" << getClassName() << "\" ";

  // Write out information to top-level tag
  outStream << "t=\"" << changeTime << "\" ";
  outStream << "joint=\"" << jointWeight << "\" ";
  outStream << "weightMetric=\"" << jointWeight << "\" ";

  if (jointLimitWeight != -1.0)
  {
    outStream << "weightJL=\"" << jointLimitWeight << "\" ";
  }

  // If there are no children, we close the tag in the first line
  if (children.empty())
  {
    outStream << " />" << std::endl;
  }
  // Go recursively through all child sets if any
  else
  {
    outStream << " >" << std::endl << std::endl;

    for (size_t i=0; i< children.size(); ++i)
    {
      children[i]->toXML(outStream, indent+2);
    }
    outStream << indStr << "</ConstraintSet>" << std::endl << std::endl;
  }

}



}   // namespace tropic
