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


#include "ConnectBodyConstraint.h"
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
REGISTER_CONSTRAINT(ConnectBodyConstraint);

ConnectBodyConstraint::ConnectBodyConstraint(double t, const std::string& child_,
                                             const std::string& parent_) :
  GraphConstraint(), childName(child_), parentName(parent_), attachTime(t), active(true)
{
  setClassName("ConnectBodyConstraint");
}

ConnectBodyConstraint::ConnectBodyConstraint(xmlNode* node) :
  GraphConstraint(), attachTime(0.0), active(true)
{
  setClassName("ConnectBodyConstraint");
  fromXML(node);
}

ConnectBodyConstraint::ConnectBodyConstraint(const ConnectBodyConstraint& other) :
  GraphConstraint(other), childName(other.childName), parentName(other.parentName),
  attachTime(other.attachTime), active(other.active)
{
}

ConnectBodyConstraint* ConnectBodyConstraint::clone() const
{
  ConnectBodyConstraint* tSet = new ConnectBodyConstraint(attachTime, childName, parentName);
  tSet->constraint = constraint;
  tSet->className = className;

  for (size_t i = 0; i < set.size(); ++i)
  {
    tSet->add(std::shared_ptr<ConstraintSet>(set[i]->clone()));
  }

  return tSet;
}

ConnectBodyConstraint::~ConnectBodyConstraint()
{
}

double ConnectBodyConstraint::compute(double dt)
{
  attachTime -= dt;
  //RLOG(0, "attachTime is %f", attachTime);

  if ((attachTime<0.0) && (attachTime>=-dt))
  {
    RLOG(0, "Appending \"%s\" to \"%s\"", childName.c_str(), parentName.c_str());

    RcsBody* child = RcsGraph_getBodyByName(graph, childName.c_str());
    RCHECK_MSG(child, "%s", childName.c_str());
    RcsBody* parent = RcsGraph_getBodyByName(graph, parentName.c_str());
    RCHECK_MSG(parent, "%s", parentName.c_str());
    bool success = RcsBody_attachToBodyId(graph, child->id, parent->id);
    RCHECK(success);
    this->active = false;
  }

  return GraphConstraint::compute(dt);
}

bool ConnectBodyConstraint::inUse() const
{
  return this->active;
}

double ConnectBodyConstraint::getStartTimeRecurse() const
{
  double startTime = ConstraintSet::getStartTimeRecurse();
  startTime = std::min(attachTime, startTime);
  return startTime < 0.0 ? 0.0 : startTime;
}

double ConnectBodyConstraint::getEndTime() const
{
  return std::max(attachTime, ConstraintSet::getEndTime());
}

void ConnectBodyConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  bool success = Rcs::getXMLNodePropertySTLString(node, "parent", parentName);
  success = Rcs::getXMLNodePropertySTLString(node, "child", childName) && success;
  success = getXMLNodePropertyDouble(node, "t", &attachTime) && success;

  active = (attachTime>0.0) ? true : false;

  node = node->children;

  while (node)
  {
    add(ConstraintFactory::create(node));
    node = node->next;
  }

}

void ConnectBodyConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\"" << getClassName() << "\" ";

  // Write out information to top-level tag
  outStream << "t=\"" << attachTime << "\" ";
  outStream << "parent=\"" << parentName << "\" ";
  outStream << "child=\"" << childName << "\" ";

  // If there are no children, we close the tag in the first line
  if (set.empty())
  {
    outStream << " />" << std::endl;
  }
  // Go recursively through all child sets if any
  else
  {
    outStream << " >" << std::endl << std::endl;

    for (size_t i=0; i<set.size(); ++i)
    {
      set[i]->toXML(outStream, indent+2);
    }
    outStream << indStr << "</ConstraintSet>" << std::endl << std::endl;
  }

}



}   // namespace tropic
