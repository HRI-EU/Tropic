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

#include "PositionConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>

#include <string>



namespace tropic
{
REGISTER_CONSTRAINT(PositionConstraint);

PositionConstraint::PositionConstraint() : ConstraintSet()
{
  setClassName("PositionConstraint");
}

PositionConstraint::PositionConstraint(xmlNode* node) : ConstraintSet(node)
{
  setClassName("PositionConstraint");
  fromXML(node);
}

PositionConstraint::PositionConstraint(double t, double x, double y, double z,
                                       const std::string& trajNameND,
                                       int flag) :
  ConstraintSet()
{
  setClassName("PositionConstraint");
  add(t, x, 0.0, 0.0, flag, trajNameND + " 0");
  add(t, y, 0.0, 0.0, flag, trajNameND + " 1");
  add(t, z, 0.0, 0.0, flag, trajNameND + " 2");
}

PositionConstraint::PositionConstraint(double t, const double I_r_IP[3],
                                       const std::string& trajNameND,
                                       int flag) :
  ConstraintSet()
{
  setClassName("PositionConstraint");
  add(t, I_r_IP[0], 0.0, 0.0, flag, trajNameND + " 0");
  add(t, I_r_IP[1], 0.0, 0.0, flag, trajNameND + " 1");
  add(t, I_r_IP[2], 0.0, 0.0, flag, trajNameND + " 2");
}

PositionConstraint::~PositionConstraint()
{
}

PositionConstraint* PositionConstraint::clone() const
{
  PositionConstraint* tSet = new PositionConstraint();
  tSet->constraint = constraint;
  tSet->className = className;

  for (size_t i = 0; i < set.size(); ++i)
  {
    auto child = set[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}

void PositionConstraint::getPosition(double I_pt[3])
{
  for (size_t i=0; i<numConstraints(false); ++i)
  {
    I_pt[i] = getConstraint(i)->getPosition();
  }
}

void PositionConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  double t, pos[3];
  std::string tName;
  bool success = Rcs::getXMLNodePropertySTLString(node, "trajectory", tName);
  success = getXMLNodePropertyDouble(node, "t", &t) && success;
  success = getXMLNodePropertyVec3(node, "pos", pos) && success;

  // If the flag is not given, we set it to the default (7: fully specified)
  int flag = 7;
  getXMLNodePropertyInt(node, "flag", &flag);

  if (success)
  {
    add(t, pos[0], 0.0, 0.0, flag, tName + " 0");
    add(t, pos[1], 0.0, 0.0, flag, tName + " 1");
    add(t, pos[2], 0.0, 0.0, flag, tName + " 2");
  }
  else
  {
    RLOG(1, "Failed to create constraint!");
  }

  node = node->children;

  while (node)
  {
    add(ConstraintFactory::create(node));
    node = node->next;
  }

}

void PositionConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\"" << getClassName() << "\" ";

  // Check that we have only 3 constraints

  if (constraint.empty())
  {
    throw (std::string("Found PositionConstraint with no constraints"));
  }
  else if (constraint.size()!=3)
  {
    std::string errMsg = "Trajectory \"" + constraint[0].trajName1D + " has "
                         + std::to_string(constraint.size())
                         + " dimensions, but is expected to have 3\n";
    throw (errMsg);
  }

  // Write out information to top-level tag
  outStream << "t=\"" << constraint[0].c->getTime() << "\" ";
  outStream << "pos=\""
            << constraint[0].c->getPosition() << " "
            << constraint[1].c->getPosition() << " "
            << constraint[2].c->getPosition() << "\" ";

  // Only write out flag if constraint is not fully specified
  if (constraint[0].c->getFlag() != 7)
  {
    outStream << "flag=\"" << constraint[0].c->getFlag() << "\" ";
  }

  char tmp[256];
  bool ok = String_removeSuffix(tmp, constraint[0].trajName1D.c_str(), ' ');

  if (!ok)
  {
    std::string errMsg = "Trajectory \"" + std::string(tmp) + " should end like \" 1\"\n";
    throw (errMsg);
  }

  outStream << "trajectory=\"" << tmp << "\"";

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
