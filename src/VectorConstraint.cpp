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

#include "VectorConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>
#include <Rcs_basicMath.h>

#include <string>



namespace tropic
{
REGISTER_CONSTRAINT(VectorConstraint);

VectorConstraint::VectorConstraint() : ConstraintSet()
{
  setClassName("VectorConstraint");
}

VectorConstraint::VectorConstraint(xmlNode* node) : ConstraintSet(node)
{
  setClassName("VectorConstraint");
  fromXML(node);
}

VectorConstraint::VectorConstraint(double t, const std::vector<double>& pos,
                                   const std::string& trajNameND, int flag) :
  ConstraintSet()
{
  setClassName("VectorConstraint");

  for (size_t i=0; i<pos.size(); ++i)
  {
    add(t, pos[i], 0.0, 0.0, flag, trajNameND + " " + std::to_string(i));
  }
}

VectorConstraint::~VectorConstraint()
{
}

VectorConstraint* VectorConstraint::clone() const
{
  VectorConstraint* tSet = new VectorConstraint();
  tSet->constraint = constraint;
  tSet->className = className;

  for (size_t i = 0; i < children.size(); ++i)
  {
    auto child = children[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}

std::vector<double> VectorConstraint::getPosition() const
{
  std::vector<double> pos;

  for (size_t i=0; i<numConstraints(false); ++i)
  {
    pos.push_back(getConstraint(i)->getPosition());
  }

  return pos;
}

void VectorConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  double t;
  size_t flag = 0, dim = 0;
  std::string tName;
  bool success = Rcs::getXMLNodePropertySTLString(node, "trajectory", tName);
  success = getXMLNodePropertyDouble(node, "t", &t) && success;

  std::vector<double> pos = Rcs::getXMLNodePropertyVecSTLDouble(node, "pos");
  if (!pos.empty())
  {
    flag += 1;
    dim = pos.size();
  }

  std::vector<double> vel = Rcs::getXMLNodePropertyVecSTLDouble(node, "vel");
  if (!vel.empty())
  {
    flag += 2;
    dim = vel.size();

    if ((!pos.empty()) && (vel.size()!=pos.size()))
    {
      success = false;
    }
  }

  std::vector<double> acc = Rcs::getXMLNodePropertyVecSTLDouble(node, "acc");
  if (!acc.empty())
  {
    flag += 4;
    dim = acc.size();

    if ((!pos.empty()) && (acc.size()!=pos.size()))
    {
      success = false;
    }
    if ((!vel.empty()) && (acc.size()!=vel.size()))
    {
      success = false;
    }
  }

  // If none of pos, vel or acc are given, there's no information on the vector
  // dimensionality and we treat it as an error.
  if (flag==0)
  {
    success = false;
  }

  if (success)
  {
    for (size_t i=0; i<dim; ++i)
    {
      add(t, pos.empty() ? 0.0 : pos[i],
          vel.empty() ? 0.0 : vel[i],
          acc.empty() ? 0.0 : acc[i],
          flag, tName + " " + std::to_string(i));
    }
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

void VectorConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\"" << getClassName() << "\" ";

  // Check that we have constraints
  RCHECK_MSG(!constraint.empty(), "VectorConstraint has no constraints");

  // Write out information to top-level tag
  outStream << "t=\"" << constraint[0].c->getTime() << "\" ";

  // Write positions if corresponding flag is set
  const int flag = constraint[0].c->getFlag();

  if (Math_isBitSet(flag, 0))
  {
    outStream << "pos=\"";

    for (size_t i=0; i<constraint.size(); ++i)
    {
      outStream << constraint[i].c->getPosition();

      if (i!=constraint.size()-1)
      {
        outStream << " ";
      }
    }

    outStream << "\" ";
  }

  // Write velocities if corresponding flag is set
  if (Math_isBitSet(flag, 1))
  {
    outStream << "vel=\"";

    for (size_t i=0; i<constraint.size(); ++i)
    {
      outStream << constraint[i].c->getVelocity();

      if (i!=constraint.size()-1)
      {
        outStream << " ";
      }
    }

    outStream << "\" ";
  }

  // Write accelerations if corresponding flag is set
  if (Math_isBitSet(flag, 2))
  {
    outStream << "acc=\"";

    for (size_t i=0; i<constraint.size(); ++i)
    {
      outStream << constraint[i].c->getAcceleration();

      if (i!=constraint.size()-1)
      {
        outStream << " ";
      }
    }

    outStream << "\" ";
  }

  char tmp[256];
  bool ok = String_removeSuffix(tmp, constraint[0].trajName1D.c_str(), ' ');
  RCHECK_MSG(ok, "Trajectory \"%s\" should end like \" 1\"", tmp);

  outStream << "trajectory=\"" << tmp << "\" ";

  // Add constraint ids
  outStream << getIdsForXML();

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
