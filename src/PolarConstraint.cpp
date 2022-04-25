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

#include "PolarConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>

#include <algorithm>



namespace tropic
{
REGISTER_CONSTRAINT(PolarConstraint);

PolarConstraint::PolarConstraint() : ConstraintSet()
{
}

PolarConstraint::PolarConstraint(xmlNode* node) :
  ConstraintSet(node)
{
  setClassName("PolarConstraint");
  fromXML(node);
}

PolarConstraint::PolarConstraint(double t, double polarPhi, double polarTheta,
                                 const std::string& trajNameND) : ConstraintSet()
{
  setClassName("PolarConstraint");
  double polarAxis[3];
  Vec3d_getPolarAxis(polarAxis, polarPhi, polarTheta);

  add(t, polarAxis[0], trajNameND + " 0");
  add(t, polarAxis[1], trajNameND + " 1");
  add(t, polarAxis[2], trajNameND + " 2");
}

PolarConstraint::~PolarConstraint()
{
}

PolarConstraint* PolarConstraint::clone() const
{
  PolarConstraint* tSet = new PolarConstraint();
  tSet->constraint = constraint;
  tSet->className = className;

  for (size_t i = 0; i < children.size(); ++i)
  {
    auto child = children[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}

void PolarConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  double t, polarAngles[2];
  std::string tName;
  bool success = Rcs::getXMLNodePropertySTLString(node, "trajectory", tName);
  success = getXMLNodePropertyDouble(node, "t", &t) && success;
  success = getXMLNodePropertyVecN(node, "pos", polarAngles, 2) && success;
  Vec3d_constMulSelf(polarAngles, M_PI/180.0);   // Convert angles from degrees

  // If the flag is not given, we set it to the default (7: fully specified)
  int flag = 7;
  getXMLNodePropertyInt(node, "flag", &flag);

  if (success)
  {
    double polarAxis[3];
    Vec3d_getPolarAxis(polarAxis, polarAngles[0], polarAngles[1]);
    add(t, polarAxis[0], 0.0, 0.0, flag, tName + " 0");
    add(t, polarAxis[1], 0.0, 0.0, flag, tName + " 1");
    add(t, polarAxis[2], 0.0, 0.0, flag, tName + " 2");
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

void PolarConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Open set's xml description. The getClassName() function is polymorphic
  std::string indStr(indent, ' ');
  outStream << indStr << "<ConstraintSet type=\""
            << getClassName() << "\" ";

  // Convert internal Polar axis to Polar angles. These go into the xml file.
  double polarAxis[3], polarAngles[2], len;

  for (size_t i=0; i<3; ++i)
  {
    polarAxis[i] = getConstraint(i)->getPosition();
  }

  len = Vec3d_normalizeSelf(polarAxis);
  RCHECK_MSG(len>0.0, "Couldn't normalize Polar axis");
  Vec3d_getPolarAngles(polarAngles, polarAxis);
  VecNd_constMulSelf(polarAngles, 180.0/M_PI, 2);   // Write angles in degrees

  // Write out information to top-level tag
  outStream << "t=\"" << constraint[0].c->getTime() << "\" ";
  outStream << "pos=\""<< polarAngles[0] << " " << polarAngles[1] << "\" ";

  // Only write out flag if constraint is not fully specified
  if (constraint[0].c->getFlag() != 7)
  {
    outStream << "flag=\"" << constraint[0].c->getFlag() << "\" ";
  }

  char tmp[256] = "";
  bool ok = String_removeSuffix(tmp, constraint[0].trajName1D.c_str(), ' ');
  RCHECK_MSG(ok, "Trajectory \"%s\" should end like \" 1\"", tmp);

  outStream << "trajectory=\"" << tmp << "\"";

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
