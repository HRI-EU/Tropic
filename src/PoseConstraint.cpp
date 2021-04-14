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

#include "PoseConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>



namespace tropic
{
REGISTER_CONSTRAINT(PoseConstraint);


PoseConstraint::PoseConstraint() : ConstraintSet()
{
  setClassName("PoseConstraint");
}

PoseConstraint::PoseConstraint(xmlNode* node) : ConstraintSet(node)
{
  setClassName("PoseConstraint");

  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  double t, pos[6];
  std::vector<std::string> tName;

  Rcs::getXMLNodePropertyVecSTLString(node, "trajectory", tName);
  bool success = (tName.size()==2) ? true : false;
  success = getXMLNodePropertyDouble(node, "t", &t) && success;
  success = getXMLNodePropertyVecN(node, "pos", pos, 6) && success;
  Vec3d_constMulSelf(&pos[3], M_PI/180.0);   // Convert angles from degrees

  // If the flag is not given, we set it to the default (7: fully specified)
  int flag = 7;
  getXMLNodePropertyInt(node, "flag", &flag);

  if (success)
  {
    add(std::make_shared<PositionConstraint>(t, pos[0], pos[1], pos[2], tName[0], flag));
    add(std::make_shared<EulerConstraint>(t, &pos[3], tName[1]));
    RCHECK_MSG(flag==7, "No constructor for EulerConstraint with explicit flag");
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

PoseConstraint::PoseConstraint(double t, const HTr* A_PI, const std::string& trajNamePos,
                               const std::string& trajNameOri) : ConstraintSet()
{
  setClassName("PoseConstraint");
  double ea[3];
  Mat3d_toEulerAngles(ea, (double(*)[3]) A_PI->rot);
  add(std::make_shared<PositionConstraint>(t, A_PI->org, trajNamePos));
  add(std::make_shared<EulerConstraint>(t, ea, trajNameOri));
}

PoseConstraint::PoseConstraint(double t, const HTr* A_BI, const HTr* A_PI,
                               const std::string& trajNamePos,
                               const std::string& trajNameOri) : ConstraintSet()
{
  setClassName("PoseConstraint");
  double ea[3];
  Mat3d_toEulerAngles(ea, (double(*)[3]) A_PI->rot);
  add(std::make_shared<PositionConstraint>(t, A_BI, A_PI->org, trajNamePos));
  add(std::make_shared<EulerConstraint>(t, A_BI, ea, trajNameOri));
}

PoseConstraint::~PoseConstraint()
{
}

PoseConstraint* PoseConstraint::clone()
{
  PoseConstraint* tSet = new PoseConstraint();
  tSet->constraint = constraint;
  tSet->className = className;

  for (size_t i = 0; i < set.size(); ++i)
  {
    auto child = set[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}

void PoseConstraint::print() const
{
  printf("%s (%s): ", getClassName().c_str(), getTypeName().c_str());
  printf("t=%.3f ", set[0]->getConstraint(0)->getTime());
  printf("pos=%.3f %.3f %.3f ",
         set[0]->getConstraint(0)->getPosition(),
         set[0]->getConstraint(1)->getPosition(),
         set[0]->getConstraint(2)->getPosition());
  double quat[4];
  quat[0] = set[1]->getConstraint(0)->getPosition();
  quat[1] = set[1]->getConstraint(1)->getPosition();
  quat[2] = set[1]->getConstraint(2)->getPosition();
  quat[3] = set[1]->getConstraint(3)->getPosition();
  double rm[3][3], ea[3];
  Quat_toRotationMatrix(rm, quat);
  Mat3d_toEulerAngles(ea, rm);
  Vec3d_constMulSelf(ea, 180.0/M_PI);
  printf("ori =%.1f %.1f %.1f deg", ea[0], ea[1], ea[2]);
  printf("\n");

  // No further recursion
}

void PoseConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\""
            << getClassName() << "\" ";

  PositionConstraint* posSet = dynamic_cast<PositionConstraint*>(getSet(0).get());
  EulerConstraint* oriSet    = dynamic_cast<EulerConstraint*>(getSet(1).get());
  RCHECK(posSet);
  RCHECK(oriSet);

  // Convert internal quaternion to Euler angles. These go into the xml file.
  double quat[4], ea[3], len;
  oriSet->getQuaternion(quat);
  len = VecNd_normalizeSelf(quat, 4);
  RCHECK_MSG(len>0.0, "Couldn't normalize quaternion");
  Quat_toEulerAngles(ea, quat);
  Vec3d_constMulSelf(ea, 180.0/M_PI);   // Write angles in degrees

  // Write out information to top-level tag
  outStream << "t=\"" << posSet->getConstraint(0)->getTime() << "\" ";
  outStream << "pos=\""
            << posSet->getConstraint(0)->getPosition() << " "
            << posSet->getConstraint(1)->getPosition() << " "
            << posSet->getConstraint(2)->getPosition() << " "
            << ea[0] << " "<< ea[1] << " " << ea[2] << "\" ";

  // Only write out flag if constraint is not fully specified
  const int flag = posSet->getConstraint(0)->getFlag();
  if (flag != 7)
  {
    outStream << "flag=\"" << flag << "\" ";
  }

  char tmp[256];
  bool ok = String_removeSuffix(tmp, posSet->getTrajectoryName(0).c_str(), ' ');
  RCHECK_MSG(ok, "Trajectory \"%s\" should end like \" 1\"", tmp);

  outStream << "trajectory=\"" << tmp << " " << oriSet->getTrajectoryName() << "\"";

  // If there are no children, we close the tag in the first line
  if (set.size()==2)
  {
    outStream << " />" << std::endl;
  }
  // Go recursively through all child sets if any
  else
  {
    outStream << " >" << std::endl << std::endl;

    for (size_t i=2; i<set.size(); ++i)
    {
      set[i]->toXML(outStream, indent+2);
    }
    outStream << indStr << "</ConstraintSet>" << std::endl << std::endl;
  }

}

}   // namespace Rcs
