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

#include "EulerConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>

#include <algorithm>



namespace tropic
{
REGISTER_CONSTRAINT(EulerConstraint);

EulerConstraint::EulerConstraint() :
  ConstraintSet(), A_BI(NULL), oriTrj(NULL)
{
  setClassName("EulerConstraint");
}

EulerConstraint::EulerConstraint(xmlNode* node) :
  ConstraintSet(node), A_BI(NULL), oriTrj(NULL)
{
  setClassName("EulerConstraint");
  fromXML(node);
}

EulerConstraint::EulerConstraint(double t, const double I_eulerXYZ[3],
                                 const std::string& trajNameND) :
  ConstraintSet(), A_BI(NULL), oriTrj(NULL), oriTrjName(trajNameND)
{
  setClassName("EulerConstraint");
  double quat[4];
  Mat3d_setIdentity(this->A_PB);
  Quat_fromEulerAngles(quat, I_eulerXYZ);

  add(t, quat[0], trajNameND + " 0");
  add(t, quat[1], trajNameND + " 1");
  add(t, quat[2], trajNameND + " 2");
  add(t, quat[3], trajNameND + " 3");
}

EulerConstraint::EulerConstraint(double t, const HTr* A_BI_,
                                 const double I_eulerXYZ[3],
                                 const std::string& trajNameND) :
  ConstraintSet(), A_BI(A_BI_), oriTrj(NULL), oriTrjName(trajNameND)
{
  setClassName("EulerConstraint");
  double A_PI[3][3], quat[4];
  Mat3d_fromEulerAngles(A_PI, I_eulerXYZ);
  Mat3d_mulTranspose(this->A_PB, A_PI, (double(*)[3])A_BI->rot);
  Quat_fromEulerAngles(quat, I_eulerXYZ);

  add(t, quat[0], trajNameND + " 0");
  add(t, quat[1], trajNameND + " 1");
  add(t, quat[2], trajNameND + " 2");
  add(t, quat[3], trajNameND + " 3");
}

EulerConstraint::EulerConstraint(const EulerConstraint& other) :
  ConstraintSet(other), A_BI(NULL), oriTrj(NULL),
  oriTrjName(other.oriTrjName)
{
  Mat3d_copy(this->A_PB, (double(*)[3])other.A_PB);
  RCHECK_MSG(other.A_BI==NULL, "Should we handle this?");
}

EulerConstraint::~EulerConstraint()
{
}

EulerConstraint* EulerConstraint::clone() const
{
  RCHECK_MSG(A_BI==NULL, "This does not yet work due to the A_BI pointer");
  EulerConstraint* tSet = new EulerConstraint();
  tSet->constraint = constraint;
  tSet->className = className;
  tSet->oriTrj = NULL;
  tSet->oriTrjName = oriTrjName;
  Mat3d_copy(tSet->A_PB, (double (*)[3])A_PB);

  for (size_t i = 0; i < set.size(); ++i)
  {
    tSet->add(std::shared_ptr<ConstraintSet>(set[i]->clone()));
  }

  return tSet;
}

bool EulerConstraint::makeShortestPath(double qCurr[4])
{
  RCHECK(this->oriTrj);
  if (numConstraints(false)==0 || this->oriTrj==NULL)
  {
    RLOG(5, "No constraints assigned - skipping shortest path generation");
    return false;
  }

  // Here we check the magnitude of the intermediate angle between the
  // previous and current quaternion. If the dot product of the quaternions
  // in negative, the intermediate angle is not the shortest connection of
  // the rotations. In this case, we negate the target quaternion. This will
  // still be the same rotation, however the interpolation will be the
  // shortest one.
  bool flipped = false;
  double qPrev[4];

  getPreviousQuaternion(qPrev);

  if (Quat_dot(qCurr, qPrev)<0.0)
  {
    VecNd_constMulSelf(qCurr, -1.0, 4);
    flipped = true;
  }

  return flipped;
}

void EulerConstraint::getWorldQuaternion(double quat[4])
{
  if (A_BI)
  {
    double A_PI[3][3];
    Mat3d_transposeMul(A_PI, A_PB, (double(*)[3]) A_BI->rot);
    Quat_fromRotationMatrix(quat, A_PI);
  }
  else
  {
    Quat_fromRotationMatrix(quat, A_PB);
  }

}

std::string EulerConstraint::getTrajectoryName() const
{
  return oriTrjName;
}

void EulerConstraint::apply(std::vector<TrajectoryND*>& trajectory,
                            std::map<std::string, Trajectory1D*>& tMap,
                            bool permissive)
{
  RCHECK(numConstraints(false) == 4);

  // First, we store the reference for the orientation trajectory, since
  // this is needed in the compute() method.
  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    if (trajectory[i]->getName() == this->oriTrjName)
    {
      this->oriTrj = trajectory[i];
      break;
    }
  }

  if (!permissive)
  {
    RCHECK_MSG(this->oriTrj, "Couldn't find trajectory \"%s\"",
               oriTrjName.c_str());
  }

  if (this->oriTrj)
  {
    // Here's the actual apply() part, where we also consider the shortest path
    double qCurr[4];
    getQuaternion(qCurr);
    makeShortestPath(qCurr);
    setQuaternion(qCurr);

    for (size_t i=0; i<4; ++i)
    {
      // Here we check if the constraint is added after all other constraints.
      // This is mandatory to make sure the quaternion interpolation is done
      // on the shortest path.
      Trajectory1D* traj_i = oriTrj->getTrajectory1D(i);
      size_t nGoals = traj_i->getNumberOfGoals();
      double t_last = -1.0;

      if (nGoals > 0)
      {
        t_last = traj_i->getGoalTime(nGoals-1);
      }

      size_t nVia = traj_i->getNumberOfViaPoints();

      if (nVia > 0)
      {
        double t_lastInter = traj_i->getViaTime(nVia-1);
        t_last = std::max(t_last, t_lastInter);
      }

      // That's a problem. In this case, we would need to check all consecutive
      // quaternion constraints against their new and possibly flipped
      // predecessors. Since we usually add constraints in increasing time,
      // this problem will be tackled once needed. Hopefully never ...
      const double cTime = getConstraint(i)->getTime();
      if ((nVia+nGoals>0) && (cTime<t_last) && (cTime>0.0))
      {
        RLOG(0, "EulerConstraint: OH NO! Constraint time %f < time of last "
             "constraint %f", cTime, t_last);
      }

      // Add the constraint to the trajectory.
      traj_i->addConstraint(getConstraint(i));
    }

  }   // if (this->oriTrj)

  // Continue recursion
  for (size_t i = 0; i<set.size(); ++i)
  {
    set[i]->apply(trajectory, tMap, permissive);
  }

}

// The big question: Do we need to check this at any point in time? Otherwise,
// we could drop the oriTrj member pointer, which would make copying of this
// class etc. a lot more easy.
double EulerConstraint::compute(double dt)
{
  double quat[4];
  getQuaternion(quat);
  bool flip = makeShortestPath(quat);
  if (flip)
  {
    RFATAL("FLIP");
  }
  setQuaternion(quat);

  return ConstraintSet::compute(dt);
}

void EulerConstraint::getQuaternion(double quat[4]) const
{
  RCHECK_MSG(constraint.size()==4, "Size is %zu", constraint.size());
  quat[0] = getConstraint(0)->getPosition();
  quat[1] = getConstraint(1)->getPosition();
  quat[2] = getConstraint(2)->getPosition();
  quat[3] = getConstraint(3)->getPosition();
}

void EulerConstraint::setQuaternion(const double quat[4])
{
  RCHECK_MSG(constraint.size()==4, "Size is %zu", constraint.size());
  getConstraint(0)->setPosition(quat[0]);
  getConstraint(1)->setPosition(quat[1]);
  getConstraint(2)->setPosition(quat[2]);
  getConstraint(3)->setPosition(quat[3]);
}

void EulerConstraint::getPreviousQuaternion(double qPrev[4]) const
{
  RCHECK(this->oriTrj);
  const double t_constraint = getConstraint(0)->getTime();

  for (size_t i=0; i<4; ++i)
  {
    const Trajectory1D* t1d = oriTrj->getTrajectory1D(i);
    qPrev[i] = t1d->getPositionConstraintBefore(t_constraint);
  }

}

void EulerConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  double t, I_eulerXYZ[3];
  bool success = Rcs::getXMLNodePropertySTLString(node, "trajectory", oriTrjName);
  success = getXMLNodePropertyDouble(node, "t", &t) && success;
  success = getXMLNodePropertyVec3(node, "pos", I_eulerXYZ) && success;
  Vec3d_constMulSelf(I_eulerXYZ, M_PI/180.0);   // Convert angles from degrees

  int flag = 7;
  getXMLNodePropertyInt(node, "flag", &flag);

  if (success)
  {
    double quat[4];
    Quat_fromEulerAngles(quat, I_eulerXYZ);
    add(t, quat[0], 0.0, 0.0, flag, oriTrjName + " 0");
    add(t, quat[1], 0.0, 0.0, flag, oriTrjName + " 1");
    add(t, quat[2], 0.0, 0.0, flag, oriTrjName + " 2");
    add(t, quat[3], 0.0, 0.0, flag, oriTrjName + " 3");
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

void EulerConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Open set's xml description. The getClassName() function is polymorphic
  std::string indStr(indent, ' ');
  outStream << indStr << "<ConstraintSet type=\""
            << getClassName() << "\" ";

  // Convert internal quaternion to Euler angles. These go into the xml file.
  double quat[4], ea[3], len;
  getQuaternion(quat);
  len = VecNd_normalizeSelf(quat, 4);
  RCHECK_MSG(len>0.0, "Couldn't normalize quaternion");
  Quat_toEulerAngles(ea, quat);
  Vec3d_constMulSelf(ea, 180.0/M_PI);   // Write angles in degrees

  // Write out information to top-level tag
  outStream << "t=\"" << constraint[0].c->getTime() << "\" ";
  outStream << "pos=\""<< ea[0] << " " << ea[1] << " " << ea[2] << "\" ";

  if (constraint[0].c->getFlag() != 7)
  {
    outStream << "flag=\"" << constraint[0].c->getFlag() << "\" ";
  }

  outStream << "trajectory=\"" << oriTrjName << "\"";

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
