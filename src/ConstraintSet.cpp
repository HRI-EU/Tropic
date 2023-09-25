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

#include "ConstraintSet.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <fstream>
#include <limits>
#include <string>

#if !defined (_MSC_VER)
#include <cxxabi.h>
using __cxxabiv1::__cxa_demangle;
#endif



namespace tropic
{
REGISTER_CONSTRAINT(ConstraintSet);

/*******************************************************************************
 *
 ******************************************************************************/
ConstraintSet::ConstraintSet() : className("ConstraintSet")
{
}

/*******************************************************************************
 *
 ******************************************************************************/
ConstraintSet::ConstraintSet(xmlNode* node) : className("ConstraintSet")
{
  fromXML(node);
}

/*******************************************************************************
 *
 ******************************************************************************/
ConstraintSet::ConstraintSet(const ConstraintSet& other) :
  className(other.className), constraint(other.constraint)
{
  // Remove constraints and sets on all levels
  clear();

  for (size_t i = 0; i < other.children.size(); ++i)
  {
    // Calls the copy constructor for the RHS, which in turn calls the copy
    // constructor ... so that we go through all in the hierarchy
    // auto child = std::make_shared<ConstraintSet>(*(other.set[i].get()));
    // add(child);

    add(std::shared_ptr<ConstraintSet>(other.children[i]->clone()));
  }

}

/*******************************************************************************
 * For correct polymorphism, the clone method must be implemented in each
 * derived class.
 ******************************************************************************/
ConstraintSet* ConstraintSet::clone() const
{
  //return new ConstraintSet(*this);
  ConstraintSet* tSet = new ConstraintSet();
  tSet->constraint = constraint;
  tSet->className = className;

  for (size_t i = 0; i < children.size(); ++i)
  {
    // Recursive cloning
    auto child = children[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}


/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::isEqual(const ConstraintSet& other) const
{
  if ((constraint!=other.constraint) ||
      (className!=other.className) ||
      (getTypeName()!=other.getTypeName()) ||
      (children.size()!=other.children.size()))
  {
    return false;
  }

  // From here on we call the shared_ptr isEqual() function
  for (size_t i=0; i< children.size(); ++i)
  {
    if (!children[i]->isEqual(other.children[i]))
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::isEqual(std::shared_ptr<ConstraintSet> other) const
{
  RLOG_CPP(5, "Comparing " << getClassName());
  if (constraint != other->constraint)
  {
    RLOG_CPP(5, "Constraints differ");

    REXEC(6)
    {
      if (constraint.size() != other->constraint.size())
      {
        RLOG_CPP(6, "size1 = " << constraint.size() << " size 2 = "
                 << other->constraint.size());
      }
      else
      {
        for (size_t i=0; i<constraint.size(); ++i)
        {
          RLOG_CPP(6, "Constraint " << i << " = " << *(constraint[i].c) << "   "
                   << *(other->constraint[i].c));
        }
      }
    }
    return false;
  }


  if (className != other->className)
  {
    RLOG_CPP(5, "Names differ");
    return false;
  }

  if (children.size() != other->children.size())
  {
    RLOG_CPP(5, "Sizes differ");
    return false;
  }

  // From here on we call the != compare operator.
  for (size_t i=0; i< children.size(); ++i)
  {
    if (!children[i]->isEqual(other->children[i]))
    {
      RLOG(5, "Set %zu differs", i);
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::operator == (const ConstraintSet& other) const
{
  return isEqual(other);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::operator == (const std::shared_ptr<ConstraintSet> other) const
{
  return isEqual(other);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::operator != (const ConstraintSet& other) const
{
  return !isEqual(other);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::operator != (const std::shared_ptr<ConstraintSet> other) const
{
  return !isEqual(other);
}

/*******************************************************************************
 *
 ******************************************************************************/
ConstraintSet::~ConstraintSet()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t ConstraintSet::numConstraints(bool recursive) const
{
  size_t nConstraints = constraint.size();

  if (recursive)
  {
    for (size_t i=0; i< children.size(); ++i)
    {
      nConstraints += children[i]->numConstraints(recursive);
    }
  }

  return nConstraints;
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t ConstraintSet::numSets(bool recursive) const
{
  size_t nSets = children.size();

  if (recursive==false)
  {
    return nSets;
  }

  for (size_t i=0; i< children.size(); ++i)
  {
    nSets += children[i]->numSets();
  }

  return nSets;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ConstraintSet::compute(double dt)
{
  double endTime = 0.0;

  // Determine the end time of this set's constraints
  for (size_t i=0; i<this->constraint.size(); ++i)
  {
    endTime = std::max(endTime, constraint[i].c->getTime());
  }

  std::vector<int> idxToDelete;

  for (int i=this->children.size()-1; i>=0; i--)
  {
    // If the sub-set has no children, and none of its constraints is used
    // anywhere else, we delete it. This means that the constraint will also
    // get deleted if its time is larger than zero. This might create some
    // issues if compute() is called for a set that has not been added to
    // any trajectory, and is inspected (or written to file) after calling
    // compute: Some constraints might be missing then.
    if ((children[i]->children.size()==0) && (!children[i]->inUse()))
    {
      idxToDelete.push_back(i);
    }
    else
    {
      const double sTime = children[i]->compute(dt);
      endTime = std::max(endTime, sTime);
    }
  }

  // idxToDelete is sorted from large to small, so we can erase the entries in
  // a loop going forward in idxToDelete.
  for (size_t i=0; i<idxToDelete.size(); ++i)
  {
    children.erase(children.begin()+idxToDelete[i]);
  }


  return endTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ConstraintSet::getStartTime() const
{
  double startTime = getStartTimeRecurse();

  if (startTime==std::numeric_limits<double>::max())
  {
    startTime = 0.0;
  }

  return startTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ConstraintSet::getStartTimeRecurse() const
{
  double startTime = std::numeric_limits<double>::max();

  for (size_t i=0; i<this->constraint.size(); ++i)
  {
    startTime = std::min(startTime, constraint[i].c->getTime());
  }

  for (size_t i=0; i<this->children.size(); ++i)
  {
    startTime = std::min(startTime, children[i]->getStartTimeRecurse());
  }

  return startTime < 0.0 ? 0.0 : startTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ConstraintSet::getEndTime() const
{
  double endTime = 0.0;

  for (size_t i=0; i<this->constraint.size(); ++i)
  {
    endTime = std::max(endTime, constraint[i].c->getTime());
  }

  for (size_t i=0; i<this->children.size(); ++i)
  {
    endTime = std::max(endTime, children[i]->getEndTime());
  }

  return endTime;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ConstraintSet::getDuration() const
{
  return getEndTime() - getStartTime();
}

/*******************************************************************************
 *
 ******************************************************************************/
std::shared_ptr<Constraint1D> ConstraintSet::getConstraint(size_t idx) const
{
  if (idx >= constraint.size())
  {
    RLOG(1, "WARNING: idx: %d   size: %d", (int)idx, (int)constraint.size());
    return nullptr;
  }

  return constraint[idx].c;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::shared_ptr<ConstraintSet> ConstraintSet::getSet(size_t idx) const
{
  if (idx >= children.size())
  {
    RLOG_CPP(1, "WARNING: idx: " << idx << " size: " << children.size());
    return nullptr;
  }

  return children[idx];
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string ConstraintSet::getTrajectoryName(size_t idx) const
{
  if (idx >= constraint.size())
  {
    RLOG_CPP(1, "WARNING: idx: " << idx << " size: " << constraint.size());
    return std::string();
  }

  return constraint[idx].trajName1D;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::add(std::shared_ptr<ConstraintSet> other)
{
  if (other != nullptr)
  {
    this->children.push_back(other);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::add(NamedConstraint1D idxConstraint)
{
  this->constraint.push_back(idxConstraint);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::add(double t, double x,
                        const std::string& trajName)
{
  auto tc = std::make_shared<Constraint1D>(t, x, 0.0, 0.0, 7);
  this->constraint.push_back(NamedConstraint1D(tc, trajName));
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::add(double t, double x, double x_dot,
                        double x_ddot, int flag,
                        const std::string& trajName)
{
  auto tc = std::make_shared<Constraint1D>(t, x, x_dot, x_ddot, flag);
  this->constraint.push_back(NamedConstraint1D(tc, trajName));
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::shiftTime(double dt)
{
  for (size_t i=0; i<constraint.size(); ++i)
  {
    constraint[i].c->shiftTime(dt);
  }

  for (size_t i=0; i< children.size(); ++i)
  {
    children[i]->shiftTime(dt);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::dxdPos(MatNd* grad, const std::shared_ptr<Constraint1D> c,
                           double t0, double t1, double dt) const
{
  RFATAL("FIXME");
  const Trajectory1D* traj = NULL;// findConstraintTrajectory(c);

  if (traj==NULL)
  {
    RLOG(1, "Can't find trajectory for constraint - skipping gradient");
    return false;
  }

  return traj->dxdPosConstraint(grad, c, t0, t1, dt);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::apply(std::vector<TrajectoryND*>& trajectory,
                          bool permissive)
{
  std::map<std::string,Trajectory1D*> tMap;

  for (size_t i=0; i<trajectory.size(); ++i)
  {
    const size_t dim = trajectory[i]->getInternalDim();

    for (size_t j=0; j<dim; ++j)
    {
      Trajectory1D* tj = trajectory[i]->getTrajectory1D(j);
      tMap[tj->getName()] = tj;
    }

  }

  apply(trajectory, tMap, permissive);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::apply(std::vector<TrajectoryND*>& trajectory,
                          std::map<std::string,Trajectory1D*>& tMap,
                          bool permissive)
{
  for (size_t i=0; i<constraint.size(); ++i)
  {
    std::map<std::string, Trajectory1D*>::iterator it;
    it = tMap.find(constraint[i].trajName1D);

    // We failed
    if (it == tMap.end())
    {
      if (!permissive)
      {
        RLOG_CPP(1, "Trajectory map has " << tMap.size() << " entries");
        for (auto it=tMap.begin(); it!=tMap.end(); ++it)
        {
          RLOG_CPP(1, "Candidate: " << it->first);
        }

        RFATAL("Couldn't find trajectory \"%s\"",
               constraint[i].trajName1D.c_str());
      }
    }
    else
    {
      // We succeeded
      Trajectory1D* traj1D = it->second;
      traj1D->addConstraint(constraint[i].c);
    }
  }

  for (size_t i = 0; i< children.size(); ++i)
  {
    children[i]->apply(trajectory, tMap, permissive);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::clear()
{
  // We go through all sub-sets and call their clear function before we
  // delete it
  for (size_t i=0; i< children.size(); ++i)
  {
    children[i]->clear();
  }

  // Then we empty the vector of sub-sets of this set
  children.clear();

  // Then we remove all constraint from the current set. They don't need to be
  // deleted explicitely since that's done through shared_ptrs.
  constraint.clear();
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string ConstraintSet::getClassName() const
{
  return this->className;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::setClassName(const std::string& name)
{
  this->className = name;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::print() const
{
  printf("%s (%s): ", getClassName().c_str(), getTypeName().c_str());

  if (constraint.size() == 0)
  {
    printf("empty\n");
  }
  else
  {
    printf("\n");

    for (size_t i=0; i<constraint.size(); ++i)
    {
      printf("   Constraint %d: time=%f ", (int) i, constraint[i].c->getTime());

      if (Math_isBitSet(constraint[i].c->getFlag(), 0))
      {
        printf("pos=%f ", constraint[i].c->getPosition());
      }

      if (Math_isBitSet(constraint[i].c->getFlag(), 1))
      {
        printf("vel=%f ", constraint[i].c->getVelocity());
      }

      if (Math_isBitSet(constraint[i].c->getFlag(), 2))
      {
        printf("acc=%f ", constraint[i].c->getAcceleration());
      }

      printf("trajectory=\"%s\" ", constraint[i].trajName1D.c_str());

      printf("\n");
    }
  }

  if (children.empty())
  {
    printf("no child sets\n");
  }

  for (size_t i=0; i<this->children.size(); ++i)
  {
    children[i]->print();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::inUse() const
{
  for (size_t i = 0; i < constraint.size(); ++i)
  {
    if (constraint[i].c.use_count() > 1)
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string ConstraintSet::getTypeName() const
{
#if defined (_MSC_VER)
  std::string demStr = typeid(*this).name();
#else
  char* demangled = __cxa_demangle(typeid(*this).name(), NULL, 0, NULL);
  std::string demStr = demangled;
  free(demangled);
#endif

  return demStr;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  node = node->children;

  while (node)
  {
    if (isXMLNodeName(node, "Constraint"))
    {
      double t, x, xd, xdd;
      int flag;
      std::string tName;
      bool success = getXMLNodePropertyDouble(node, "t", &t);
      success = getXMLNodePropertyDouble(node, "pos", &x) && success;
      success = getXMLNodePropertyDouble(node, "vel", &xd) && success;
      success = getXMLNodePropertyDouble(node, "acc", &xdd) && success;
      success = getXMLNodePropertyInt(node, "flag", &flag) && success;
      success = Rcs::getXMLNodePropertySTLString(node, "trajectory", tName) && success;

      if (success)
      {
        add(NamedConstraint1D(std::make_shared<Constraint1D>(t, x, xd, xdd, flag), tName));
      }
      else
      {
        RLOG(1, "Failed to create constraint!");
      }

    }
    else
    {
      add(ConstraintFactory::create(node));
    }

    node = node->next;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::toXML(std::string fileName) const
{
  std::ofstream fd;
  fd.open(fileName.c_str());

  if (!fd.good())
  {
    RLOG_CPP(1, "Failed to open file " << fileName);
    return false;
  }

  toXML(fd, 0);

  fd.close();
  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string ConstraintSet::getIdsForXML() const
{
  std::string str = "id=\"";
  for (size_t i=0; i< constraint.size(); ++i)
  {
    str += std::to_string(constraint[i].c->getID());

    if (i != constraint.size()-1)
    {
      str += " ";
    }
  }
  str += "\" ";

  return str;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ConstraintSet::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\""
            << getClassName() << "\" >" << std::endl;

  for (size_t i=0; i<constraint.size(); ++i)
  {
    outStream << indStr << "  <Constraint ";
    outStream << "t=\"" << constraint[i].c->getTime() << "\" ";
    outStream << "pos=\"" << constraint[i].c->getPosition() << "\" ";
    outStream << "vel=\"" << constraint[i].c->getVelocity() << "\" ";
    outStream << "acc=\"" << constraint[i].c->getAcceleration() << "\" ";
    outStream << "flag=\"" << constraint[i].c->getFlag() << "\" ";
    outStream << "trajectory=\"" << constraint[i].trajName1D << "\" ";
    outStream << "id=\"" << constraint[i].c->getID() << "\" ";
    outStream << " />" << std::endl;
  }

  // Go recursively through all child sets
  for (size_t i=0; i< children.size(); ++i)
  {
    children[i]->toXML(outStream, indent+2);
  }

  // Close set's xml description
  outStream << indStr << "</ConstraintSet>" << std::endl;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ConstraintSet::testXML() const
{
  std::string file1("config/xml/Tropic/ConstraintSetTest1.xml");
  std::string file2("ConstraintSetTest2.xml");

  toXML(file1);
  auto tt = ConstraintFactory::create(file1);
  if (!tt)
  {
    return false;
  }

  tt->toXML(file2);

  return File_isEqual(file1.c_str(), file2.c_str());
}

}   // namespace tropic
