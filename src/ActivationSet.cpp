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

#include "ActivationSet.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>

#include <algorithm>



namespace tropic
{
REGISTER_CONSTRAINT(ActivationSet);

ActivationSet::ActivationSet() : ConstraintSet()
{
  setClassName("ActivationSet");
}

ActivationSet::ActivationSet(const ActivationSet& other) :
  ConstraintSet(other)
{
  for (size_t i=0; i<other.aVec.size(); ++i)
  {
    ActivationPoint* otherA = other.aVec[i].c->clone();
    aVec.push_back(NamedActivationPoint(std::shared_ptr<ActivationPoint>(otherA),
                                        other.aVec[i].trajNameND));
  }
}

ActivationSet::ActivationSet(xmlNode* node)
{
  setClassName("ActivationSet");
  fromXML(node);
}

ActivationSet::~ActivationSet()
{
}

ActivationSet* ActivationSet::clone() const
{
  //return new ActivationSet(*this);
  ActivationSet* tSet = new ActivationSet();
  tSet->constraint = constraint;
  tSet->className = className;
  for (size_t i=0; i<aVec.size(); ++i)
  {
    ActivationPoint* newA = aVec[i].c->clone();
    tSet->aVec.push_back(NamedActivationPoint(std::shared_ptr<ActivationPoint>(newA), aVec[i].trajNameND));

  }

  for (size_t i = 0; i < children.size(); ++i)
  {
    auto child = children[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}

bool ActivationSet::isEqual(const ConstraintSet& other) const
{
  // Check if other is of same type
  const ActivationSet* otherA = dynamic_cast<const ActivationSet*>(&other);

  // ... if there are the same number of activations
  if (aVec.size() != otherA->aVec.size())
  {
    return false;
  }

  // ... and if the activations are identical
  for (size_t i=0; i<aVec.size(); ++i)
  {
    if (!(aVec[i]==otherA->aVec[i]))
    {
      return false;
    }
  }

  // ... and continue with parent class traversal
  return ConstraintSet::isEqual(other);
}

void ActivationSet::apply(std::vector<TrajectoryND*>& trajectory,
                          std::map<std::string, Trajectory1D*>& tMap,
                          bool permissive)
{
  ConstraintSet::apply(trajectory, tMap, permissive);

  std::vector<int> idxToDelete;

  for (int i = aVec.size()-1; i>=0; i--)
  {
    TrajectoryND* trajND = NULL;

    for (size_t j=0; j<trajectory.size(); ++j)
    {
      //RLOG(0, "Comparing %s - %s", trajectory[j]->getName().c_str(), aVec[i]->trajName.c_str());
      if (trajectory[j]->getName()==aVec[i].trajNameND)
      {
        trajND = trajectory[j];
        break;
      }
    }

    if (!permissive)
    {
      RCHECK_MSG(trajND, "Can't apply ActivationSet for \"%s\"",
                 aVec[i].trajNameND.c_str());
    }

    if (trajND)
    {
      // The addActivationPoint() method returns false if the point has not
      // been added, for instance because there is already another point with
      // the same time point. In such cases, only the values are overwritten,
      // but the activation point itself is not added to the TrajectoryND's
      // activation vector. In order to not having a dangling constraint, we
      // delete it right away. If we don't, it will lead to erroneous
      // computations of the start and ent times.
      bool success = trajND->addActivationPoint(aVec[i].c);
      if (!success)
      {
        idxToDelete.push_back(i);
      }
    }
  }

  // idxToDelete is sorted from large to small, so we can erase the entries in
  // a loop going forward in idxToDelete.
  for (size_t i=0; i<idxToDelete.size(); ++i)
  {
    aVec.erase(aVec.begin()+idxToDelete[i]);
  }

}

void ActivationSet::clear()
{
  aVec.clear();
  ConstraintSet::clear();
}

void ActivationSet::addActivation(std::shared_ptr<ActivationPoint> aPt, std::string trajNameND)
{
  aVec.push_back(NamedActivationPoint(aPt, trajNameND));
}

void ActivationSet::addActivation(double time, bool switchOn, double horizon,
                                  std::string trajNameND)
{
  addActivation(std::make_shared<ActivationPoint>(time, switchOn, horizon), trajNameND);
}

double ActivationSet::compute(double dt)
{
  double endTime = ConstraintSet::compute(dt);

  // Determine the end time of this set's activation points
  for (auto const& a : aVec)
  {
    endTime = std::max(endTime, a.c->getTime());
  }

  return endTime;
}

double ActivationSet::getStartTimeRecurse() const
{
  double startTime = ConstraintSet::getStartTimeRecurse();

  for (auto const& a: aVec)
  {
    startTime = std::min(startTime, a.c->getTime());
  }

  return startTime < 0.0 ? 0.0 : startTime;
}

double ActivationSet::getEndTime() const
{
  double endTime = 0.0;

  for (auto const& a : aVec)
  {
    endTime = std::max(endTime, a.c->getTime());
  }

  return std::max(endTime, ConstraintSet::getEndTime());
}

bool ActivationSet::toXML(std::string fileName) const
{
  return ConstraintSet::toXML(fileName);
}

bool ActivationSet::inUse() const
{
  for (size_t i = 0; i < aVec.size(); ++i)
  {
    if (aVec[i].c.use_count() > 1)// || aVec[i].c->getTime()>0.0)
    {
      return true;
    }
  }

  return ConstraintSet::inUse();
}

void ActivationSet::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\""
            << getClassName() << "\" >" << std::endl;

  for (auto const& ai : aVec)
  {
    outStream << indStr << "  <Activation ";
    outStream << "t=\"" << ai.c->getTime() << "\" ";

    if (ai.c->switchesOn())
    {
      outStream << "switchesOn=\"true\" ";
    }
    else
    {
      outStream << "switchesOn=\"false\" ";
    }

    outStream << "horizon=\"" << ai.c->getHorizon() << "\" ";
    outStream << "trajectory=\"" << ai.trajNameND << "\" />" << std::endl;
  }

  // \todo: This duplicates the function in TrajectoryConstraint Set. Maybe here we can do better?
  for (auto const& ci : constraint)
  {
    outStream << indStr << "  <Constraint ";
    outStream << "t=\"" << ci.c->getTime() << "\" ";
    outStream << "pos=\"" << ci.c->getPosition() << "\" ";
    outStream << "vel=\"" << ci.c->getVelocity() << "\" ";
    outStream << "acc=\"" << ci.c->getAcceleration() << "\" ";
    outStream << "flag=\"" << ci.c->getFlag() << "\" ";
    outStream << "trajectory=\"" << ci.trajName1D << "\" ";
    outStream << "id=\"" << ci.c->getID() << "\" ";
    outStream << " />" << std::endl;
  }

  // Go recursively through all child sets
  for (auto const& child : children)
  {
    child->toXML(outStream, indent+2);
  }

  // Close set's xml description
  outStream << indStr << "</ConstraintSet>" << std::endl;
}

void ActivationSet::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  node = node->children;

  while (node)
  {
    if (isXMLNodeName(node, "Activation"))
    {
      double t, horizon;
      bool switchesOn;
      std::string tName;
      bool success = getXMLNodePropertyDouble(node, "t", &t);
      success = getXMLNodePropertyBoolString(node, "switchesOn", &switchesOn) && success;
      success = getXMLNodePropertyDouble(node, "horizon", &horizon) && success;
      success = Rcs::getXMLNodePropertySTLString(node, "trajectory", tName) && success;

      if (success)
      {
        addActivation(t, switchesOn, horizon, tName);
      }
      else
      {
        RLOG(1, "Failed to create activation constraint!");
      }

    }

    // \todo: This duplicates the function in TrajectoryConstraint Set. Maybe here we can do better?
    else if (isXMLNodeName(node, "Constraint"))
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


}   // namespace tropic
