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

#include "KeepPositionConstraint.h"
#include "ConstraintFactory.h"
#include "VectorConstraint.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>

#include <string>
#include <limits>
#include <algorithm>



namespace tropic
{
REGISTER_CONSTRAINT(KeepPositionConstraint);

KeepPositionConstraint::KeepPositionConstraint() :
  ConstraintSet(), startTime(-1.0), endTime(-1.0), horizon(1.0), active(true), trajND(NULL)
{
  setClassName("KeepPositionConstraint");
}

KeepPositionConstraint::KeepPositionConstraint(xmlNode* node) :
  ConstraintSet(node), startTime(-1.0), endTime(-1.0), horizon(1.0), active(true), trajND(NULL)
{
  setClassName("KeepPositionConstraint");
  fromXML(node);
}

KeepPositionConstraint::KeepPositionConstraint(double t0, double t1, const std::string& trajNameND) :
  ConstraintSet(), startTime(t0), endTime(t1), horizon(1.0), active(true), trajND(NULL), trjName(trajNameND)
{
  setClassName("KeepPositionConstraint");
  RLOG_CPP(0, "KeepPositionConstraint: trajectory is " << trjName);
}

KeepPositionConstraint::KeepPositionConstraint(double t0, double t1, double hrz, const std::string& trajNameND) :
  ConstraintSet(), startTime(t0), endTime(t1), horizon(hrz), active(true), trajND(NULL), trjName(trajNameND)
{
  setClassName("KeepPositionConstraint");
}

KeepPositionConstraint::~KeepPositionConstraint()
{
}

KeepPositionConstraint* KeepPositionConstraint::clone() const
{
  KeepPositionConstraint* tSet = new KeepPositionConstraint();
  tSet->constraint = constraint;
  tSet->className = className;

  tSet->startTime = startTime;
  tSet->endTime = endTime;
  tSet->horizon = horizon;
  tSet->active = active;
  tSet->trajND = NULL;;
  tSet->trjName = trjName;

  for (size_t i = 0; i < set.size(); ++i)
  {
    auto child = set[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}

void KeepPositionConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  bool success = Rcs::getXMLNodePropertySTLString(node, "trajectory", trjName);
  success = getXMLNodePropertyDouble(node, "t0", &startTime) && success;
  success = getXMLNodePropertyDouble(node, "t1", &endTime) && success;
  horizon = 1.0;
  getXMLNodePropertyDouble(node, "horizon", &horizon);

  node = node->children;

  while (node)
  {
    add(ConstraintFactory::create(node));
    node = node->next;
  }

}

void KeepPositionConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\"" << getClassName() << "\" ";

  // Write out information to top-level tag
  outStream << "t0=\"" << startTime << "\" ";
  outStream << "t1=\"" << endTime << "\" ";
  outStream << "horizon=\"" << horizon << "\" ";
  outStream << "trajectory=\"" << trjName << "\"";

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

double KeepPositionConstraint::compute(double dt)
{
  startTime -= dt;
  endTime -= dt;
  double activationTime = startTime - horizon;

  if ((activationTime < 0.0) && (activationTime >= -dt))
  {
    std::vector<double> x_keep(trajND->getDim());
    std::vector<double> x_keep_i(trajND->getInternalDim());

    trajND->getPosition(horizon, x_keep.data());
    trajND->toInternalCoords(x_keep_i.data(), x_keep.data());

    for (size_t i = 0; i < x_keep_i.size(); ++i)
    {
      double t0 = horizon;
      double t1 = horizon + (endTime - startTime);
      trajND->getTrajectory1D(i)->addConstraint(t0, x_keep_i[i], 0.0, 0.0, 7);
      trajND->getTrajectory1D(i)->addConstraint(t1, x_keep_i[i], 0.0, 0.0, 7);
    }

  }

  // When active is false, inUse() returns false and the set will automatically
  // be cleaned up.
  if ((endTime < 0.0) && (endTime >= -dt))
  {
    this->active = false;
  }

  return ConstraintSet::compute(dt);
}

bool KeepPositionConstraint::inUse() const
{
  return this->active;
}

void KeepPositionConstraint::apply(std::vector<TrajectoryND*>& trajectory,
                                   std::map<std::string, Trajectory1D*>& tMap,
                                   bool permissive)
{
  permissive = false;
  size_t found = 0;
  double minHorizon = std::numeric_limits<double>::max();

  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    if (trjName == trajectory[i]->getName())
    {
      trajND = trajectory[i];

      // Here we check that the class's given horizon is less than the minimum
      // of each trajectories horizon. In case of the turbo mode, this is the
      // maximum allowed future time point to calculate the polynomials.
      for (size_t i = 0; i < trajND->getDim(); ++i)
      {
        minHorizon = std::min(minHorizon, trajND->getTrajectory1D(i)->getHorizonTime());
      }

      found++;
    }
  }

  RLOG(0, "Horizon is %f, trajectory can be computed up to time point %f",
       horizon, minHorizon);

  if (!permissive)
  {
    if (minHorizon < horizon)
    {
      RFATAL("Horizon is %f, but trajectory can only be computed up to time point %f",
             horizon, minHorizon);
    }

    if (found != 1)
    {
      RFATAL("Found %zu trajectories with name \"%s\" - expected 1",
             found, trjName.c_str());
    }
  }

  ConstraintSet::apply(trajectory, tMap, permissive);
}

}   // namespace tropic
