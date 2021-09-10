
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

#ifndef TROPIC_LIFTOBJECTCONSTRAINT_H
#define TROPIC_LIFTOBJECTCONSTRAINT_H

#include "ConstraintSet.h"
#include "PoseConstraint.h"
#include "PolarConstraint.h"
#include "ConnectBodyConstraint.h"
#include "VectorConstraint.h"
#include "EulerConstraint.h"
#include "ActivationSet.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for lifting an object from a surface
 *
 */
class LiftObjectConstraint : public ConstraintSet
{
public:

  LiftObjectConstraint(const Rcs::ControllerBase* controller,
                       const std::string& handName_,
                       const std::string& objectName_,
                       const std::string& surfaceName_,
                       double t_start,
                       double duration_grasp,
                       double duration_lift,
                       double graspHeight=0.1) : ConstraintSet()
  {
    RLOG_CPP(0, "handName is " << handName_);
    RLOG_CPP(0, "objectName is " << objectName_);
    RLOG_CPP(0, "surfaceName is " << surfaceName_);

    const RcsBody* bdy;
    bdy = RcsGraph_getBodyByName(controller->getGraph(), handName_.c_str());
    if (!bdy)
    {
      throw (std::string("Failed to find body for " + handName_));
    }

    handName = std::string(bdy->name);

    bdy = RcsGraph_getBodyByName(controller->getGraph(), objectName_.c_str());
    if (!bdy)
    {
      throw (std::string("Failed to find body for " + objectName_));
    }
    objectName = std::string(bdy->name);

    bdy = RcsGraph_getBodyByName(controller->getGraph(), surfaceName_.c_str());
    if (!bdy)
    {
      throw (std::string("Failed to find body for " + surfaceName_));
    }
    surfaceName = std::string(bdy->name);

    RLOG_CPP(0, "handName is " << handName);
    RLOG_CPP(0, "objectName is " << objectName);
    RLOG_CPP(0, "surfaceName is " << surfaceName);

    for (size_t i=0; i<controller->getNumberOfTasks(); ++i)
    {
      const Rcs::Task* ti = controller->getTask(i);

      RLOG_CPP(0, "Task: " << ti->getName() << ": effector: "
               << getEffectorName(ti) << " refBdy: "
               << getRefBodyName(ti));

      // taskObjHandPos
      if ((getEffectorName(ti)==objectName) &&
          (getRefBodyName(ti)==handName) &&
          (ti->getClassName()=="XYZ"))
      {
        RCHECK(taskObjHandPos.empty());
        taskObjHandPos = ti->getName();
      }

      // taskObjSurfacePos
      if ((getEffectorName(ti)==objectName) &&
          (getRefBodyName(ti)==surfaceName) &&
          (ti->getClassName()=="Z"))
      {
        RCHECK(taskObjSurfacePos.empty());
        RLOG(0, "Found task with effector=\"%s\" and refBdy=\"%s\"",
             objectName.c_str(), surfaceName.c_str());
        taskObjSurfacePos = ti->getName();
      }

      // taskObjPolar
      if ((getEffectorName(ti)==objectName) &&
          (ti->getClassName()=="POLAR"))
      {
        RCHECK(taskObjPolar.empty());
        taskObjPolar = ti->getName();
      }

      // taskHandObjPolar
      if ((getEffectorName(ti)==handName) &&
          (getRefBodyName(ti)== objectName) &&
          (ti->getClassName()=="POLAR"))
      {
        RCHECK(taskHandObjPolar.empty());
        taskHandObjPolar = ti->getName();
      }
    }
    // Check that all tasks have been found
    RCHECK_MSG(!taskObjHandPos.empty(),
               "Didn't find task with effector=\"%s\" and refBdy=\"%s\"",
               objectName.c_str(), handName.c_str());
    RCHECK_MSG(!taskObjSurfacePos.empty(),
               "Didn't find task with effector=\"%s\" and refBdy=\"%s\"",
               objectName.c_str(), surfaceName.c_str());
    RCHECK(!taskObjPolar.empty());
    RCHECK(!taskHandObjPolar.empty());

    const double t_grasp = t_start + duration_grasp;
    const double t_end = t_grasp + duration_lift;
    add(lift(t_start, t_grasp, t_end, graspHeight));
    add(put(t_end, t_end+3, t_end+6));
  }

  std::shared_ptr<tropic::ActivationSet> put(double t_start, double t_put, double t_release) const
  {
    // Grasp the bottle and lift it up
    auto a1 = std::make_shared<tropic::ActivationSet>();

    // Put object on surface
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePos);
    a1->addActivation(t_put, false, 0.5, taskObjSurfacePos);
    a1->add(t_put, 0.0, 0.0, 0.0, 7, taskObjSurfacePos + " 0");
    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_put, objectName, surfaceName));

    // Object orientation wrt world frame
    a1->addActivation(t_start, true, 0.5, taskObjPolar);
    a1->addActivation(t_put, false, 0.5, taskObjPolar);
    a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjPolar));

    return a1;
  }

  std::shared_ptr<tropic::ActivationSet> lift(double t_start, double t_grasp, double t_end, double graspHeight) const
  {
    // Grasp the bottle and lift it up
    auto a1 = std::make_shared<tropic::ActivationSet>();

    // Hand position with respect to bottle
    a1->addActivation(t_start, true, 0.5, taskObjHandPos);
    a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
    a1->add(std::make_shared<tropic::PositionConstraint>(t_grasp, 0.0, 0.0, -graspHeight, taskObjHandPos));

    // Hand orientation with respect to bottle
    a1->addActivation(t_start, true, 0.5, taskHandObjPolar);
    a1->addActivation(t_grasp, false, 0.5, taskHandObjPolar);
    a1->add(std::make_shared<tropic::PolarConstraint>(t_grasp, 0.0, 0.0, taskHandObjPolar));

    // Object orientation wrt world frame
    a1->addActivation(t_grasp, true, 0.5, taskObjPolar);
    a1->addActivation(t_end, false, 0.5, taskObjPolar);
    a1->add(std::make_shared<tropic::PolarConstraint>(t_end, 0.0, 0.0, taskObjPolar));

    // Lift object
    a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePos);
    a1->addActivation(t_end, false, 0.5, taskObjSurfacePos);
    a1->add(t_end, 0.2, 0.0, 0.0, 7, taskObjSurfacePos + " 0");
    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, handName));

    return a1;
  }

  LiftObjectConstraint(xmlNode* node)
  {
  }

  LiftObjectConstraint(const LiftObjectConstraint& other)
  {
    RFATAL("Implement me");// ? is it needed?
  }

  virtual LiftObjectConstraint* clone() const
  {
    return nullptr;
  }

  virtual ~LiftObjectConstraint()
  {
  }
protected:

  virtual void fromXML(xmlNode* node)
  {
  }

  std::string getEffectorName(const Rcs::Task* tsk)
  {
    if (!tsk)
    {
      return std::string();
    }
    if (!tsk->getEffector())
    {
      return std::string();
    }
    return std::string(tsk->getEffector()->name);
  }

  std::string getRefBodyName(const Rcs::Task* tsk)
  {
    if (!tsk)
    {
      return std::string();
    }
    if (!tsk->getRefBody())
    {
      return std::string();
    }
    return std::string(tsk->getRefBody()->name);
  }

  std::string taskObjHandPos;
  std::string taskObjSurfacePos;
  std::string taskObjPolar;
  std::string taskHandObjPolar;

  std::string objectName;
  std::string handName;
  std::string surfaceName;
};

}   // namespace tropic


#endif   // TROPIC_LIFTOBJECTCONSTRAINT_H
