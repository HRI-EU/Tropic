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
class LiftObjectConstraint
{
public:

  LiftObjectConstraint(const Rcs::ControllerBase* controller,
                       const std::string& handName_,
                       const std::string& objectName_,
                       const std::string& surfaceName_) :
    handName(handName_), objectName(objectName_), surfaceName(surfaceName_)
  {
    bool success = getLiftPutTasks(controller, handName, objectName, surfaceName,
                                   taskObjHandPos, taskHandObjPolar, taskObjSurfacePosX,
                                   taskObjSurfacePosY, taskObjSurfacePosZ, taskObjPolar);

    RCHECK(success);
  }

  static bool getLiftPutTasks(const Rcs::ControllerBase* controller,
                              std::string& hand_,
                              std::string& object_,
                              std::string& surface_,
                              std::string& tskObjHandPos_,
                              std::string& tskHandObjPolar_,
                              std::string& tskObjSurfacePosX_,
                              std::string& tskObjSurfacePosY_,
                              std::string& tskObjSurfacePosZ_,
                              std::string& tskObjPolar_)
  {
    bool success = true;

    // Make a local copy of all arguments, we only set them after we know that
    // the function returns with success.
    std::string hand = hand_;
    std::string object = object_;
    std::string surface = surface_;
    std::string tskObjHandPos = tskObjHandPos_;
    std::string tskHandObjPolar = tskHandObjPolar_;
    std::string tskObjSurfacePosX = tskObjSurfacePosX_;
    std::string tskObjSurfacePosY = tskObjSurfacePosY_;
    std::string tskObjSurfacePosZ = tskObjSurfacePosZ_;
    std::string tskObjPolar = tskObjPolar_;

    RLOG_CPP(5, "handName is " << hand);
    RLOG_CPP(5, "objectName is " << object);
    RLOG_CPP(5, "surfaceName is " << surface);

    // We go through the following look-ups to resolve the names of possible
    // generic bodies.
    const RcsBody* bdy;
    bdy = RcsGraph_getBodyByName(controller->getGraph(), hand.c_str());
    if (!bdy)
    {
      RLOG_CPP(1, "Failed to find body for " << hand);
      return false;
    }
    hand = std::string(bdy->name);

    bdy = RcsGraph_getBodyByName(controller->getGraph(), object.c_str());
    if (!bdy)
    {
      RLOG_CPP(1, "Failed to find body for " << object);
      return false;
    }
    object = std::string(bdy->name);

    bdy = RcsGraph_getBodyByName(controller->getGraph(), surface.c_str());
    if (!bdy)
    {
      RLOG_CPP(1, "Failed to find body for " << surface);
      return false;
    }
    surface = std::string(bdy->name);


    for (size_t i=0; i<controller->getNumberOfTasks(); ++i)
    {
      const Rcs::Task* ti = controller->getTask(i);
      std::string efName = getEffectorName(ti);
      std::string refName = getRefBodyName(ti);
      std::string className = ti->getClassName();

      // taskObjHandPos
      if ((efName==object) && (refName==hand) && (className=="XYZ"))
      {
        if (!tskObjHandPos.empty())
        {
          RLOG_CPP(1, "Found duplicate entry for task " << tskObjHandPos);
          return false;
        }
        tskObjHandPos = ti->getName();
      }

      // taskObjSurfacePosition z and sideways velocities
      if ((efName==object) && (refName==surface))
      {
        if (className=="Z")
        {
          if (!tskObjSurfacePosZ.empty())
          {
            RLOG_CPP(1, "Found duplicate entry for task " << tskObjSurfacePosZ);
            return false;
          }
          tskObjSurfacePosZ = ti->getName();
        }
        else if (className=="X")
        {
          if (!tskObjSurfacePosX.empty())
          {
            RLOG_CPP(1, "Found duplicate entry for task " << tskObjSurfacePosX);
            return false;
          }
          tskObjSurfacePosX = ti->getName();
        }
        else if (className=="Y")
        {
          if (!tskObjSurfacePosY.empty())
          {
            RLOG_CPP(1, "Found duplicate entry for task " << tskObjSurfacePosY);
            return false;
          }
          tskObjSurfacePosY = ti->getName();
        }
      }

      // taskObjPolar
      if ((efName==object) && (refName.empty()) && (className=="POLAR"))
      {
        if (!tskObjPolar.empty())
        {
          RLOG_CPP(1, "Found duplicate entry for task " << tskObjPolar);
          return false;
        }
        tskObjPolar = ti->getName();
      }

      // taskHandObjPolar
      if ((efName==hand) && (refName== object) && (className=="POLAR"))
      {
        if (!tskHandObjPolar.empty())
        {
          RLOG_CPP(1, "Found duplicate entry for task " << tskHandObjPolar);
          return false;
        }
        tskHandObjPolar = ti->getName();
      }
    }

    // Check that all tasks have been found
    if (tskObjHandPos.empty())
    {
      RLOG(1, "Didn't find XYZ task with effector=\"%s\" and refBdy=\"%s\"",
           object.c_str(), hand.c_str());
      success = false;
    }

    if (tskObjSurfacePosX.empty())
    {
      RLOG(1, "Didn't find X task with effector=\"%s\" and refBdy=\"%s\"",
           object.c_str(), surface.c_str());
      success = false;
    }

    if (tskObjSurfacePosY.empty())
    {
      RLOG(1, "Didn't find Y task with effector=\"%s\" and refBdy=\"%s\"",
           object.c_str(), surface.c_str());
      success = false;
    }

    if (tskObjSurfacePosZ.empty())
    {
      RLOG(1, "Didn't find Z task with effector=\"%s\" and refBdy=\"%s\"",
           object.c_str(), surface.c_str());
      success = false;
    }

    if (tskObjPolar.empty())
    {
      RLOG(1, "Didn't find POLAR task with effector=\"%s\"", object.c_str());
      success = false;
    }

    if (tskHandObjPolar.empty())
    {
      RLOG(1, "Didn't find POLAR task with effector=\"%s\" and refBdy=\"%s\"",
           hand.c_str(), object.c_str());
      success = false;
    }

    if (success)
    {
      hand_ = hand;
      object_ = object;
      surface_ = surface;
      tskObjHandPos_ = tskObjHandPos;
      tskHandObjPolar_ = tskHandObjPolar;
      tskObjSurfacePosX_ = tskObjSurfacePosX;
      tskObjSurfacePosY_ = tskObjSurfacePosY;
      tskObjSurfacePosZ_ = tskObjSurfacePosZ;
      tskObjPolar_ = tskObjPolar;
    }

    return success;
  }

  std::shared_ptr<tropic::ConstraintSet> put(double t_start, double t_put, double t_release) const
  {
    // Grasp the bottle and lift it up
    auto a1 = std::make_shared<tropic::ActivationSet>();

    // Put object on surface
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosX);
    a1->addActivation(t_put, false, 0.5, taskObjSurfacePosX);
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosY);
    a1->addActivation(t_put, false, 0.5, taskObjSurfacePosY);

    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosZ);
    a1->addActivation(t_put, false, 0.5, taskObjSurfacePosZ);
    a1->add(t_put, 0.0, 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");
    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_put, objectName, surfaceName));

    // Retract hand from object
    a1->addActivation(t_put, true, 0.5, taskObjHandPos);
    a1->addActivation(t_release, false, 0.5, taskObjHandPos);
    a1->add(t_release, 0.2, 0.0, 0.0, 7, taskObjHandPos + " 0");

    // Object orientation wrt world frame
    a1->addActivation(t_start, true, 0.5, taskObjPolar);
    a1->addActivation(t_put, false, 0.5, taskObjPolar);
    a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjPolar));

    return a1;
  }

  static std::shared_ptr<tropic::ConstraintSet>
  pourWithTwoHands(const Rcs::ControllerBase* controller,
                   std::string rHand,
                   std::string lHand,
                   std::string bottle,
                   std::string glas,
                   std::string table,
                   std::string bottleTip,
                   std::string glasTip,
                   double t_start, double t_end)
  {
    const double graspHeight = 0.1;
    const double duration = t_end - t_start;
    const double t_tilt = t_start + 0.25*duration;
    const double t_put = t_start + 0.75*duration;
    auto a1 = std::make_shared<tropic::ActivationSet>();

    LiftObjectConstraint rh(controller, rHand, bottle, table);
    LiftObjectConstraint lh(controller, lHand, glas, table);

    double t_mid = t_start+0.5*(t_tilt-t_start);
    a1->add(rh.lift(t_start, t_mid, t_tilt, graspHeight));
    a1->add(lh.lift(t_start, t_mid, t_tilt, graspHeight));

    t_mid = t_tilt+0.5*(t_put-t_tilt);
    a1->add(rh.tilt(controller, bottleTip, glasTip, t_tilt, 5.0, t_put));

    t_mid = t_put+0.5*(t_end-t_put);
    a1->add(rh.put(t_put, t_mid, t_end));
    a1->add(lh.put(t_put, t_mid, t_end));

    return a1;
  }

  std::shared_ptr<tropic::ConstraintSet> lift(double t_start, double t_grasp, double t_end, double graspHeight) const
  {
    // Grasp the bottle and lift it up
    auto a1 = std::make_shared<tropic::ActivationSet>();

    // Hand position with respect to bottle
    const double t_pregrasp = t_start + 0.5*(t_grasp-t_start);
    a1->addActivation(t_start, true, 0.5, taskObjHandPos);
    a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
    a1->add(t_pregrasp, 0.2, 0.0, 0.0, 1, taskObjHandPos + " 0");
    a1->add(std::make_shared<tropic::PositionConstraint>(t_grasp, 0.0, 0.0, -graspHeight, taskObjHandPos));

    // Hand orientation with respect to bottle
    a1->addActivation(t_start, true, 0.5, taskHandObjPolar);
    a1->addActivation(t_grasp, false, 0.5, taskHandObjPolar);
    a1->add(std::make_shared<tropic::PolarConstraint>(t_grasp, 0.0, 0.0, taskHandObjPolar));

    // Object orientation with respect to world frame
    a1->addActivation(t_grasp, true, 0.5, taskObjPolar);
    a1->addActivation(t_end, false, 0.5, taskObjPolar);
    a1->add(std::make_shared<tropic::PolarConstraint>(t_end, 0.0, 0.0, taskObjPolar));

    // Lift object
    a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePosZ);
    a1->addActivation(t_end, false, 0.5, taskObjSurfacePosZ);
    a1->add(t_end, 0.2, 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");
    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, handName));

    return a1;
  }

  std::shared_ptr<tropic::ConstraintSet> tilt(const Rcs::ControllerBase* controller,
                                              std::string objToPourFrom,
                                              std::string objToPourInto,
                                              double t_start, double t_pour, double t_end) const
  {
    std::string taskRelPos, taskRelOri, taskObjPolar;

    // We go through the following looku+ps to resolve the names of possible
    // generic bodies.
    const RcsBody* bdy;
    bdy = RcsGraph_getBodyByName(controller->getGraph(), objToPourFrom.c_str());
    if (!bdy)
    {
      throw (std::string("Failed to find body for " + objToPourFrom));
    }
    objToPourFrom = std::string(bdy->name);

    bdy = RcsGraph_getBodyByName(controller->getGraph(), objToPourInto.c_str());
    if (!bdy)
    {
      throw (std::string("Failed to find body for " + objToPourInto));
    }
    objToPourInto = std::string(bdy->name);

    for (size_t i=0; i<controller->getNumberOfTasks(); ++i)
    {
      const Rcs::Task* ti = controller->getTask(i);

      // taskObjHandPos
      if ((getEffectorName(ti)==objToPourFrom && getRefBodyName(ti)==objToPourInto) ||
          (getEffectorName(ti)==objToPourInto && getRefBodyName(ti)==objToPourFrom))
      {

        if (ti->getClassName()=="XYZ")
        {
          RCHECK(taskRelPos.empty());
          taskRelPos = ti->getName();
        }
        else if (ti->getClassName()=="Composite")
        {
          RCHECK(taskRelOri.empty());
          taskRelOri = ti->getName();
        }

      }

      // taskObjPolar
      if ((getEffectorName(ti)==objToPourInto) &&
          (getRefBodyName(ti).empty()) &&
          (ti->getClassName()=="POLAR"))
      {
        RCHECK(taskObjPolar.empty());
        taskObjPolar = ti->getName();
      }

    }

    // Check that all tasks have been found
    RCHECK_MSG(!taskRelPos.empty(),
               "Didn't find position task with effector=\"%s\" and refBdy=\"%s\"",
               objectName.c_str(), handName.c_str());
    RCHECK_MSG(!taskRelOri.empty(),
               "Didn't find Polar angle task with effector=\"%s\" and refBdy=\"%s\"",
               objectName.c_str(), surfaceName.c_str());
    RCHECK_MSG(!taskObjPolar.empty(),
               "Didn't find Polar angle task with effector=\"%s\"",
               objectName.c_str());

    auto a1 = std::make_shared<tropic::ActivationSet>();

    // Hand position with respect to bottle. We move the bottle tip over the
    // glas tip, keep it a little bit (while the bottle tilts), and then
    // move bottle and glas sideways apart.
    a1->addActivation(t_start, true, 0.5, taskRelPos);
    a1->addActivation(t_end, false, 0.5, taskRelPos);
    double dur = t_pour - t_start;
    a1->add(std::make_shared<tropic::PositionConstraint>(t_pour-0.25*dur, 0.0, 0.0, 0.0, taskRelPos));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_pour+0.25*dur, 0.0, 0.0, 0.0, taskRelPos));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.0, -0.2, 0.0, taskRelPos));

    a1->addActivation(t_start, true, 0.5, taskRelOri);
    a1->addActivation(t_end, false, 0.5, taskRelOri);
    a1->add(t_pour-0.25*dur, RCS_DEG2RAD(80.0), 0.0, 0.0, 7, taskRelOri  + " 0");
    a1->add(t_pour+0.25*dur, RCS_DEG2RAD(150.0), 0.0, 0.0, 7, taskRelOri  + " 0");
    a1->add(t_end, RCS_DEG2RAD(10.0), 0.0, 0.0, 7, taskRelOri  + " 0");
    a1->add(t_pour, RCS_DEG2RAD(0.0), 0.0, 0.0, 7, taskRelOri  + " 1");
    a1->add(t_end, RCS_DEG2RAD(0.0), 0.0, 0.0, 7, taskRelOri  + " 1");

    a1->addActivation(t_start, true, 0.5, taskObjPolar);
    a1->addActivation(t_end, false, 0.5, taskObjPolar);

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

  static std::string getEffectorName(const Rcs::Task* tsk)
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

  static std::string getRefBodyName(const Rcs::Task* tsk)
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
  std::string taskObjSurfacePosZ;
  std::string taskObjSurfacePosX;
  std::string taskObjSurfacePosY;
  std::string taskObjPolar;
  std::string taskHandObjPolar;

  std::string handName;
  std::string objectName;
  std::string surfaceName;
};

}   // namespace tropic


#endif   // TROPIC_LIFTOBJECTCONSTRAINT_H
