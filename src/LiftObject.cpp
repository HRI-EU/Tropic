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

#include "LiftObject.h"
#include "ConstraintSet.h"
#include "PoseConstraint.h"
#include "PolarConstraint.h"
#include "ConnectBodyConstraint.h"
#include "VectorConstraint.h"
#include "EulerConstraint.h"
#include "ActivationSet.h"
#include "KeepPositionConstraint.h"

#include <TaskFactory.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>

#define fingersOpen   (0.01)
#define fingersClosed (0.7)
#define t_fingerMove  (1.5)


namespace tropic
{

LiftObjectConstraint::LiftObjectConstraint(const Rcs::ControllerBase* controller,
                                           const std::string& handName_,
                                           const std::string& objectName_,
                                           const std::string& surfaceName_) :
  handName(handName_),
  objectName(objectName_),
  surfaceName(surfaceName_)
{
  bool success = getLiftPutTasks(controller,
                                 handName,
                                 objectName,
                                 surfaceName,
                                 taskObjHandPos,
                                 taskHandObjPolar,
                                 taskObjSurfacePosX,
                                 taskObjSurfacePosY,
                                 taskObjSurfacePosZ,
                                 taskObjPolar,
                                 taskFingers);
  RCHECK(success);

  // Compute the transformation of the object with respect to the
  // support surface
  const RcsBody* table = RcsGraph_getBodyByName(controller->getGraph(),
                                                surfaceName.c_str());
  const RcsBody* obj = RcsGraph_getBodyByName(controller->getGraph(),
                                              objectName.c_str());

  RCHECK(table);
  RCHECK(obj);
  HTr_invTransform(&this->A_OT, &table->A_BI, &obj->A_BI);
}

// static
HTr LiftObjectConstraint::getGraspFrame(const Rcs::ControllerBase* controller,
                                        std::string object)
{
  HTr frm;
  HTr_setIdentity(&frm);

  const RcsBody* obj = RcsGraph_getBodyByName(controller->getGraph(),
                                              object.c_str());

  if (!obj)
  {
    RLOG(1, "Failed to find grasp frame - object %s not found in graph",
         object.c_str());
    return frm;
  }

  RCSBODY_TRAVERSE_SHAPES(obj)
  {
    if (SHAPE->type==RCSSHAPE_REFFRAME)
    {
      HTr_copy(&frm, &SHAPE->A_CB);
      return frm;
    }
  }

  RLOG(1, "Failed to find grasp frame - no frame shape in body %s",
       object.c_str());

  return frm;
}

// static
bool LiftObjectConstraint::getLiftPutTasks(const Rcs::ControllerBase* controller,
                                           std::string& hand_,
                                           std::string& object_,
                                           std::string& surface_,
                                           std::string& tskObjHandPos_,
                                           std::string& tskHandObjPolar_,
                                           std::string& tskObjSurfacePosX_,
                                           std::string& tskObjSurfacePosY_,
                                           std::string& tskObjSurfacePosZ_,
                                           std::string& tskObjPolar_,
                                           std::string& tskFingers_)
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

  const RcsBody* parent = RCSBODY_BY_ID(controller->getGraph(), bdy->parentId);
  if (parent)
  {
    //if (STREQ(parent->bdySuffix, "_L") || STREQ(parent->bdySuffix, "_R"))
    {
      //tskFingers_ = "Fingers" + std::string(parent->bdySuffix);
      tskFingers_ = hand + "_fingers";
    }
  }

  bdy = RcsGraph_getBodyByName(controller->getGraph(), object.c_str());
  if (!bdy)
  {
    RLOG_CPP(0, "Failed to find body for " << object);
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
        RLOG_CPP(0, "Found duplicate entry for task " << tskObjHandPos);
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
          RLOG_CPP(0, "Found duplicate entry for task " << tskObjSurfacePosX);
          return false;
        }
        tskObjSurfacePosX = ti->getName();
      }
      else if (className=="Y")
      {
        if (!tskObjSurfacePosY.empty())
        {
          RLOG_CPP(0, "Found duplicate entry for task " << tskObjSurfacePosY);
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
        RLOG_CPP(0, "Found duplicate entry for task " << tskObjPolar);
        return false;
      }
      tskObjPolar = ti->getName();
    }

    // taskHandObjPolar
    if ((efName==hand) && (refName== object) && (className=="POLAR"))
    {
      if (!tskHandObjPolar.empty())
      {
        RLOG_CPP(0, "Found duplicate entry for task " << tskHandObjPolar);
        return false;
      }
      tskHandObjPolar = ti->getName();
    }
  }

  // Check that all tasks have been found
  if (tskObjHandPos.empty())
  {
    RLOG(0, "Didn't find XYZ task with effector=\"%s\" and refBdy=\"%s\"",
         object.c_str(), hand.c_str());
    success = false;
  }

  if (tskObjSurfacePosX.empty())
  {
    RLOG(0, "Didn't find X task with effector=\"%s\" and refBdy=\"%s\"",
         object.c_str(), surface.c_str());
    success = false;
  }

  if (tskObjSurfacePosY.empty())
  {
    RLOG(0, "Didn't find Y task with effector=\"%s\" and refBdy=\"%s\"",
         object.c_str(), surface.c_str());
    success = false;
  }

  if (tskObjSurfacePosZ.empty())
  {
    RLOG(0, "Didn't find Z task with effector=\"%s\" and refBdy=\"%s\"",
         object.c_str(), surface.c_str());
    success = false;
  }

  if (tskObjPolar.empty())
  {
    RLOG(0, "Didn't find POLAR task with effector=\"%s\"", object.c_str());
    success = false;
  }

  if (tskHandObjPolar.empty())
  {
    RLOG(0, "Didn't find POLAR task with effector=\"%s\" and refBdy=\"%s\"",
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

std::vector<Rcs::Task*> LiftObjectConstraint::createLiftPutTasks(const RcsGraph* graph,
                                                                 const std::string& hand_,
                                                                 const std::string& object_,
                                                                 const std::string& surface_,
                                                                 std::vector<std::string> fingers)
{
  // We make a local copy of the body strings, since they might be resolved into
  // different names when being a GenericBody
  std::string hand = hand_, object = object_, surface = surface_;
  std::vector<Rcs::Task*> tasks;

  // We go through the following look-ups to resolve the names of possible
  // generic bodies.
  const RcsBody* bdy;
  bdy = RcsGraph_getBodyByName(graph, hand.c_str());
  if (!bdy)
  {
    RLOG_CPP(1, "Failed to find body for " << hand);
    return tasks;
  }
  hand = std::string(bdy->name);

  bdy = RcsGraph_getBodyByName(graph, object.c_str());
  if (!bdy)
  {
    RLOG_CPP(0, "Failed to find body for " << object);
    return tasks;
  }
  object = std::string(bdy->name);

  bdy = RcsGraph_getBodyByName(graph, surface.c_str());
  if (!bdy)
  {
    RLOG_CPP(1, "Failed to find body for " << surface);
    return tasks;
  }
  surface = std::string(bdy->name);

  std::string fingerTask;
  for (auto f : fingers)
  {
    const RcsJoint* jnt = RcsGraph_getJointByName(graph, f.c_str());
    if (!jnt)
    {
      RLOG_CPP(1, "Failed to find joint for " << f);
      return tasks;
    }
    fingerTask += f;
    fingerTask += " ";
  }


  // taskObjHandPos: XYZ-task with effector=object and refBdy=hand
  std::string xmlTask = "<Task name = \"" + object + "-" + hand + "-XYZ\" " +
                        "controlVariable=\"XYZ\" " +
                        "effector=\"" + object + "\" " +
                        "refBdy=\"" + hand + "\" />";
  tasks.push_back(Rcs::TaskFactory::createTask(xmlTask, graph));

  // taskObjSurfacePosition z and sideways velocities
  xmlTask = "<Task name = \"" + object + "-" + surface + "-X\" " +
            "controlVariable=\"X\" " +
            "effector=\"" + object + "\" " + "refBdy=\"" + surface + "\" />";
  tasks.push_back(Rcs::TaskFactory::createTask(xmlTask, graph));

  xmlTask = "<Task name = \"" + object + "-" + surface + "-Y\" " +
            "controlVariable=\"Z\" " +
            "effector=\"" + object + "\" " + "refBdy=\"" + surface + "\" />";
  tasks.push_back(Rcs::TaskFactory::createTask(xmlTask, graph));

  xmlTask = "<Task name = \"" + object + "-" + surface + "-Z\" " +
            "controlVariable=\"Y\" " +
            "effector=\"" + object + "\" " + "refBdy=\"" + surface + "\" />";
  tasks.push_back(Rcs::TaskFactory::createTask(xmlTask, graph));

  // taskObjPolar
  xmlTask = "<Task name = \"" + object + "-POLAR\" " +
            "controlVariable=\"POLAR\" " + "effector=\"" + object + "\" />";
  tasks.push_back(Rcs::TaskFactory::createTask(xmlTask, graph));

  // taskHandObjPolar
  xmlTask = "<Task name = \"" + hand + "-" + object + "-POLAR\" " +
            "controlVariable=\"POLAR\" " + "effector=\"" + hand + "\" " +
            "refBdy=\"" + object + "\" />";
  tasks.push_back(Rcs::TaskFactory::createTask(xmlTask, graph));

  // Fingers
  xmlTask = "<Task name = \"" + hand + "_fingers\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerTask + "\" />";
  tasks.push_back(Rcs::TaskFactory::createTask(xmlTask, graph));

  // Check that all constructed tasks are valid
  bool valid = true;
  for (size_t i = 0; i < tasks.size(); ++i)
  {
    if (!tasks[i])
    {
      valid = false;
      RLOG_CPP(0, "Task " << i << " is NULL");
    }
  }

  // If we found one or more NULL task, we delete all tasks and return an
  // empty vector
  if (!valid)
  {
    for (auto ti : tasks)
    {
      delete ti;   // deleting NULL is safe
    }

    tasks.clear();
  }

  return tasks;
}

std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::put(double t_start, double t_put, double t_release,
                          const HTr* putPose) const
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

  if (putPose)
  {
    const double* x = putPose->org;
    a1->add(t_put, x[0], 0.0, 0.0, 7, taskObjSurfacePosX + " 0");
    a1->add(t_put, x[1], 0.0, 0.0, 7, taskObjSurfacePosY + " 0");
    a1->add(t_put, x[2], 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");
  }
  else
  {
    a1->add(t_put, 0.0, 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");
  }

  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_put, objectName, surfaceName));

  // Retract hand from object
  a1->addActivation(t_put, true, 0.5, taskObjHandPos);
  a1->addActivation(t_release, false, 0.5, taskObjHandPos);
  a1->add(t_release+1, 0.2, 0.0, 0.0, 7, taskObjHandPos + " 0");

  // Object orientation wrt world frame. The object is re-connected to the
  // table at t=t_put. Therefore we must switch of the hand-object relative
  // orientation to avoid conflicting constraints. If we want the hand to
  // remain upright a bit longer, we would need to activate an orientation
  // task that is absolute with respect to the hand, for instance taskHandObjPolar.
  a1->addActivation(t_start, true, 0.5, taskObjPolar);
  a1->addActivation(t_put, false, 0.5, taskObjPolar);
  a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjPolar));
  a1->addActivation(t_put, true, 0.5, taskHandObjPolar);
  a1->addActivation(t_put + 0.5*(t_release-t_put), false, 0.5, taskHandObjPolar);


  // Open fingers
  a1->addActivation(t_release, false, 0.5, taskFingers);
  a1->add(std::make_shared<tropic::VectorConstraint>(t_put-0.5*t_fingerMove, std::vector<double> {fingersClosed, fingersClosed, fingersClosed}, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_put+0.5*t_fingerMove, std::vector<double> {fingersOpen, fingersOpen, fingersOpen}, taskFingers));

  return a1;
}

// static
std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::pourWithTwoHands(const Rcs::ControllerBase* controller,
                                       std::string rHand,
                                       std::string lHand,
                                       std::string bottle,
                                       std::string glas,
                                       std::string table,
                                       std::string bottleTip,
                                       std::string glasTip,
                                       double t_start, double t_end)
{
  HTr graspBottle = getGraspFrame(controller, bottle);
  HTr graspGlas = getGraspFrame(controller, glas);
  const double duration = t_end - t_start;
  const double t_lift = t_start + 0.33*duration;
  const double t_put = t_start + 0.75*duration;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  LiftObjectConstraint rh(controller, rHand, bottle, table);
  LiftObjectConstraint lh(controller, lHand, glas, table);

  const double t_grasp = t_start+0.66*(t_lift-t_start);
  const double liftHeight = 0.1;
  a1->add(rh.lift(t_start, t_grasp, t_lift, liftHeight, graspBottle.org[2]));
  a1->add(lh.lift(t_start, t_grasp, t_lift, liftHeight, graspGlas.org[2]));

  a1->add(rh.tilt(controller, bottleTip, glasTip, t_lift, t_put, true));

  const double t_release = t_put+0.66*(t_end-t_put);
  a1->add(rh.put(t_put, t_release, t_end, &rh.A_OT));
  a1->add(lh.put(t_put, t_release, t_end, &lh.A_OT));
  return a1;
}

// static
std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::screwAndPourWithTwoHands(const Rcs::ControllerBase* controller,
                                               std::string rHand,
                                               std::string lHand,
                                               std::string bottle,
                                               std::string glas,
                                               std::string table,
                                               std::string bottleTip,
                                               std::string glasTip,
                                               double t_start, double t_end)
{
  HTr graspBottle = getGraspFrame(controller, bottle);
  HTr graspGlas = getGraspFrame(controller, glas);
  const double duration = t_end - t_start;
  const double t_screw = t_start + 0.1*duration;   // Start screwing
  const double t_pour = t_start + 0.5*duration;    // Start pouring
  const double t_put = t_start + 0.8*duration;     // Start putting down
  auto a1 = std::make_shared<tropic::ActivationSet>();

  LiftObjectConstraint rh(controller, rHand, bottle, table);
  LiftObjectConstraint lh(controller, lHand, glas, table);

  const double t_grasp = t_start + 0.66*(t_screw - t_start);
  const double liftHeight = 0.1;
  a1->add(rh.lift(t_start, t_grasp, t_screw, liftHeight, graspBottle.org[2]));

  a1->add(rh.screw(controller, bottle, bottleTip, t_screw, t_pour));

  // Otherwise beteen t_pour and t_put, then hand will move away due to the null space
  a1->add(std::make_shared<tropic::KeepPositionConstraint>(t_pour+.11, t_put, 0.1, rh.taskObjSurfacePosX));
  a1->add(std::make_shared<tropic::KeepPositionConstraint>(t_pour+.11, t_put, 0.1, rh.taskObjSurfacePosY));

  a1->add(lh.lift(t_pour, t_pour+0.5*(t_put-t_pour), t_put, liftHeight, graspGlas.org[2]));

  a1->add(rh.tilt(controller, bottleTip, glasTip, t_put, t_end, true));

  const double t_end2 = t_end + 0.1*duration;
  const double t_release = t_end + 0.66*(t_end2 - t_end);
  a1->add(rh.put(t_end, t_release, t_end2, &rh.A_OT));
  a1->add(lh.put(t_end, t_release, t_end2, &lh.A_OT));

  return a1;
}

// static
std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::liftWithOneHand(const Rcs::ControllerBase* controller,
                                      std::string hand,
                                      std::string bottle,
                                      std::string table,
                                      double t_start, double t_end)
{
  HTr graspBottle = getGraspFrame(controller, bottle);
  const double liftHeight = 0.1;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  LiftObjectConstraint rh(controller, hand, bottle, table);

  const double t_grasp = t_start + 0.66*(t_end - t_start);
  a1->add(rh.lift(t_start, t_grasp, t_end, liftHeight, graspBottle.org[2]));

  a1->addActivation(t_end, true, 0.5, rh.taskObjPolar);
  //a1->addActivation(t_end, true, 0.5, "Bottle XYZ");// HACK

  return a1;
}

// static
std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::putWithOneHand(const Rcs::ControllerBase* controller,
                                      std::string hand,
                                      std::string bottle,
                                      std::string table,
                                      double t_start, double t_end)
{
  LiftObjectConstraint rh(controller, hand, bottle, table);
  const double t_put = t_start + 0.66 * (t_end - t_start);
  auto a1 = std::make_shared<tropic::ActivationSet>();
  a1->add(rh.put(t_start, t_put, t_end));

  return a1;
}

// static
std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::pourWithOneHand(const Rcs::ControllerBase* controller,
                                      std::string hand,
                                      std::string bottle,
                                      std::string glas,
                                      std::string table,
                                      std::string bottleTip,
                                      std::string glasTip,
                                      double t_start, double t_end)
{
  HTr graspBottle = getGraspFrame(controller, bottle);
  const double liftHeight = 0.1;
  const double duration = t_end - t_start;
  const double t_lift = t_start + 0.25*duration;
  const double t_put = t_start + 0.75*duration;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  LiftObjectConstraint rh(controller, hand, bottle, table);

  const double t_grasp = t_start+0.66*(t_lift-t_start);
  a1->add(rh.lift(t_start, t_grasp, t_lift, liftHeight, graspBottle.org[2]));

  a1->add(rh.tilt(controller, bottleTip, glasTip, t_lift, t_put, false));

  const double t_release = t_put+0.66*(t_end-t_put);
  a1->add(rh.put(t_put, t_release, t_end, &rh.A_OT));

  return a1;
}

// static
std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::screwWithTwoHands(const Rcs::ControllerBase* controller,
                                        std::string rHand,
                                        std::string lHand,
                                        std::string bottle,
                                        std::string glas,
                                        std::string table,
                                        std::string bottleTip,
                                        std::string glasTip,
                                        double t_start, double t_end)
{
  HTr graspBottle = getGraspFrame(controller, bottle);
  const double duration = t_end - t_start;
  const double t_screw = t_start + 0.15*duration;
  const double t_put = t_start + 0.85*duration;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  LiftObjectConstraint rh(controller, rHand, bottle, table);

  double t_grasp = t_start + 0.8*(t_screw - t_start);
  const double liftHeight = 0.1;
  a1->add(rh.lift(t_start, t_grasp, t_screw, liftHeight, graspBottle.org[2]));

  a1->add(rh.screw(controller, bottle, bottleTip, t_screw, t_put));

  double t_release = t_put + 0.8*(t_end - t_put);
  a1->add(rh.put(t_put, t_release, t_end, &rh.A_OT));

  return a1;
}

// static
std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::screwAndPourWithOneHand(const Rcs::ControllerBase* controller,
                                              std::string rHand,
                                              std::string lHand,
                                              std::string bottle,
                                              std::string glas,
                                              std::string table,
                                              std::string bottleTip,
                                              std::string glasTip,
                                              double t_start, double t_end)
{
  HTr graspBottle = getGraspFrame(controller, bottle);
  const double duration = t_end - t_start;
  const double t_screw = t_start + 0.1*duration;   // Start screwing
  const double t_pour = t_start + 0.7*duration;    // Start pouring
  const double t_put = t_start + 0.9*duration;     // Start putting down
  auto a1 = std::make_shared<tropic::ActivationSet>();

  LiftObjectConstraint rh(controller, rHand, bottle, table);

  double t_grasp = t_start + 0.8*(t_screw - t_start);
  const double liftHeight = 0.1;
  a1->add(rh.lift(t_start, t_grasp, t_screw, liftHeight, graspBottle.org[2]));

  a1->add(rh.screw(controller, bottle, bottleTip, t_screw, t_pour));

  a1->add(rh.tilt(controller, bottleTip, glasTip, t_pour, t_put, false));

  const double t_release = t_put + 0.66*(t_end - t_put);
  a1->add(rh.put(t_put, t_release, t_end, &rh.A_OT));

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::lift(double t_start,
                           double t_grasp,
                           double t_end,
                           double liftHeight,
                           double graspHeight) const
{
  // Grasp the bottle and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();

  // Hand position with respect to bottle
  const double t_pregrasp = t_start + 0.5*(t_grasp-t_start);
  a1->addActivation(t_start, true, 0.5, taskObjHandPos);
  a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
  a1->add(t_pregrasp, 0.2, 0.0, 0.0, 1, taskObjHandPos + " 0");// 20cm in front
  a1->add(t_pregrasp, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 1");// and centered
  a1->add(std::make_shared<tropic::PositionConstraint>(t_grasp, 0.0, 0.0, -graspHeight, taskObjHandPos));

  // Hand orientation with respect to bottle
  a1->addActivation(t_start, true, 0.5, taskHandObjPolar);
  a1->addActivation(t_grasp, false, 0.5, taskHandObjPolar);
  a1->add(std::make_shared<tropic::PolarConstraint>(t_grasp, 0.0, 0.0, taskHandObjPolar));

  // Object orientation with respect to world frame. We need to make sure
  // that the objects are a little bit inclined, so that they rotate into
  // a defined direction. Otherwise, the inclination task will lead to random
  // tilting directions.
  a1->addActivation(t_grasp, true, 0.5, taskObjPolar);
  a1->addActivation(t_end, false, 0.5, taskObjPolar);
  double tiltSgn = 1.0;
  if (fabs(graspHeight)< 0.09)
  {
    tiltSgn = -1.0;
  }
  a1->add(std::make_shared<tropic::PolarConstraint>(t_end, 0.05, tiltSgn*M_PI_2, taskObjPolar));

  // Lift object
  a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePosZ);
  a1->addActivation(t_end, false, 0.5, taskObjSurfacePosZ);
  a1->add(t_end+1, 0.2, 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, handName));

  // Lift it up in the y-z plane (keep forward position constant)
  a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_end, false, 0.5, taskObjSurfacePosX);

  // Reduce snap to null space y-position by moving up vertically
  a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePosY);
  a1->addActivation(t_end, false, 0.5, taskObjSurfacePosY);

  // Close fingers
  a1->addActivation(t_start, true, 0.5, taskFingers);
  a1->add(std::make_shared<tropic::VectorConstraint>(t_grasp-0.5*t_fingerMove, std::vector<double> {fingersOpen, fingersOpen, fingersOpen}, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_grasp+0.5*t_fingerMove, std::vector<double> {fingersClosed, fingersClosed, fingersClosed}, taskFingers));

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::tilt(const Rcs::ControllerBase* controller,
                           std::string objToPourFrom,
                           std::string objToPourInto,
                           double t_start, double t_end, bool bimanual) const
{
  std::string taskRelPos, taskRelOri, taskObjPolar;
  double t_pour = t_start + 0.5 * (t_end - t_start);

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
  const double d_separate = 0.3;// How much do hands go away from each other once pouring finished



  //a1->add(std::make_shared<tropic::PositionConstraint>(t_pour-0.5*dur, 0.0, 0.0, 0.1, taskRelPos, 1));
  a1->add(t_pour-0.5*dur, 0.15, 0.0, 0.0, 1, taskRelPos  + " 2");
  a1->add(t_pour-0.25*dur, 0.1, 0.0, 0.0, 1, taskRelPos  + " 2");




  //a1->add(std::make_shared<tropic::PositionConstraint>(t_pour-0.25*dur, 0.0, 0.0, 0.5, taskRelPos, 1));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_pour, 0.0, 0.0, 0.0, taskRelPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_pour+0.5*dur, 0.0, 0.0, 0.0, taskRelPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end+1, 0.0, -d_separate, 0.1, taskRelPos));// 0.1 is a little bit up

  a1->addActivation(t_start, true, 0.5, taskRelOri);
  a1->addActivation(t_end, false, 0.5, taskRelOri);
  a1->add(t_pour-0.25*dur, RCS_DEG2RAD(80.0), 0.0, 0.0, 1, taskRelOri  + " 0");
  a1->add(t_pour+0.25*dur, RCS_DEG2RAD(150.0), 0.0, 0.0, 7, taskRelOri  + " 0");
  a1->add(t_end+1, RCS_DEG2RAD(30.0), 0.0, 0.0, 7, taskRelOri  + " 0");
  a1->add(t_end+1, RCS_DEG2RAD(90.0), 0.0, 0.0, 7, taskRelOri  + " 1");

  if (bimanual)
  {
    // Glas orientation to be upright. This should not be constrained if the
    // glas is not lifted by a hand, but standing on the table. We can leave it
    // on if we add some regularization to the IK to be able to cope with
    // conflicting constraints.
    a1->addActivation(t_start, true, 0.5, taskObjPolar);
    a1->addActivation(t_end, false, 0.5, taskObjPolar);

    // The bottle should not move forward or backward during tilting
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosX);
    a1->addActivation(t_end, false, 0.5, taskObjSurfacePosX);
  }

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
LiftObjectConstraint::screw(const Rcs::ControllerBase* controller,
                            std::string objToScrewFrom,
                            std::string objToScrewOff,
                            double t_start, double t_end, bool screwOpen) const
{
  double duration = t_end - t_start;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  std::string twistingFingers;
  if (taskFingers == "Fingers_L")
  {
    twistingFingers = "Fingers_R";
  }
  else if (taskFingers == "Fingers_R")
  {
    twistingFingers = "Fingers_L";
  }
  else
  {
    RFATAL("Finger task error!");
  }

  a1->addActivation(t_start, true, 0.5, twistingFingers);
  a1->addActivation(t_start, true, 0.5, taskFingers);

  a1->addActivation(t_start, true, 0.5, "LH-BottleTip");   // XYZ relative hand - bottle
  a1->addActivation(t_end, false, 0.5, "LH-BottleTip");
  a1->addActivation(t_start, true, 0.5, "LH-BottleTip2");  // ABC relative hand - bottle
  a1->addActivation(t_end, false, 0.5, "LH-BottleTip2");

  // Keep bottle forwardly where it was picked up. This avoids bringing it too
  // close to the body and running into self-collisions
  a1->addActivation(t_start + 0.2*duration, true, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_end, false, 0.5, taskObjSurfacePosX);

#if 1
  // Keep forward position
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_end, false, 0.5, taskObjSurfacePosX);
  a1->add(std::make_shared<tropic::KeepPositionConstraint>(t_start + 0.5, t_end, 0.45, taskObjSurfacePosX));
#endif

#if 1
  // Keep vertical position
  a1->addActivation(t_start + 0*0.15*duration, true, 0.5, taskObjSurfacePosZ);
  a1->addActivation(t_end, false, 0.5, taskObjSurfacePosZ);
  a1->add(std::make_shared<tropic::KeepPositionConstraint>(t_start + 0.5, t_end, 0.5, taskObjSurfacePosZ));
#endif

#if 0
  // Keep Inclination
  a1->addActivation(t_start + 0.15*duration, true, 0.5, "Bottle Inclination");
  a1->addActivation(t_end, false, 0.5, "Bottle Inclination");
  a1->add(std::make_shared<tropic::KeepPositionConstraint>(t_start + 0.15*duration + 1, t_end, 1, "Bottle Inclination"));
#endif

#if 0
  // Keep Inclination
  a1->addActivation(t_start, true, 0.5, "Bottle Inclination");
  a1->addActivation(t_end, false, 0.5, "Bottle Inclination");
  a1->add(t_start + 0.15*duration, RCS_DEG2RAD(45.0), 0.0, 0.0, 7, "Bottle Inclination 0");
  a1->add(t_end, RCS_DEG2RAD(30.0), 0.0, 0.0, 7, "Bottle Inclination 0");
#endif

  /////////Srewing start ///////////////
  double direction = (screwOpen ? -1.0 : 1.0);
  const double screwAngle = direction*RCS_DEG2RAD(60.0);
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.15*duration, 0.0, 0.0, 0.0, "LH-BottleTip"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.2*duration, M_PI_2, screwAngle, -M_PI_2, "LH-BottleTip2"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.3*duration, M_PI_2, -screwAngle, -M_PI_2, "LH-BottleTip2"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.4*duration, M_PI_2, screwAngle, -M_PI_2, "LH-BottleTip2"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.5*duration, M_PI_2, -screwAngle, -M_PI_2, "LH-BottleTip2"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.6*duration, M_PI_2, screwAngle, -M_PI_2, "LH-BottleTip2"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.7*duration, M_PI_2, -screwAngle, -M_PI_2, "LH-BottleTip2"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.8*duration, M_PI_2, screwAngle, -M_PI_2, "LH-BottleTip2"));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_start+0.9*duration, M_PI_2, -screwAngle, -M_PI_2, "LH-BottleTip2"));

  /////////Avoiding collision between left hand and bottle tip/////////////
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.9*duration, 0.0, 0.0, 0.0, "LH-BottleTip"));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end+1, 0.0, 0.0, 0.2, "LH-BottleTip"));

  ///////Fingers////////////
  const double fingersClosedCap = 0.8;

  a1->add(std::make_shared<tropic::PositionConstraint>(t_start+0.15*duration, fingersOpen, fingersOpen, fingersOpen, twistingFingers));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_start+0.2*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start+0.3*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start+0.35*duration, fingersOpen, fingersOpen, fingersOpen, twistingFingers));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.4*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.5*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.55*duration, fingersOpen, fingersOpen, fingersOpen, twistingFingers));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.6*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.7*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.75*duration, fingersOpen, fingersOpen, fingersOpen, twistingFingers));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.8*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.9*duration, fingersClosedCap, fingersClosedCap, fingersClosedCap, twistingFingers));
  // This will open the fingers after the last turn, so that the cap would fall down.
  //a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.95*duration, fingersOpen, fingersOpen, fingersOpen, twistingFingers));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_start, fingersClosed, fingersClosed, fingersClosed, taskFingers));

  return a1;
}


LiftObjectConstraint::LiftObjectConstraint(xmlNode* node)
{
  RFATAL("Implement me");
}

LiftObjectConstraint::LiftObjectConstraint(const LiftObjectConstraint& other)
{
  RFATAL("Implement me");// ? is it needed?
}

LiftObjectConstraint* LiftObjectConstraint::clone() const
{
  return nullptr;
}

LiftObjectConstraint::~LiftObjectConstraint()
{
}

void LiftObjectConstraint::fromXML(xmlNode* node)
{
  RFATAL("Implement me");
}

// static
std::string LiftObjectConstraint::getEffectorName(const Rcs::Task* tsk)
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

// static
std::string LiftObjectConstraint::getRefBodyName(const Rcs::Task* tsk)
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

}   // namespace tropic
