/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "ExampleTrajectoryIK.h"
#include "PositionTrajectoryNode.h"
#include "ActivationSet.h"
#include "PositionTrajectoryNode.h"
#include "MultiGoalConstraint.h"
#include "ConstraintFactory.h"
#include "LiftObject.h"
#include "PouringConstraint.h"

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <IkSolverConstraintRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_kinematics.h>
#include <Rcs_utilsCPP.h>

#include <KeyCatcher.h>


void TropicExampleInfo()
{
  Rcs::ExampleFactory::print();
}

namespace tropic
{

static Rcs::ExampleFactoryRegistrar<ExampleTrajectoryIK> ExampleTrajectoryIK_("Trajectory", "InverseKinematics");

ExampleTrajectoryIK::ExampleTrajectoryIK(int argc, char** argv) :
  Rcs::ExampleBase(argc, argv)
{
  algo = 1;
  loopCount = 0;
  alpha = 0.05;
  lambda = 0.0;
  dt = 0.01;
  dt_calc = 0.0;
  determinant = 0.0;
  jlCost = 0.0;
  dJlCost = 0.0;
  horizon = 2.0;
  calcDistance = true;

  pause = false;
  launchJointWidget = false;
  manipulability = false;
  cAvoidance = false;
  constraintIK = false;

  valgrind = false;
  simpleGraphics = false;
  zigzag = false;
  showOnly = true;
  noTaskGui = false;
  permissive = false;
  nomutex = false;
  showTimingsGui = false;

  effortBdyId = -1;

  pthread_mutex_init(&graphLock, NULL);
  mtx = &graphLock;

  controller = NULL;
  ikSolver = NULL;

  dq_des     = NULL;
  q_dot_des  = NULL;
  a_des      = NULL;
  x_curr     = NULL;
  x_des      = NULL;
  x_des_prev = NULL;
  x_des_f    = NULL;
  dx_des     = NULL;
  dH         = NULL;
  timings    = NULL;
  F_effort   = NULL;

  cGui = NULL;
  jGui = NULL;
  mGuiTimings = NULL;
  mGuiEffort = NULL;

  viewer = NULL;
  hudText[0] = '\0';
}


ExampleTrajectoryIK::~ExampleTrajectoryIK()
{
  clear();
  pthread_mutex_destroy(&graphLock);
  Rcs_removeResourcePath(directory.c_str());
}

void ExampleTrajectoryIK::clear()
{
  if (valgrind == false)
  {
    delete viewer;
    viewer = NULL;
    delete cGui;
    cGui = NULL;
    delete jGui;
    jGui = NULL;
    delete mGuiTimings;
    mGuiTimings = NULL;
    delete mGuiEffort;
    mGuiEffort = NULL;
  }

  MatNd_destroy(dq_des);
  MatNd_destroy(q_dot_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x_curr);
  MatNd_destroy(x_des);
  MatNd_destroy(x_des_prev);
  MatNd_destroy(x_des_f);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(timings);
  MatNd_destroy(F_effort);

  dq_des     = NULL;
  q_dot_des  = NULL;
  a_des      = NULL;
  x_curr     = NULL;
  x_des      = NULL;
  x_des_prev = NULL;
  x_des_f    = NULL;
  dx_des     = NULL;
  dH         = NULL;
  timings    = NULL;
  F_effort   = NULL;

  if (tc)
  {
    tc->takeControllerOwnership(false);
  }
}

bool ExampleTrajectoryIK::initParameters()
{
  xmlFileName = "cPouring.xml";
  directory = "config/xml/Tropic";

  return true;
}

bool ExampleTrajectoryIK::parseArgs(Rcs::CmdLineParser* argP)
{
  argP->getArgument("-horizon", &horizon, "Trajectory horizon (default is %f)",
                    horizon);
  argP->getArgument("-algo", &algo, "IK algorithm: 0: left inverse, 1: "
                    "right inverse (default is %d)", algo);
  argP->getArgument("-alpha", &alpha,
                    "Null space scaling factor (default is %f)", alpha);
  argP->getArgument("-lambda", &lambda, "Regularization (default is %f)",
                    lambda);
  argP->getArgument("-f", &xmlFileName, "Configuration file name (default "
                    "is \"%s\")", xmlFileName.c_str());
  argP->getArgument("-dir", &directory, "Configuration file directory "
                    "(default is \"%s\")", directory.c_str());
  argP->getArgument("-dt", &dt, "Sampling time interval (default is %f)", dt);
  argP->getArgument("-staticEffort", &effortBdyName,
                    "Body to map static effort (default: none)");
  argP->getArgument("-pause", &pause, "Pause after each iteration");
  argP->getArgument("-jointWidget", &launchJointWidget,
                    "Launch JointWidget");
  argP->getArgument("-manipulability", &manipulability,
                    "Manipulability criterion in "
                    "null space");
  argP->getArgument("-ca", &cAvoidance, "Collision avoidance in null space");
  argP->getArgument("-constraintIK", &constraintIK, "Use constraint IK solver");
  argP->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");
  argP->getArgument("-simpleGraphics", &simpleGraphics, "OpenGL without fancy"
                    " stuff (shadows, anti-aliasing)");
  argP->getArgument("-zigzag", &zigzag, "ZigZag trajectory");
  argP->getArgument("-passiveGui", &showOnly, "Gui shows values only");
  argP->getArgument("-noGui", &noTaskGui, "No task Gui");
  argP->getArgument("-permissive", &permissive, "No pedantic check for "
                    "validity of ConstraintSet instances");
  argP->getArgument("-showTimings", &showTimingsGui, "Show Gui for timings");

  return true;
}

bool ExampleTrajectoryIK::initAlgo()
{
  Rcs_addResourcePath(directory.c_str());

  if (nomutex)
  {
    mtx = NULL;
  }

  // Create controller
  controller = new Rcs::ControllerBase(xmlFileName);

  if (constraintIK==true)
  {
    ikSolver = new Rcs::IkSolverConstraintRMR(controller);
  }
  else
  {
    ikSolver = new Rcs::IkSolverRMR(controller);
  }

  dq_des     = MatNd_create(controller->getGraph()->dof, 1);
  q_dot_des  = MatNd_create(controller->getGraph()->dof, 1);
  a_des      = MatNd_create(controller->getNumberOfTasks(), 1);
  x_curr     = MatNd_create(controller->getTaskDim(), 1);
  x_des      = MatNd_create(controller->getTaskDim(), 1);
  x_des_prev = MatNd_create(controller->getTaskDim(), 1);
  x_des_f    = MatNd_create(controller->getTaskDim(), 1);
  dx_des     = MatNd_create(controller->getTaskDim(), 1);
  dH         = MatNd_create(1, controller->getGraph()->nJ);
  timings    = MatNd_create(5, 1);
  MatNd_set(timings, 0, 0, 1.0);
  MatNd_set(timings, 1, 0, 20.0);

  controller->readActivationsFromXML(a_des);
  controller->computeX(x_curr);
  MatNd_copy(x_des, x_curr);
  MatNd_copy(x_des_f, x_curr);
  MatNd_copy(x_des_prev, x_curr);

  if (zigzag)
  {
    tc = std::make_shared<tropic::TrajectoryController<tropic::ZigZagTrajectory1D>>(controller, horizon);
  }
  else
  {
    tc = std::make_shared<tropic::TrajectoryController<tropic::ViaPointTrajectory1D>>(controller, horizon);
  }

  tc->setTurboMode(false);
  tc->takeControllerOwnership(true);


  // Body for static effort null space gradient
  const RcsBody* effortBdy = RcsGraph_getBodyByName(controller->getGraph(),
                                                    effortBdyName.c_str());
  effortBdyId = effortBdy ? effortBdy->id : -1;
  F_effort = MatNd_create(4, 1);
  MatNd_reshape(F_effort, 3, 1);

  return true;
}

bool ExampleTrajectoryIK::initGraphics()
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("T", "Run controller test");
  Rcs::KeyCatcherBase::registerKey("p", "Toggle pause");
  Rcs::KeyCatcherBase::registerKey("a", "Change IK algorithm");
  Rcs::KeyCatcherBase::registerKey("D", "Write trajectory to file traj_out.xml");
  Rcs::KeyCatcherBase::registerKey("d", "Read trajectory from file traj.xml");
  Rcs::KeyCatcherBase::registerKey("n", "Reset to default state");
  Rcs::KeyCatcherBase::registerKey("C", "Toggle closest point lines");
  Rcs::KeyCatcherBase::registerKey("o", "Toggle distance calculation");
  Rcs::KeyCatcherBase::registerKey("m", "Manipulability null space");
  Rcs::KeyCatcherBase::registerKey("v", "Write current model_state to console");
  Rcs::KeyCatcherBase::registerKey("t", "Load trajectory from Johannes's cool class");
  Rcs::KeyCatcherBase::registerKey("l", "Bi-manual pouring");
  Rcs::KeyCatcherBase::registerKey("y", "One-handed pouring");
  Rcs::KeyCatcherBase::registerKey("O", "Create tasks");

  if (valgrind)
  {
    return true;
  }

  viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
  kc      = new Rcs::KeyCatcher();
  gn      = new Rcs::GraphNode(controller->getGraph());
  hud     = new Rcs::HUD();
  dragger = new Rcs::BodyPointDragger();
  dragger->scaleDragForce(0.01);
  viewer->add(gn);
  viewer->add(hud);
  viewer->add(kc);
  viewer->add(dragger);

  if (controller->getCollisionMdl() != NULL)
  {
    cn = new Rcs::VertexArrayNode(controller->getCollisionMdl()->cp,
                                  osg::PrimitiveSet::LINES, "RED");
    cn->toggle();
    viewer->add(cn);
  }

  RCHECK(tc->getController());
  for (size_t i=0; i<tc->getController()->getNumberOfTasks(); ++i)
  {
    RCHECK(tc->getTrajectory(i));
    if (STREQ(tc->getTrajectory(i)->getClassName(), "TrajectoryPos3D") &&
        (!tc->getController()->getTask(i)->getRefBody()))
    {
      tropic::PositionTrajectoryNode* pn = new tropic::PositionTrajectoryNode(tc->getTrajectory(i));
      //pn->setPointSize(10.0);
      viewer->add(pn);
    }
  }

  viewer->runInThread(mtx);

  return true;
}

bool ExampleTrajectoryIK::initGuis()
{
  if (valgrind)
  {
    return true;
  }

  // Launch the task widget
  if (!noTaskGui)
  {
    if (showOnly)
    {
      cGui = new Rcs::ControllerGui(controller, a_des,
                                    x_des_f, x_curr, mtx, true);
    }
    else
    {
      cGui = new Rcs::ControllerGui(controller, a_des, x_des,
                                    x_curr, mtx, false);
    }
  }

  if (launchJointWidget==true)
  {
    jGui = new Rcs::JointGui(controller->getGraph(), mtx);
  }

  if (effortBdyId != -1)
  {
    std::vector<std::string> labels;
    mGuiEffort = new Rcs::MatNdGui(F_effort, F_effort, -1.0, 1.0,
                                   "F_effort", mtx);
    labels.push_back("Fx");
    labels.push_back("Fy");
    labels.push_back("Fz");
    labels.push_back("gain");
    mGuiEffort->setLabels(labels);
  }

  if (showTimingsGui)
  {
    std::vector<std::string> labels;
    mGuiTimings = new Rcs::MatNdGui(timings, timings, 0.0, 40.0,
                                    "Timings", mtx);
    labels.push_back("t0");
    labels.push_back("t1");
    labels.push_back("t2");
    labels.push_back("t3");
    labels.push_back("t4");
    mGuiTimings->setLabels(labels);
  }

  return true;
}

void ExampleTrajectoryIK::step()
{
  pthread_mutex_lock(&graphLock);

  dt_calc = Timer_getTime();

  //////////////////////////////////////////////////////////////////
  // Step trajectories
  //////////////////////////////////////////////////////////////////
  MatNd* h1 = MatNd_clone(x_des_prev);
  MatNd* h2 = MatNd_clone(x_des);
  controller->compressToActiveSelf(h1, a_des);
  controller->compressToActiveSelf(h2, a_des);
  bool guiChanged = !MatNd_isEqual(h1, h2, 1.0e-8);
  MatNd_destroy(h1);
  MatNd_destroy(h2);

  if (guiChanged && (!showOnly))
  {
    auto tVec = tc->getTrajectories();
    for (size_t i=0; i<tVec.size(); ++i)
    {
      tVec[i]->removeConstraintsAfter(0.5*horizon);
    }

    auto guiConstraint = std::make_shared<tropic::MultiGoalConstraint>(horizon, x_des, tc->getTrajectories());
    tc->addAndApply(guiConstraint, permissive);
  }

  MatNd_copy(x_des_prev, x_des);

  double endTime = tc->step(dt);
  tc->getPosition(0.0, x_des_f);

  if (showOnly || noTaskGui)
  {
    tc->getActivation(a_des);
  }

  controller->computeDX(dx_des, x_des_f);
  double clipLimit = 0.1;
  MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
  MatNd_saturateSelf(dx_des, &clipArr);

  controller->computeJointlimitGradient(dH);

  if (calcDistance==true)
  {
    controller->computeCollisionCost();
  }

  if (cAvoidance==true)
  {
    MatNd* dH_ca = MatNd_create(1, controller->getGraph()->dof);
    controller->getCollisionGradient(dH_ca);
    RcsGraph_limitJointSpeeds(controller->getGraph(), dH_ca,
                              1.0, RcsStateIK);
    MatNd_constMulSelf(dH_ca, 0.01);
    MatNd_addSelf(dH, dH_ca);
    MatNd_destroy(dH_ca);
  }

  if (manipulability)
  {
    MatNd_setZero(dH);
    controller->computeManipulabilityGradient(dH, a_des);
    MatNd_constMulSelf(dH, 100.0);
  }

  if (effortBdyId!=-1)
  {
    const RcsBody* effortBdy = RCSBODY_BY_ID(controller->getGraph(), effortBdyId);

    MatNd* W_ef = MatNd_create(controller->getGraph()->dof, 1);
    RCSGRAPH_TRAVERSE_JOINTS(controller->getGraph())
    {
      W_ef->ele[JNT->jointIndex] = 1.0/JNT->maxTorque;
    }

    RcsGraph_stateVectorToIKSelf(controller->getGraph(), W_ef);
    MatNd* effortGrad = MatNd_create(1, controller->getGraph()->nJ);
    RcsGraph_staticEffortGradient(controller->getGraph(), effortBdy,
                                  F_effort, W_ef, NULL, effortGrad);
    MatNd_destroy(W_ef);
    MatNd_reshape(F_effort, 4, 1);
    MatNd_constMulSelf(effortGrad, 1000.0*MatNd_get(F_effort, 3, 0));
    MatNd_reshape(F_effort, 3, 1);
    MatNd_addSelf(dH, effortGrad);

    MatNd_destroy(effortGrad);
  }

  MatNd_constMulSelf(dH, alpha*tc->computeBlending());

  if (valgrind==false)
  {
    dragger->addJointTorque(dH, controller->getGraph());
  }

  switch (algo)
  {
    case 0:
      determinant = ikSolver->solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
      break;

    case 1:
    {
      determinant = ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
    }
    break;

    default:
      RFATAL("No such algorithm; %d", algo);
  }

  MatNd_constMul(q_dot_des, dq_des, 1.0/dt);

  MatNd_addSelf(controller->getGraph()->q, dq_des);
  RcsGraph_setState(controller->getGraph(), NULL, q_dot_des);
  bool poseOK = controller->checkLimits();
  controller->computeX(x_curr);

  dJlCost = -jlCost;
  jlCost = controller->computeJointlimitCost();
  dJlCost += jlCost;

  dt_calc = Timer_getTime() - dt_calc;

  pthread_mutex_unlock(&graphLock);


  MatNd F_effort3 = MatNd_fromPtr(3, 1, F_effort->ele);
  const RcsBody* effortBdy = RCSBODY_BY_ID(controller->getGraph(), effortBdyId);

  sprintf(hudText, "IK calculation: %.1f us\ndof: %d nJ: %d "
          "nqr: %d nx: %d\nJL-cost: %.6f dJL-cost: %.6f %s %s"
          "\nalgo: %d lambda:%g alpha: %g\n"
          "Manipulability index: %.6f\n"
          "Static effort: %.6f\n"
          "Robot pose %s   Constraints: %d\nend time: %.3f",
          1.0e6*dt_calc, controller->getGraph()->dof,
          controller->getGraph()->nJ, ikSolver->getInternalDof(),
          (int) controller->getActiveTaskDim(a_des),
          jlCost, dJlCost,
          determinant==0.0?"SINGULAR":"",
          ((dJlCost > 1.0e-8) && (MatNd_getNorm(dx_des) == 0.0)) ?
          "COST INCREASE" : "",
          algo, lambda, alpha,
          controller->computeManipulabilityCost(a_des),
          RcsGraph_staticEffort(controller->getGraph(),
                                effortBdy, &F_effort3, NULL, NULL),
          poseOK ? "VALID" : "VIOLATES LIMITS",
          (int) tc->getNumberOfSetConstraints(), endTime);

  if (hud != NULL)
  {
    hud->setText(hudText);
  }
  else
  {
    std::cout << hudText;
  }

  if ((valgrind==true) && (loopCount>10))
  {
    runLoop = false;
  }

  if (pause==true || determinant==0.0)
  {
    RPAUSE();
  }

  loopCount++;
  Timer_waitDT(dt);
}

void ExampleTrajectoryIK::handleKeys()
{
  if (!kc.valid())
  {
    return;
  }

  if (kc->getAndResetKey('q'))
  {
    runLoop = false;
  }
  else if (kc->getAndResetKey('a'))
  {
    algo++;
    if (algo>1)
    {
      algo = 0;
    }

    RLOGS(0, "Switching to IK algorithm %d", algo);
  }
  else if (kc->getAndResetKey('T'))
  {
    RLOGS(0, "Running controller test");
    controller->test(true);
  }
  else if (kc->getAndResetKey('O'))
  {
    RLOGS(0, "Creating tasks");
    std::vector<std::string> fingers = {"j2s7s300_joint_finger_1_R",
                                        "j2s7s300_joint_finger_2_R",
                                        "j2s7s300_joint_finger_3_R"
                                       };
    auto tasks = tropic::LiftObjectConstraint::createLiftPutTasks(tc->getController()->getGraph(),
                                                                  "GenericBody0",
                                                                  "GenericBody3",
                                                                  "GenericBody2", fingers);

    for (auto const t : tasks)
    {
      t->print();
    }

    Rcs::ControllerBase c(RcsGraph_clone(tc->getController()->getGraph()));

    for (auto const t : tasks)
    {
      c.add(t);
    }

    c.toXML("cTestMe.xml");

  }
  else if (kc->getAndResetKey('p'))
  {
    pause = !pause;
    RMSG("Pause modus is %s", pause ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('d'))
  {
    auto ts = tropic::ConstraintFactory::create("traj.xml");

    if (!ts)
    {
      RMSG("Failed to load trajectory from file \"traj.xml\"");
    }
    else
    {
      bool success = tc->addAndApply(ts, permissive);
      RMSG("%s loading trajectory from file \"traj.xml\"",
           success ? "Success" : "Failure");
      ts->print();
    }
  }
  else if (kc->getAndResetKey('D'))
  {
    RMSG("Writing trajectory to \"traj_out.dat\"");
    tc->toXML("traj_out.xml");
  }
  else if (kc->getAndResetKey('t'))
  {
    // RMSG("Loading Johannes's class");
    // auto ts = std::make_shared<tropic::PouringConstraint>();
    // tc->addAndApply(ts, permissive);
    // ts->print();
    // tc->toXML("traj_out.xml");
    // RMSG("Done loading Johannes's class");
    RMSG("Loading trajectory from file. Please enter file name:");
    std::string tFile;
    std::cin >> tFile;
    std::shared_ptr<ConstraintSet> ts = tropic::ConstraintFactory::create(tFile);
    tc->addAndApply(ts, permissive);
  }
  else if (kc->getAndResetKey('u'))
  {
    RMSG("Swapping Generic Bodies");
    static bool swapObject = false;
    if (swapObject)
    {
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 3, "Bottle");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 6, "BottleTip");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 4, "Glas");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 7, "GlasTip");
    }
    else
    {
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 4, "Bottle");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 7, "BottleTip");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 3, "Glas");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 6, "GlasTip");
    }
    swapObject = !swapObject;
    RMSG("Done swapping Generic Bodies for object");
    tc->getInternalController()->toXML("controller->xml");
  }
  else if (kc->getAndResetKey('U'))
  {
    RMSG("Swapping Generic Bodies for hands");
    static bool swapHands = false;
    swapHands = !swapHands;
    if (swapHands)
    {
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 1, "PowerGrasp_R");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 0, "PowerGrasp_L");
    }
    else
    {
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 0, "PowerGrasp_R");
      RcsGraph_linkGenericBody(tc->getInternalController()->getGraph(), 1, "PowerGrasp_L");
    }
    RMSG("Done swapping hands");
    tc->getInternalController()->toXML("controller->xml");
  }
  else if (kc->getAndResetKey('l'))
  {
    auto tSet = tropic::LiftObjectConstraint::pourWithTwoHands(tc->getController(),
                                                               "GenericBody0",
                                                               "GenericBody1",
                                                               "GenericBody3",
                                                               "GenericBody4",
                                                               "GenericBody2",
                                                               "GenericBody6",
                                                               "GenericBody7",
                                                               timings->ele[0], timings->ele[1]);
    tc->addAndApply(tSet, permissive);
    tc->toXML("traj_out.xml");
  }
  else if (kc->getAndResetKey('P'))
  {
    auto tSet = tropic::LiftObjectConstraint::putWithOneHand(tc->getController(),
                                                             "GenericBody0",
                                                             "GenericBody3",
                                                             "GenericBody2",
                                                             0.0, 6.0);
    tc->addAndApply(tSet, permissive);
    tc->toXML("traj_out.xml");
  }
  else if (kc->getAndResetKey('y'))
  {
    auto tSet = tropic::LiftObjectConstraint::pourWithOneHand(tc->getController(),
                                                              "GenericBody0",
                                                              "GenericBody3",
                                                              "GenericBody4",
                                                              "GenericBody2",
                                                              "GenericBody6",
                                                              "GenericBody7", 1.0, 8.0);
    tc->addAndApply(tSet, permissive);
  }
  else if (kc->getAndResetKey('n'))
  {
    RMSG("Resetting");
    RcsGraph_setDefaultState(controller->getGraph());
    tc->clear(true);
    tc->setActivation(false);
    MatNd_setZero(a_des);
    controller->computeX(x_curr);
    MatNd_copy(x_des, x_curr);
    MatNd_copy(x_des_f, x_curr);
    MatNd_copy(x_des_prev, x_curr);
  }
  else if (kc->getAndResetKey('C') && cn)
  {
    RMSG("Toggle closest points visualization");
    cn->toggle();
  }
  else if (kc->getAndResetKey('o'))
  {
    calcDistance = !calcDistance;
    RMSG("Distance calculation is %s", calcDistance ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('m'))
  {
    manipulability = !manipulability;
    RMSG("Manipulation index nullspace is %s",
         manipulability ? "ON" : "OFF");
  }
  else if (kc->getAndResetKey('v'))
  {
    RcsGraph_fprintModelState(stdout, controller->getGraph(),
                              controller->getGraph()->q, NULL, 0);
  }
  else if (kc->getAndResetKey('t'))
  {
    // auto set1 = std::make_shared<tropic::ConstraintSet>();
    // auto set2 = std::make_shared<tropic::ActivationSet>();
    // auto ts = std::make_shared<tropic::ConnectBodyConstraint>(2.0, "Glas", "PowerGrasp_L");
    // set1->add(set2);
    // set2->add(ts);
    // tc->addAndApply(set1, permissive);

    // RLOG(0, "Adding Polar constraint in 2 seconds");
    // auto ts = std::make_shared<tropic::PolarConstraint>(2.0, 0.0, 0.0, "Polar_L");
    // tc->addAndApply(ts, permissive);

    // RLOG(0, "Adding PositionConstraint in 2 seconds");
    // auto ts = std::make_shared<tropic::PositionConstraint>(2.0, 0.0, 0.0, 0.0, "XYZ_L");
    // tc->addAndApply(ts, permissive);



    {
      RLOG(0, "Adding Polar constraint in 2 seconds");
      auto as = std::make_shared<tropic::ActivationSet>();
      as->addActivation(0.5, true, 0.5, "Polar_L");
      auto ts = std::make_shared<tropic::PolarConstraint>(2.0, 0.0, 0.0, "Polar_L");
      as->add(ts);
      tc->addAndApply(as, permissive);
    }

    {
      RLOG(0, "Adding PositionConstraint in 2 seconds");
      auto as = std::make_shared<tropic::ActivationSet>();
      as->addActivation(0.5, true, 0.5, "XYZ_L");
      auto ts = std::make_shared<tropic::PositionConstraint>(2.0, 0.55, 0.2, 1.0, "XYZ_L");
      as->add(ts);
      tc->addAndApply(as, permissive);
      as->print();
    }


  }
}

std::string ExampleTrajectoryIK::help()
{
  std::stringstream s;
  s << "  Trajectory IK test\n\n";
  s << "  L: Lift\n";
  s << "  P: Put\n";
  s << "  u: Swap objects\n";
  s << "  U: Swap hands\n";
  s << Rcs::ControllerBase::printUsageToString(xmlFileName);
  s << Rcs::getResourcePaths();
  s << Rcs::CmdLineParser::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  return s.str();
}

}   // namespace tropic
