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

#include "TrajectoryController.h"
#include "ActivationSet.h"
#include "PositionTrajectoryNode.h"
#include "MultiGoalConstraint.h"
#include "ConstraintFactory.h"
#include "IkSolverConstraintRMR.h"
#include "TrajectoryPlotter1D.h"
#include "PoseConstraint.h"
#include "AnchoredPositionConstraint.h"
#include "PolarConstraint.h"
#include "ConnectBodyConstraint.h"
#include "PouringConstraint.h"
#include "LiftObject.h"

#include <ControllerWidgetBase.h>
#include <MatNdWidget.h>
#include <JointWidget.h>
#include <TaskPosition3D.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_kinematics.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_parser.h>
#include <GraphNode.h>
#include <CapsuleNode.h>
#include <HUD.h>
#include <VertexArrayNode.h>
#include <KeyCatcher.h>
#include <RcsViewer.h>
#include <BodyPointDragger.h>
#include <Rcs_guiFactory.h>
#include <StackVec.h>
#include <SegFaultHandler.h>

#include <iostream>
#include <memory>

RCS_INSTALL_ERRORHANDLERS

bool runLoop = true;


typedef Rcs::StackVec<double, 8> VecNd;



/*******************************************************************************
 *
 ******************************************************************************/
static void testIK2()
{
  RLOG(0, "This example requires some improvements when changing activations");

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
  Rcs::KeyCatcherBase::registerKey("t", "Test ConnectBodyConstraint");

  int algo = 1;
  double alpha = 0.05, lambda = 0.0, dt = 0.01, dt_calc = 0.0;
  double jlCost = 0.0, horizon = 2.0;
  char xmlFileName[128] = "cDualArmScitos7.xml";
  char directory[128] = "config/xml/Kinova";

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  Rcs::CmdLineParser argP;
  argP.getArgument("-horizon", &horizon, "Trajectory horizon (default is %f)",
                   horizon);
  argP.getArgument("-algo", &algo, "IK algorithm: 0: left inverse, 1: "
                   "right inverse (default is %d)", algo);
  argP.getArgument("-alpha", &alpha,
                   "Null space scaling factor (default is %f)", alpha);
  argP.getArgument("-lambda", &lambda, "Regularization (default is %f)",
                   lambda);
  argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                   "is \"%s\")", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is \"%s\")", directory);
  argP.getArgument("-dt", &dt, "Sampling time interval");
  bool zigzag = argP.hasArgument("-zigzag", "ZigZag trajectory");

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  if (argP.hasArgument("-h"))
  {
    printf("Resolved motion rate control test\n\n");
    pthread_mutex_destroy(&graphLock);
    return;
  }

  Rcs_addResourcePath(directory);

  // Create controller
  Rcs::ControllerBase controller(xmlFileName);
  Rcs::IkSolverRMR* ikSolver = new Rcs::IkSolverRMR(&controller);

  MatNd* dq_des     = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* q_dot_des  = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* a_des      = MatNd_create(controller.getNumberOfTasks(), 1);
  MatNd* x_curr     = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des      = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des_prev = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des_f    = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dx_des     = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dH         = MatNd_create(1, controller.getGraph()->nJ);

  controller.readActivationsFromXML(a_des);
  controller.computeX(x_curr);
  MatNd_copy(x_des, x_curr);
  MatNd_copy(x_des_f, x_curr);
  MatNd_copy(x_des_prev, x_curr);

  std::shared_ptr<tropic::TrajectoryControllerBase> tc;

  if (zigzag)
  {
    tc = std::make_shared<tropic::TrajectoryController<tropic::ZigZagTrajectory1D>>(&controller, horizon);
  }
  else
  {
    tc = std::make_shared<tropic::TrajectoryController<tropic::ViaPointTrajectory1D>>(&controller, horizon);
  }

  // tc->setActivation(true);
  tc->setTurboMode(false);
  tc->takeControllerOwnership(true);


  // Create visualization
  char hudText[2056] = "";
  Rcs::Viewer* v           = new Rcs::Viewer();
  Rcs::KeyCatcher* kc      = new Rcs::KeyCatcher();
  Rcs::GraphNode* gn       = new Rcs::GraphNode(controller.getGraph());
  Rcs::HUD* hud            = new Rcs::HUD();
  v->add(gn);
  v->add(hud);
  v->add(kc);
  v->runInThread(mtx);




  // Endless loop
  while (runLoop == true)
  {
    pthread_mutex_lock(&graphLock);

    dt_calc = Timer_getTime();

    //////////////////////////////////////////////////////////////////
    // Step trajectories
    //////////////////////////////////////////////////////////////////
    tc->step(dt);
    tc->getPosition(0.0, x_des_f);
    tc->getActivation(a_des);

    controller.computeDX(dx_des, x_des_f);
    double clipLimit = 0.1;
    MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
    MatNd_saturateSelf(dx_des, &clipArr);
    controller.computeJointlimitGradient(dH);
    MatNd_constMulSelf(dH, alpha);
    ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
    MatNd_constMul(q_dot_des, dq_des, 1.0/dt);

    MatNd_addSelf(controller.getGraph()->q, dq_des);
    RcsGraph_setState(controller.getGraph(), NULL, q_dot_des);
    //bool poseOK = controller.checkLimits();
    controller.computeX(x_curr);

    dt_calc = Timer_getTime() - dt_calc;

    pthread_mutex_unlock(&graphLock);

    if (kc && kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc && kc->getAndResetKey('t'))
    {
      RMSG("Loading Johannes's class");
      auto ts = std::make_shared<tropic::PouringConstraint>();
      tc->addAndApply(ts, true);
      ts->print();
      RMSG("Done loading Johannes's class");
    }


    sprintf(hudText, "IK calculation: %.1f us\ndof: %d nJ: %d "
            "nqr: %d nx: %d\nJL-cost: %.6f"
            "\nlambda:%g alpha: %g constraints: %d",
            1.0e6*dt_calc, controller.getGraph()->dof,
            controller.getGraph()->nJ, ikSolver->getInternalDof(),
            (int) controller.getActiveTaskDim(a_des),
            jlCost, lambda, alpha,
            (int) tc->getNumberOfSetConstraints());

    if (hud != NULL)
    {
      hud->setText(hudText);
    }
    else
    {
      std::cout << hudText;
    }

    Timer_waitDT(dt);
  }



  // Clean up
  delete v;

  tc->takeControllerOwnership(false);

  MatNd_destroy(dq_des);
  MatNd_destroy(q_dot_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x_curr);
  MatNd_destroy(x_des);
  MatNd_destroy(x_des_f);
  MatNd_destroy(x_des_prev);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);

  pthread_mutex_destroy(&graphLock);

  delete ikSolver;
}

static void testLinearAccelerationTrajectory(int argc, char** argv)
{
  double t1 = 1.0;
  double dt = t1 / 3.0;
  double t1_2 = t1 * t1;
  double t1_3 = t1_2 * t1;

  MatNd* B = MatNd_create(6, 6);
  MatNd* x = MatNd_create(6, 1);
  MatNd* p = MatNd_create(6, 1);

  // Create B-matrix
  MatNd_setIdentity(B);

  // Acceleration
  double* row = MatNd_getRowPtr(B, 3);
  VecNd_set6(row, 1.0, 0.0, 0.0, t1, t1-dt, t1-2.0*dt);

  // Velocity
  row = MatNd_getRowPtr(B, 4);
  VecNd_set6(row, t1, 1.0, 0.0, 0.5*t1_2, 0.5*(t1-dt)*(t1-dt), 0.5*(t1 - 2.0*dt)*(t1 - 2.0*dt));

  // Position
  row = MatNd_getRowPtr(B, 5);
  VecNd_set6(row, 0.5*t1_2, t1, 1.0, t1_3 / 6.0, pow(t1-dt,3)/6.0, pow(t1 - 2.0*dt, 3) / 6.0);

  MatNd_printCommentDigits("B", B, 4);

  // Create constraint vector
  VecNd_set6(x->ele, -10.0, 0.0, -1.0, 0.0, 0.0, 1.0);

  // Solve for parameter vector
  MatNd* invB = MatNd_createLike(B);
  double det = MatNd_gaussInverse(invB, B);
  MatNd_mul(p, invB, x);

  REXEC(1)
  {
    RLOG(0, "Determinant is %f", det);
    MatNd_printCommentDigits("inv(B)", invB, 4);
    MatNd_printCommentDigits("p", p, 4);

    // Test
    MatNd* x_test = MatNd_createLike(x);
    MatNd_mul(x_test, B, p);
    MatNd_printCommentDigits("x_test", x_test, 4);
    MatNd_destroy(x_test);

    // Another test
    MatNd* B_test = MatNd_createLike(B);
    MatNd_mul(B_test, B, invB);
    MatNd_printCommentDigits("B_test", B_test, 4);
    MatNd_destroy(B_test);
  }

  const int nSteps = 1000;
  MatNd* plot = MatNd_create(nSteps, 3);
  for (int i = 0; i < nSteps; ++i)
  {
    double t = (double)i / (double)nSteps;

    double a1 = p->ele[0];
    double a2 = p->ele[3];
    double a3 = p->ele[4];
    double a4 = p->ele[5];
    double b = p->ele[1];
    double c = p->ele[2];

    double xpp = a1 + a2 * t;
    double xp = b + a1 * t + 0.5*a2*t*t;
    double x = c + b*t + 0.5*a1*t*t + a2*t*t*t/6.0;

    if (t >= dt)
    {
      xpp += a3 * (t - dt);
      xp += 0.5*a3*(t - dt)*(t - dt);
      x += a3 * pow(t - dt, 3) / 6.0;
    }
    if (t>=2.0*dt)
    {
      xpp += a4 * (t - 2.0*dt);
      xp += 0.5*a4*(t - 2.0*dt)*(t - 2.0*dt);
      x += a4 * pow(t - 2.0*dt, 3) / 6.0;
    }

    MatNd_set(plot, i, 0, xpp);
    MatNd_set(plot, i, 1, xp);
    MatNd_set(plot, i, 2, x);
  }

  MatNd_gnuplot("xpp", plot);





  MatNd_destroy(B);
  MatNd_destroy(invB);
  MatNd_destroy(x);
  MatNd_destroy(p);
  MatNd_destroy(plot);
}
/*******************************************************************************
 * Contact points for box:
 * (view from the robot side)
 *
 *           z
 *           ^
 *           |
 *      y <---
 *
 *
 *          6    5    4    3    2
 *       ____________________________
 *    7 |            e1              | 1
 *      |                            |
 *    8 |e2                        e0| 0
 *      |            e3              |
 *    9 |____________________________| 15
 *          10   11   12   13   14
 *
 ******************************************************************************/
std::vector<HTr> getContacts()
{
  std::vector<HTr> contact(16);

  HTr handTransform;
  double x[6];

  VecNd_set6(x, 0.0, -0.3, 0.0, -M_PI_2, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[0] = handTransform;

  VecNd_set6(x, 0.0, -0.3, 0.1, -M_PI_2, 0.0, 0.1);
  HTr_from6DVector(&handTransform, x);
  contact[1] = handTransform;

  VecNd_set6(x, 0.0, -0.3, -0.1, -M_PI_2, 0.0, -0.1);
  HTr_from6DVector(&handTransform, x);
  contact[15] = handTransform;



  VecNd_set6(x, 0.0, 0.3, 0.1, M_PI_2, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[8] = handTransform;

  VecNd_set6(x, 0.0, 0.3, 0.0, M_PI_2, 0.0, 0.1);
  HTr_from6DVector(&handTransform, x);
  contact[7] = handTransform;

  VecNd_set6(x, 0.0, 0.3, -0.1, M_PI_2, 0.0, -0.1);
  HTr_from6DVector(&handTransform, x);
  contact[9] = handTransform;



  VecNd_set6(x, 0.0, 0.2, -0.15, 0.0, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[10] = handTransform;

  VecNd_set6(x, 0.0, 0.1, -0.15, 0.0, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[11] = handTransform;

  VecNd_set6(x, 0.0, 0.0, -0.15, 0.0, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[12] = handTransform;

  VecNd_set6(x, 0.0, -0.1, -0.15, 0.0, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[13] = handTransform;

  VecNd_set6(x, 0.0, -0.2, -0.15, 0.0, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[14] = handTransform;



  VecNd_set6(x, 0.0, 0.2, 0.15, M_PI, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[6] = handTransform;

  VecNd_set6(x, 0.0, 0.1, 0.15, M_PI, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[5] = handTransform;

  VecNd_set6(x, 0.0, 0.0, 0.15, M_PI, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[4] = handTransform;

  VecNd_set6(x, 0.0, -0.1, 0.15, M_PI, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[3] = handTransform;

  VecNd_set6(x, 0.0, -0.2, 0.15, M_PI, 0.0, 0.0);
  HTr_from6DVector(&handTransform, x);
  contact[2] = handTransform;


  return contact;
}

class RelGrip : public tropic::ActivationSet
{
public:
  using ConstraintSet::apply;

  enum TransitionType
  {
    RotateObject,
    RegraspRight,
    RegraspLeft,
    MoveAll,
    Undefined
  };

  RelGrip(tropic::TrajectoryControllerBase* tc_, double deltaPhi_) :
    tc(tc_), deltaPhi(deltaPhi_)
  {
    this->trajPhi  = tc->getTrajectory("Phi_Box");
    this->trajPosR = tc->getTrajectory("XYZ_R");
    this->trajPosL = tc->getTrajectory("XYZ_L");
    this->trajOriR = tc->getTrajectory("ABC_R");
    this->trajOriL = tc->getTrajectory("ABC_L");

    this->taskPhi  = tc->getTask("Phi_Box");
    this->taskPosR = tc->getTask("XYZ_R");
    this->taskPosL = tc->getTask("XYZ_L");
    this->taskOriR = tc->getTask("ABC_R");
    this->taskOriL = tc->getTask("ABC_L");
  }

  bool moveTo(int phi, int rh, int lh, double t_goal)
  {
    std::vector<HTr> contacts = getContacts();

    if ((rh<0) || ((size_t)rh>=contacts.size()))
    {
      RLOG(1, "Invalid state for next right hand: %d (should be 0...%d)",
           rh, (int) contacts.size());
      return false;
    }

    if ((lh<0) || ((size_t)lh>=contacts.size()))
    {
      RLOG(1, "Invalid state for next left hand: %d (should be 0...%d)",
           lh, (int) contacts.size());
      return false;
    }

    set.clear();
    double ea[3];
    Vec3d_set(ea, deltaPhi*phi, 0.0, 0.0);
    add(std::make_shared<tropic::EulerConstraint>(t_goal, ea, trajPhi->getName()));
    add(std::make_shared<tropic::PoseConstraint>(t_goal, &contacts[rh],
                                                 trajPosR->getName(), trajOriR->getName()));
    add(std::make_shared<tropic::PoseConstraint>(t_goal, &contacts[lh],
                                                 trajPosL->getName(), trajOriL->getName()));
    ConstraintSet::apply(tc->getTrajectoriesRef());

    return true;
  }

  std::vector<int> getState()
  {
    std::vector<HTr> contacts = getContacts();
    std::vector<int> state(3);
    double x[3];

    // Box rotation
    taskPhi->computeX(x);
    state[0] = round(x[0]/deltaPhi);

    // Right hand
    state[1] = 0;
    taskPosR->computeX(x);
    double minDist = Vec3d_distance(x, contacts[0].org);
    for (size_t i=1; i<contacts.size(); ++i)
    {
      double dist_i = Vec3d_distance(x, contacts[i].org);
      if (dist_i < minDist)
      {
        minDist = dist_i;
        state[1] = i;
      }
    }

    // Left hand
    state[2] = 0;
    taskPosL->computeX(x);
    minDist = Vec3d_distance(x, contacts[0].org);
    for (size_t i=1; i<contacts.size(); ++i)
    {
      double dist_i = Vec3d_distance(x, contacts[i].org);
      if (dist_i < minDist)
      {
        minDist = dist_i;
        state[2] = i;
      }
    }

    return state;
  }

  int getState(int idx)
  {
    std::vector<int> state = getState();
    return state[idx];
  }

  TransitionType getTransitionType(int phi0, int rh0, int lh0,
                                   int phi1, int rh1, int lh1)
  {
    TransitionType tt;

    if (phi0!=phi1)
    {
      tt = RotateObject;

      if ((rh0!=rh1) || (lh0!=lh1))
      {
        tt = Undefined;
      }
    }
    else if (rh0 != rh1)
    {
      tt = RegraspRight;

      if ((phi0!=phi1) || (lh0!=lh1))
      {
        tt = Undefined;
      }
    }
    else if (lh0 != lh1)
    {
      tt = RegraspLeft;

      if ((phi0!=phi1) || (rh0!=rh1))
      {
        tt = Undefined;
      }
    }
    else
    {
      tt = Undefined;
    }

    return tt;
  }

  bool addTransition(int phi1, int rh1, int lh1,
                     double t0, double duration)
  {
    std::vector<int> currentState = getState();

    int phi0 = currentState[0];
    int rh0 = currentState[1];
    int lh0 = currentState[2];

    return addTransition(phi0, rh0, lh0, phi1, rh1, lh1, t0, duration);
  }

  bool addTransition(int phi0, int rh0, int lh0,
                     int phi1, int rh1, int lh1,
                     double t0, double duration)
  {
    std::vector<HTr> contacts = getContacts();

    if ((rh0<0) || ((size_t)rh0>=contacts.size()))
    {
      RLOG(1, "Invalid state for previous right hand: %d (should be 0...%d)",
           rh0, (int) contacts.size());
      return false;
    }

    if ((lh0<0) || ((size_t)lh0>=contacts.size()))
    {
      RLOG(1, "Invalid state for previous left hand: %d (should be 0...%d)",
           lh0, (int) contacts.size());
      return false;
    }

    if ((rh1<0) || ((size_t)rh1>=contacts.size()))
    {
      RLOG(1, "Invalid state for next right hand: %d (should be 0...%d)",
           rh1, (int) contacts.size());
      return false;
    }

    if ((lh1<0) || ((size_t)lh1>=contacts.size()))
    {
      RLOG(1, "Invalid state for next left hand: %d (should be 0...%d)",
           lh1, (int) contacts.size());
      return false;
    }

    TransitionType transition = getTransitionType(phi0, rh0, lh0,
                                                  phi1, rh1, lh1);

    bool success = true;
    const double t1 = t0 + duration;
    const double phiBox = deltaPhi*phi1;
    const double backDist = 0.2;
    const double swayDist = 0.2;

    double boxPos[3];
    Vec3d_set(boxPos, phiBox, 0.0, 0.0);

    add(std::make_shared<tropic::PositionConstraint>(t1, boxPos, trajPhi->getName()));
    add(std::make_shared<tropic::PoseConstraint>(t1, &contacts[rh1], trajPosR->getName(), trajOriR->getName()));
    add(std::make_shared<tropic::PoseConstraint>(t1, &contacts[lh1], trajPosL->getName(), trajOriL->getName()));

    switch (transition)
    {
      case RotateObject:
      {
        break;
      }

      case RegraspRight:
      {
        // Right hand backwards retract: to be applied to position index 0
        // (x points into box forward direction)
        add(t0+0.5*duration, contacts[rh0].org[0]-backDist,
            0.0, 0.0, 1, trajPosR->getTrajectory1D(0)->getName());

        // Right hand sideways retract
        double invNormal[3];
        Vec3d_copy(invNormal, contacts[rh0].rot[2]);
        Vec3d_constMulSelf(invNormal, -swayDist);

        add(t0+0.3*duration, contacts[rh0].org[2]+invNormal[2],
            0.0, 0.0, 1, trajPosR->getTrajectory1D(2)->getName());

        add(t0+0.3*duration, contacts[rh0].org[1]+invNormal[1],
            0.0, 0.0, 1, trajPosR->getTrajectory1D(1)->getName());

        addActivation(t0+0.3*duration, false, 0.1*duration, trajOriR->getName());

        // Right hand sideways approach
        Vec3d_copy(invNormal, contacts[rh1].rot[2]);
        Vec3d_constMulSelf(invNormal, -swayDist);
        add(t0+0.7*duration, contacts[rh1].org[2]+invNormal[2],
            0.0, 0.0, 1, trajPosR->getTrajectory1D(2)->getName());

        add(t0+0.7*duration, contacts[rh1].org[1]+invNormal[1],
            0.0, 0.0, 1, trajPosR->getTrajectory1D(1)->getName());

        addActivation(t0+0.5*duration, true, 0.1*duration, trajOriR->getName());
        break;
      }

      case RegraspLeft:
      {
        // Left hand backwards retract: to be applied to position index 0
        // (x points into box forward direction)
        add(t0+0.5*duration, contacts[lh0].org[0]-backDist, 0.0, 0.0, 1,
            trajPosL->getTrajectory1D(0)->getName());

        // Left hand sideways retract
        double invNormal[3];
        Vec3d_copy(invNormal, contacts[lh0].rot[2]);
        Vec3d_constMulSelf(invNormal, -swayDist);
        add(t0+0.3*duration, contacts[lh0].org[2]+invNormal[2], 0.0, 0.0, 1,
            trajPosL->getTrajectory1D(2)->getName());

        add(t0+0.3*duration, contacts[lh0].org[1]+invNormal[1], 0.0, 0.0, 1,
            trajPosL->getTrajectory1D(1)->getName());

        addActivation(t0+0.3*duration, false, 0.1*duration, trajOriL->getName());

        // Left hand sideways approach
        Vec3d_copy(invNormal, contacts[lh1].rot[2]);
        Vec3d_constMulSelf(invNormal, -swayDist);
        add(t0+0.7*duration, contacts[lh1].org[2]+invNormal[2], 0.0, 0.0, 1,
            trajPosL->getTrajectory1D(2)->getName());

        add(t0+0.7*duration, contacts[lh1].org[1]+invNormal[1], 0.0, 0.0, 1,
            trajPosL->getTrajectory1D(1)->getName());

        addActivation(t0+0.5*duration, true, 0.1*duration, trajOriL->getName());
        break;
      }

      default:
      {
        RLOG(1, "Undefined transition: %d %d %d - %d %d %d",
             phi0, rh0, lh0, phi1, rh1, lh1);
        break;
      }

    }   // switch (transition)

    return success;
  }

  tropic::TrajectoryControllerBase* tc;
  double deltaPhi;

  tropic::TrajectoryND* trajPhi;
  tropic::TrajectoryND* trajPosR;
  tropic::TrajectoryND* trajPosL;
  tropic::TrajectoryND* trajOriR;
  tropic::TrajectoryND* trajOriL;

  const Rcs::Task* taskPhi;
  const Rcs::Task* taskPosR;
  const Rcs::Task* taskPosL;
  const Rcs::Task* taskOriR;
  const Rcs::Task* taskOriL;
};



/*******************************************************************************
 * Ctrl-C destructor. Tries to quit gracefully with the first Ctrl-C
 * press, then just exits.
 ******************************************************************************/
void quit(int /*sig*/)
{
  static int kHit = 0;
  runLoop = false;
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}


/*******************************************************************************
 *
 ******************************************************************************/
// The below comment is for astyle, since otherwise it screws up the formatting
// *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
const char* xmlFileExplore =
  MULTI_LINE_STRING(
<Controller >

<Graph >

  <Body name="Ground Plane" physics="kinematic" >
    <Shape type="BOX" extents="6.0 6.0 0.04" transform="0 0 -0.02 0 0 0"
     color="PEWTER" graphics="true" physics="true" />
    <Shape type="FRAME" scale="0.5" />
  </Body>

  <Body name="Box_v" prev="Ground Plane" >
    <Joint name="DofX_v"   type="TransX" range="-2 0 2" constraint="true" transform="0.5 0 1 0 0 0" />
    <Joint name="DofY_v"   type="TransY" range="-2 0 2"  constraint="true" />
    <Joint name="DofZ_v"   type="TransZ" range="-2 0 2" constraint="true" />
    <Joint name="DofThX_v" type="RotX" range="-360 0 360" weightJL="0" />
    <Joint name="DofThY_v" type="RotY" range="-360 0 360" weightJL="0" />
    <Joint name="DofThZ_v" type="RotZ" range="-360 0 360" weightJL="0" transform="-0.5 0 0 0 0 0" />
    <Shape type="BOX" color="DARKRED_TRANS" extents="1 0.6 0.3" graphics="true" transform="0.51 0 0 0 0 0"/>
    <Shape type="FRAME" scale="0.25" />
  </Body>

  <Body name="PowerGrasp_L" prev="Box_v" color="GREEN" physics="kinematic" >
    <Joint name="DofX_L"   type="TransX" range="-4 0 4" />
    <Joint name="DofY_L"   type="TransY" range="-4 0.25 4" />
    <Joint name="DofZ_L"   type="TransZ" range="-1 -0.15 1"  />
    <Joint name="DofThX_L" type="RotX" range="-360 0 360" />
    <Joint name="DofThY_L" type="RotY" range="-360 0 360" />
    <Joint name="DofThZ_L" type="RotZ" range="-360 0 360" />
    <Shape type="CYLINDER" radius="0.01" length="0.1" graphics="true"
           transform="-0.01 0 -0.01  90 0 0" physics="true" />
    <Shape type="BOX" extents="0.2 0.1 0.02" graphics="true"
           transform="0.1 0 -0.01 0 0 0" physics="true" />
    <Shape type="BOX" extents="0.02 0.05 0.2" graphics="true"
           transform="-0.01 0 0.1 0 0 0" physics="true" />
    <Shape type="FRAME" scale="0.1" />
  </Body>

  <Body name="PowerGrasp_R" prev="Box_v" color="RED" physics="kinematic" >
    <Joint name="DofX_R"   type="TransX" range="-4 0 4" />
    <Joint name="DofY_R"   type="TransY" range="-4 -0.25 4" />
    <Joint name="DofZ_R"   type="TransZ" range="-1 -0.15 1"  />
    <Joint name="DofThX_R" type="RotX" range="-360 0 360" />
    <Joint name="DofThY_R" type="RotY" range="-360 0 360" />
    <Joint name="DofThZ_R" type="RotZ" range="-360 0 360" />
    <Shape type="SPHERE" radius="0.005" graphics="true" />
    <Shape type="CYLINDER" radius="0.01" length="0.1" graphics="true"
           transform="-0.01 0 -0.01  90 0 0" physics="true" />
    <Shape type="BOX"    extents="0.2 0.1 0.02" graphics="true"
           transform="0.1 0 -0.01 0 0 0" physics="true" />
    <Shape type="BOX"    extents="0.02 0.05 0.2" graphics="true"
           transform="-0.01 0 0.1 0 0 0" physics="true" />
    <Shape type="FRAME" scale="0.1" />
  </Body>

</Graph>

<Task name="Phi_Box" controlVariable="Joints" jnts="DofThX_v DofThY_v DofThZ_v" active="xtrue" />
<Task name="XYZ_L" controlVariable="XYZ" effector="PowerGrasp_L" refBdy="Box_v" active="xtrue" />
<Task name="XYZ_R" controlVariable="XYZ" effector="PowerGrasp_R" refBdy="Box_v" active="xtrue" />
<Task name="ABC_L" controlVariable="ABC" effector="PowerGrasp_L" refBdy="Box_v" active="xtrue" />
<Task name="ABC_R" controlVariable="ABC" effector="PowerGrasp_R" refBdy="Box_v" active="xtrue" />

</Controller>
);
// *INDENT-ON*

static void testExplore()
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("a", "Move to state 0 14 8");
  Rcs::KeyCatcherBase::registerKey("A", "Move to state 0 14 10");
  Rcs::KeyCatcherBase::registerKey("b", "Toggle pause");
  Rcs::KeyCatcherBase::registerKey("B", "Move to state 0 1 10");
  Rcs::KeyCatcherBase::registerKey("t", "Move to state entered on console");
  Rcs::KeyCatcherBase::registerKey("T", "Transition to state from console");
  Rcs::KeyCatcherBase::registerKey("f", "Freeze trajectory");



  char hudText[4096] = "";
  Rcs::CmdLineParser argP;
  double alpha = 0.05, horizon = 1.0, dt = 0.005, t = 0.0, deltaPhi = 5.0;
  argP.getArgument("-horizon", &horizon, "Trajectory horizon (default is %f)",
                   horizon);
  argP.getArgument("-deltaPhi", &deltaPhi, "Angular discretization in degrees "
                   "(default is %g)", deltaPhi);
  argP.getArgument("-alpha", &alpha,
                   "Null space scaling factor (default is %g)", alpha);
  bool pause = argP.hasArgument("-pause", "Hit key for each iteration");
  bool withGui = argP.hasArgument("-gui", "Launch Task Gui");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without fan"
                                         "cy stuff (shadows, anti-aliasing)");
  bool valgrind = argP.hasArgument("-valgrind", "Start without Guis and "
                                   "graphics");
  bool zigzag = argP.hasArgument("-zigzag", "ZigZag trajectory");
  double dt_step = dt;
  deltaPhi *= M_PI/180.0;   // Convert to radians

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  Rcs_addResourcePath(RCS_CONFIG_DIR);

  if (argP.hasArgument("-h"))
  {
    pthread_mutex_destroy(&graphLock);
    return;
  }

  Rcs::ControllerBase controller(xmlFileExplore);
  tropic::TrajectoryControllerBase* traj = NULL;

  if (zigzag)
  {
    traj = new tropic::TrajectoryController<tropic::ZigZagTrajectory1D>(&controller, horizon);
  }
  else
  {
    traj = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(&controller, horizon);
  }


  traj->getTrajectory("Phi_Box")->addActivationPoint(std::make_shared<tropic::ActivationPoint>(0.0, true, 0.0));
  traj->getTrajectory("XYZ_R")->addActivationPoint(std::make_shared<tropic::ActivationPoint>(0.0, true, 0.0));
  traj->getTrajectory("XYZ_L")->addActivationPoint(std::make_shared<tropic::ActivationPoint>(0.0, true, 0.0));
  traj->getTrajectory("ABC_R")->addActivationPoint(std::make_shared<tropic::ActivationPoint>(0.0, true, 0.0));
  traj->getTrajectory("ABC_L")->addActivationPoint(std::make_shared<tropic::ActivationPoint>(0.0, true, 0.0));



  Rcs::IkSolverRMR ikSolver(&controller);

  MatNd* x_des     = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dx_des    = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dq_des    = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* q_dot_des = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* dH        = MatNd_create(1, controller.getGraph()->nJ);

  Rcs::Viewer* viewer = NULL;
  Rcs::GraphNode* gn  = NULL;
  Rcs::HUD* hud = NULL;
  Rcs::KeyCatcher* kc = NULL;

  if (!valgrind)
  {
    viewer = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
    gn = new Rcs::GraphNode(controller.getGraph());
    gn->toggleReferenceFrames();
    viewer->add(gn);

    Rcs::BodyNode* trajectoryAnchor = gn->getBodyNode("Box_v");
    RCHECK(trajectoryAnchor);

    for (size_t i=0; i<traj->getController()->getNumberOfTasks(); ++i)
    {
      if (STREQ(traj->getTrajectory(i)->getClassName(), "TrajectoryPos3D"))
      {
        trajectoryAnchor->addChild(new tropic::PositionTrajectoryNode(traj->getTrajectory(i)));
      }
    }

    hud = new Rcs::HUD();
    viewer->add(hud);

    kc = new Rcs::KeyCatcher();
    viewer->add(kc);
    HTr A_CI;
    double x[6];
    VecNd_set6(x, -3.4, 0.9, 2.2, 0.12, 0.32, -0.30);
    HTr_from6DVector(&A_CI, x);
    viewer->setCameraTransform(&A_CI);
    viewer->runInThread(mtx);

    if (withGui==true)
    {
      // We launch it in "show only" mode, so that there are no modifications
      // to the activations. Our interface is not very well designed, so that
      // we have to do some casting here.
      Rcs::ControllerWidgetBase::create(&controller,
                                        (MatNd*)traj->getActivationPtr(),
                                        x_des, x_des, mtx, true);
    }
  }

  while (runLoop)
  {
    if (pause==true)
    {
      RPAUSE_MSG("Hit enter to continue");
    }

    /////////////////////////////////////////////////////////////////
    // Keycatcher
    /////////////////////////////////////////////////////////////////
    if (kc && kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc && kc->getAndResetKey('b'))
    {
      pause = !pause;
    }
    else if (kc && kc->getAndResetKey('a'))
    {
      auto tt = std::make_shared<RelGrip>(traj, deltaPhi);
      tt->addTransition(0, 14, 8, 0.0, 2.0);
      traj->add(tt);
      tt->apply(traj->getTrajectoriesRef());
      tt->toXML("RelGrip.xml");
    }
    else if (kc && kc->getAndResetKey('B'))
    {
      auto tt = std::make_shared<RelGrip>(traj, deltaPhi);
      tt->addTransition(0, 1, 10, 0.0, 2.0);
      traj->add(tt);
      tt->apply(traj->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('A'))
    {
      auto tt = std::make_shared<RelGrip>(traj, deltaPhi);
      tt->addTransition(0, 14, 10, 0.0, 2.0);
      traj->add(tt);
      tt->apply(traj->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('t'))
    {
      int phiNew, posLNew, posRNew;
      RMSG("Enter new state: ");
      std::cin >> phiNew;
      std::cin >> posRNew;
      std::cin >> posLNew;

      RelGrip* tt = new RelGrip(traj, deltaPhi);
      tt->moveTo(phiNew, posRNew, posLNew, 2.0);
      delete tt;
    }
    else if (kc && kc->getAndResetKey('T'))
    {
      int phiNew, posLNew, posRNew;
      RMSG("Enter new state: ");
      std::cin >> phiNew;
      std::cin >> posRNew;
      std::cin >> posLNew;

      auto tt = std::make_shared<RelGrip>(traj, deltaPhi);
      tt->addTransition(phiNew, posRNew, posLNew, 0.0, 2.0);
      traj->add(tt);
      tt->apply(traj->getTrajectoriesRef());
    }
    else if (kc && kc->getAndResetKey('f'))
    {
      dt_step = (dt_step==dt) ? 0.0 : dt;
      RMSG("%s", (dt_step==0.0) ? "FREEZE" : "UNFREEZE");
    }

    pthread_mutex_lock(&graphLock);

    /////////////////////////////////////////////////////////////////
    // Trajectory
    /////////////////////////////////////////////////////////////////
    double dt_traj = Timer_getSystemTime();
    traj->step(dt_step);
    dt_traj = Timer_getSystemTime() - dt_traj;
    traj->getPosition(0.0, x_des);

    /////////////////////////////////////////////////////////////////
    // Inverse kinematics
    /////////////////////////////////////////////////////////////////
    controller.computeDX(dx_des, x_des);
    controller.computeJointlimitGradient(dH);
    MatNd_constMulSelf(dH, alpha*traj->computeBlending());
    ikSolver.solveRightInverse(dq_des, dx_des, dH, traj->getActivationPtr(), 0.0);
    MatNd_addSelf(controller.getGraph()->q, dq_des);
    MatNd_constMul(q_dot_des, dq_des, 1.0/dt);
    RcsGraph_setState(controller.getGraph(), NULL, q_dot_des);

    pthread_mutex_unlock(&graphLock);

    //////////////////////////////////////////////////////////////
    // HUD
    /////////////////////////////////////////////////////////////////
    RelGrip tt(traj, deltaPhi);
    sprintf(hudText, "Time: %.3f\ndt_traj: %.1f msec\nConstraints: %d   "
            "Blending: %.3f\nState: %d %d %d", t, 1000.0*dt_traj,
            (int) traj->getNumberOfConstraints(), traj->computeBlending(),
            tt.getState(0), tt.getState(1), tt.getState(2));

    if (hud != NULL)
    {
      hud->setText(hudText);
    }
    else
    {
      std::cout << hudText;
    }

    Timer_waitDT(dt);
    t += dt;

    if ((valgrind==true) && (t>1.0))
    {
      runLoop = false;
    }
  }

  if (viewer)
  {
    delete viewer;
  }

  delete traj;

  MatNd_destroy(x_des);
  MatNd_destroy(dx_des);
  MatNd_destroy(dq_des);
  MatNd_destroy(q_dot_des);
  MatNd_destroy(dH);

  pthread_mutex_destroy(&graphLock);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool testViaPointTrajectory1D()
{
  bool success = true;
  double horizon = 1.0;
  int flag = 1, loopCount = 0;
  double dt = 0.01;
  Rcs::CmdLineParser argP;
  argP.getArgument("-horizon", &horizon, "Receeding horizon length [sec]");
  argP.getArgument("-flag", &flag, "Flag for via trajectory selection");
  argP.getArgument("-dt", &dt, "Sampling time for plotting");
  bool valgrind = argP.hasArgument("-valgrind", "Valgrind checking mode");
  bool turbo = argP.hasArgument("-turbo", "Set turbo mode");

  tropic::ViaPointTrajectory1D via(1.0, horizon);
  RCHECK(via.check());
  via.getViaSequence()->setTurboMode(turbo);


#if 0
  via.addConstraint(0.5, 2.0, 0.0, 0.0, 1);
  via.addConstraint(0.25, -2.0, 0.0, 0.0, 7);
  via.addConstraint(0.75, -1.0, 0.0, 0.0, 1);
  via.addConstraint(1.0, 5.0, 0.0, 0.0, 7);
  via.addConstraint(1.95, 5.0, 0.0, 0.0, 7);
#else
  via.addConstraint(0.95, 2.0, 0.0, 0.0, 0);
  via.addConstraint(1.0, 1.1, 0.0, 0.0, 7);
  via.addConstraint(1.5, 2.0, 0.0, 0.0, 1);
  via.addConstraint(1.75, 2.0, 0.0, 0.0, 0);
  via.addConstraint(1.85, 2.0, 0.0, 0.0, 0);
  via.addConstraint(1.95, 2.0, 0.0, 0.0, 0);
  via.addConstraint(2.0, 1.1, 0.0, 0.0, 7);
  via.addConstraint(10.95, 2.0, 0.0, 0.0, 0);
#endif
  via.initFromConstraints();
  std::cout << via << std::endl;

  const Rcs::ViaPointSequence* viaSeq = via.getViaSequence();
  Rcs::ViaPointSequencePlotter* plotter = NULL;

  if (valgrind==false)
  {
    plotter = new Rcs::ViaPointSequencePlotter();
    plotter->enableFixedAxes(*viaSeq, true);
  }


  while (runLoop)
  {
    if (plotter)
    {
      plotter->plot2(*viaSeq, -0.2, horizon+0.2, dt==0.0?0.01:dt, flag);
    }
    via.step(dt);
    std::cout << via << std::endl;
    if (!valgrind)
    {
      RPAUSE();
    }
    else
    {
      if (loopCount>1000)
      {
        runLoop = false;
      }
    }

    loopCount++;
  }

  delete plotter;

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
static bool testTrajectory1D()
{
  bool success = true;
  double horizon = 1.0;
  int flag = 1;
  double dt = 0.01;
  Rcs::CmdLineParser argP;
  argP.getArgument("-horizon", &horizon, "Receeding horizon length [sec]");
  argP.getArgument("-flag", &flag, "Flag for via trajectory selection");
  argP.getArgument("-dt", &dt, "Sampling time for plotting");
  bool zigzag = argP.hasArgument("-zigzag", "ZigZag trajectory");

  if (argP.hasArgument("-h"))
  {
    return success;
  }

  tropic::Trajectory1D* traj = NULL;

  if (zigzag)
  {
    traj = new tropic::ZigZagTrajectory1D(1.0, horizon);
  }
  else
  {
    traj = new tropic::ViaPointTrajectory1D(1.0, horizon);
  }

#if 0
  traj->addConstraint(0.5, 2.0, 0.0, 0.0, 1);
  traj->addConstraint(0.25, -2.0, 0.0, 0.0, 7);
  traj->addConstraint(0.75, -1.0, 0.0, 0.0, 1);
  traj->addConstraint(1.0, 5.0, 0.0, 0.0, 7);
  traj->addConstraint(1.95, 5.0, 0.0, 0.0, 7);
#else

  traj->addConstraint(1.0, .0, 0.0, 0.0, 7);
  traj->addConstraint(1.5, .0, 0.0, 0.0, 1);
  traj->addConstraint(2.0, .0, 0.0, 0.0, 7);


#endif
  traj->initFromConstraints();
  std::cout << *traj << std::endl;

  tropic::TrajectoryPlotter1D plotter;

  while (runLoop)
  {
    plotter.plot(*traj, 0.0, horizon, dt);
    traj->step(dt);
    std::cout << *traj << std::endl;
    RPAUSE();
  }

  delete traj;

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
// The below comment is for astyle, since otherwise it screws up the formatting
// *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
const char* interactiveGraph =
MULTI_LINE_STRING(
<Graph >

<Body name="Lab frame" physics="kinematic" >
  <Shape type="BOX" extents="6.0 6.0 0.04" transform="0 0 -0.02 0 0 0"
         color="PEWTER" graphics="true" physics="true" />
  <Shape type="BOX" extents="0.1 6 0.2" transform="2.95 0 0.1 0 0 0"
         color="PEWTER" graphics="true" physics="true" />
  <Shape type="BOX" extents="0.1 6 0.2" transform="-2.95 0 0.1 0 0 0"
         color="PEWTER" graphics="true" physics="true" />
  <Shape type="BOX" extents="6 0.1 0.2" transform="0 2.95 0.1 0 0 0"
         color="PEWTER" graphics="true" physics="true" />
  <Shape type="BOX" extents="6 0.1 0.2" transform="0 -2.95 0.1 0 0 0"
         color="PEWTER" graphics="true" physics="true" />
</Body>

<Body name="cursor" physics="kinematic" rigid_body_joints="0 0 1.5 0 0 0" >
  <Shape type="SPHERE" radius="0.025" color="RED" graphics="true" physics="true" />
</Body>

<Body name="box" physics="kinematic" rigid_body_joints="0 0 1 0 0 0" >
  <Shape type="BOX" extents="1.0 0.6 0.3" color="RED" graphics="true" physics="true" />
  <Shape type="FRAME" scale="0.75" />
</Body>

<Body name="sphere" physics="kinematic" rigid_body_joints="1.5 0 1 0 0 0" >
  <Shape type="SSL" radius="0.2" length="0.2" color="GREEN" graphics="true" physics="true" />
  <Shape type="FRAME" scale="0.5" />
</Body>

<Body name="torus" physics="kinematic" rigid_body_joints="-1.5 0 1 0 0 0" >
  <Shape type="TORUS" length="0.2" radius="0.5" color="BLUE" graphics="true" physics="true" />
  <Shape type="FRAME" scale="0.4" />
</Body>

</Graph>);
// *INDENT-ON*

/*******************************************************************************
 *
 ******************************************************************************/
namespace tropic
{

class ContactConstraint : public AnchoredPositionConstraint
{
public:
  ContactConstraint(std::shared_ptr<ContactConstraint> prev,
                    double t, const RcsBody* bdy, const double I_pt[3],
                    TrajectoryND* traj) :
    AnchoredPositionConstraint(t, &bdy->A_BI, I_pt, traj->getName(), 7)
  {
    setClassName("ContactConstraint");
    init(prev, t, bdy, I_pt, traj);
  }

  ContactConstraint(double t, const RcsBody* bdy, const double I_pt[3],
                    TrajectoryND* traj) :
    AnchoredPositionConstraint(t, &bdy->A_BI, I_pt, traj->getName(), 7)
  {
    setClassName("ContactConstraint");
    init(nullptr, t, bdy, I_pt, traj);
  }

  ContactConstraint(xmlNode* node) : AnchoredPositionConstraint(node)
  {
    setClassName("ContactConstraint");
  }

  void init(std::shared_ptr<ContactConstraint> prev,
            double t, const RcsBody* bdy, const double I_pt[3],
            TrajectoryND* traj)
  {
    const double distance = 0.2;
    double I_cpBdy[3], I_nBdyPt[3];
    RcsBody_distanceToPoint(bdy, I_pt, I_cpBdy, I_nBdyPt);

    Vec3d_rotate(this->B_normal, (double(*)[3]) bdy->A_BI.rot, I_nBdyPt);

    if (prev)
    {
      double t0 = prev->getEndTime(), ttc = t - t0, throughPt0[3], throughPt1[3];

      prev->getPreContactPt(throughPt0, distance);
      getPreContactPt(throughPt1, distance);

      double tmp[3];
      Vec3d_sub(tmp, throughPt1, throughPt0);
      Vec3d_constMulAndAddSelf(throughPt0, tmp, 0.1);
      Vec3d_constMulAndAddSelf(throughPt1, tmp, -0.1);

      add(std::make_shared<AnchoredPositionConstraint>(t0+0.3*ttc, prev->A_BI, throughPt0, traj->getName(), 1));
      add(std::make_shared<AnchoredPositionConstraint>(t0+0.7*ttc, A_BI, throughPt1, traj->getName(), 1));

      double midPt[3], midNormal[3], n0[3], n1[3];
      prev->getNormal(n0);
      getNormal(n1);

      for (int i=0; i<3; ++i)
      {
        midPt[i] = throughPt0[i] + 0.5*(throughPt1[i]-throughPt0[i]);
        midNormal[i] = 0.5*(n0[i]+n1[i]);
      }
      Vec3d_normalizeSelf(midNormal);
      Vec3d_constMulAndAddSelf(midPt, midNormal, 2.0*distance);
      add(std::make_shared<AnchoredPositionConstraint>(t0 + 0.5*ttc, A_BI, midPt, traj->getName(), 1));
    }
  }

  void getNormal(double I_n[3])
  {
    Vec3d_transRotate(I_n, (double(*)[3]) A_BI->rot, this->B_normal);
  }

  void getPreContactPt(double I_pt[3], double distance)
  {
    double x[3], n[3];
    getPosition(x);
    getNormal(n);
    Vec3d_constMulAndAdd(I_pt, x, n, distance);
  }

  virtual double compute(double dt)
  {
    for (size_t j=0; j<this->set.size(); ++j)
    {
      for (size_t i = 0; i<set[j]->numConstraints(false); ++i)
      {
        if ((set[j]->getConstraint(i)->getTime()<0.1) &&
            (set[j]->getConstraint(i)->getFlag()==1))
        {
          set[j]->getConstraint(i)->setFlag(0);
        }
      }
    }

    return AnchoredPositionConstraint::compute(dt);
  }

protected:

  double B_normal[3];
};

static ConstraintFactoryRegistrar<ContactConstraint> constraint("ContactConstraint");

}

/*******************************************************************************
 *
 ******************************************************************************/
static void testInteractive()
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("d", "Clear trajectories");
  Rcs::KeyCatcherBase::registerKey("p", "Print Trajectories to stdout");
  Rcs::KeyCatcherBase::registerKey("f", "Freeze time");
  Rcs::KeyCatcherBase::registerKey("x", "Create constraint under mouse");

  Timer_setZero();

  bool freeze = false;
  double time = 0.0;
  double dt = 0.01;
  double motionEndTime = 0.0;
  double horizon = 1.0;
  double ttc = 2.0;
  char hudText[1024] = "";

  Rcs::CmdLineParser argP;
  argP.getArgument("-ttc", &ttc, "Time to contact (default: %f)", ttc);
  argP.getArgument("-horizon", &horizon, "Time horizon (default: %f)", horizon);
  bool zigzag = argP.hasArgument("-zigzag", "ZigZag trajectory");

  if (argP.hasArgument("-h"))
  {
    return;
  }

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);
  RcsGraph* graph = RcsGraph_create(interactiveGraph);
  Rcs::ControllerBase controller(graph);

  RcsBody* cursor = RcsGraph_getBodyByName(graph, "cursor");
  RcsBody* box = RcsGraph_getBodyByName(graph, "box");
  RcsBody* sphere = RcsGraph_getBodyByName(graph, "sphere");
  RcsBody* torus = RcsGraph_getBodyByName(graph, "torus");
  RCHECK(cursor && box && sphere && torus);

  Rcs::TaskPosition3D* posTask = new Rcs::TaskPosition3D(graph, cursor, NULL, NULL);
  posTask->setName("Trajectory position");
  controller.add(posTask);
  MatNd* x = MatNd_create((int) controller.getTaskDim(), 1);

  tropic::TrajectoryControllerBase* tc = NULL;

  if (zigzag)
  {
    tc = new tropic::TrajectoryController<tropic::ZigZagTrajectory1D>(&controller, horizon);
  }
  else
  {
    tc = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(&controller, horizon);
  }

  tc->setActivation(true);
  tc->setActivation(0, true);
  tc->setTurboMode(false);
  tropic::TrajectoryND* posTraj = tc->getTrajectory(0);
  RCHECK(posTraj);
  std::shared_ptr<tropic::ContactConstraint> prevContact, lastContact;

  Rcs::Viewer* viewer = new Rcs::Viewer();
  viewer->add(new Rcs::GraphNode(graph));
  osg::ref_ptr<Rcs::KeyCatcher> kc = new Rcs::KeyCatcher();
  osg::ref_ptr<Rcs::HUD> hud = new Rcs::HUD();
  viewer->add(hud.get());
  viewer->add(kc.get());
  osg::ref_ptr<tropic::PositionTrajectoryNode> pn = new tropic::PositionTrajectoryNode(posTraj);
  //pn->setPointSize(10.0);
  viewer->add(pn.get());
  viewer->runInThread(&mtx);


  const RcsJoint* boxJnt = RCSJOINT_BY_ID(graph, box->jntId);
  const RcsJoint* sphereJnt = RCSJOINT_BY_ID(graph, sphere->jntId);
  const RcsJoint* torusJnt = RCSJOINT_BY_ID(graph, torus->jntId);
  const RcsJoint* cursorJnt = RCSJOINT_BY_ID(graph, cursor->jntId);
  RCHECK(boxJnt && sphereJnt && torusJnt && cursorJnt);

  while (runLoop)
  {
    double t_loop = Timer_getTime();

    //////////////////////////////////////////////////////////////////
    // Step trajectories and update graph
    //////////////////////////////////////////////////////////////////
    pthread_mutex_lock(&mtx);

    double t_calc = Timer_getSystemTime();
    motionEndTime = tc->step(freeze ? 0.0 : dt);
    tc->getPosition(0.0, x);
    t_calc = Timer_getSystemTime() - t_calc;
    graph->q->ele[boxJnt->jointIndex+3] = 0.25*sin(time);
    graph->q->ele[sphereJnt->jointIndex+3] = sin(0.8*time);
    graph->q->ele[torusJnt->jointIndex+3] = sin(1.2*time);
    Vec3d_copy(&graph->q->ele[cursorJnt->jointIndex], x->ele);
    RcsGraph_setState(graph, NULL, NULL);

    if (tc->getNumberOfConstraints()==0)
    {
      lastContact = nullptr;
      prevContact = nullptr;
    }

    pthread_mutex_unlock(&mtx);

    //////////////////////////////////////////////////////////////////
    // Handle viewer keys
    //////////////////////////////////////////////////////////////////
    if (kc->getAndResetKey('q'))
    {
      RMSGS("Quitting run loop");
      runLoop = false;
    }
    else if (kc->getAndResetKey('d'))
    {
      RMSGS("Clearing trajectories");
      tc->clear();
      lastContact = nullptr;
      prevContact = nullptr;
    }
    else if (kc->getAndResetKey('p'))
    {
      RMSGS("Here's the TrajectoryController:");
      tc->print();
    }
    else if (kc->getAndResetKey('f'))
    {
      freeze = ! freeze;
      RMSGS("%s trajectory", freeze ? "Freezing" : "Unfreezing");
    }
    else if (kc->getAndResetKey('x'))
    {
      double pt[3];
      Rcs::BodyNode* bdyNd = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>(pt);
      if (bdyNd != NULL)
      {
        const RcsBody* clicked = bdyNd->body();
        double t_last = posTraj->getTimeOfLastGoal();
        RLOG(1, "Adding constraint to body %s at x=%.3f %.3f %.3f",
             clicked->name, pt[0], pt[1], pt[2]);
        prevContact = lastContact;
        if (prevContact)
        {
          lastContact = std::make_shared<tropic::ContactConstraint>(prevContact, prevContact->getEndTime()+ttc,
                                                                    clicked, pt, posTraj);
        }
        else
        {
          lastContact = std::make_shared<tropic::ContactConstraint>(t_last+ttc, clicked, pt, posTraj);
        }
        //lastContact->apply(tc->getTrajectoriesRef());
        lastContact->apply(tc->getTrajectoriesRef());
        tc->add(lastContact);
      }


      //tc->getRootSet().toXML(std::cout);
      tc->getRootSet().toXML("ConstraintCollection.xml");
    }

    //////////////////////////////////////////////////////////////////
    // HUD
    //////////////////////////////////////////////////////////////////
    sprintf(hudText, "Time: %.3f calculation: %.2f msec\n"
            "dt: %.1f msec %s\nConstraints: %d (%d %d)\n"
            "Sets: %d\n"
            "Motion end: %.3f", time, t_calc*1000.0, freeze ? 0.0 : 1000.0*dt,
            freeze ? "(frozen)" : "",
            (int) tc->getNumberOfSetConstraints(),
            (int) tc->getNumberOfConstraints(),
            (int) tropic::Constraint1D::getNumConstraints(),
            (int) tc->getNumberOfSets(),
            motionEndTime);

    hud->setText(hudText);

    //////////////////////////////////////////////////////////////////
    // Timing
    //////////////////////////////////////////////////////////////////
    time += dt;
    t_loop = Timer_getTime() - t_loop;
    Timer_waitDT(dt-t_loop);
  }

  delete viewer;
  MatNd_destroy(x);
  delete tc;
  pthread_mutex_destroy(&mtx);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void testIK()
{
  RLOG(0, "This example requires some improvements when changing activations");

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
  Rcs::KeyCatcherBase::registerKey("l", "Load trajectory from LiftObject class");

  int algo = 1;
  double alpha = 0.05, lambda = 0.0, dt = 0.01, dt_calc = 0.0;
  double jlCost = 0.0, dJlCost = 0.0, horizon = 2.0;
  bool calcDistance = true;
  char xmlFileName[128] = "cDualArmScitos7.xml";
  char directory[128] = "config/xml/Tropic";
  char effortBdyName[256] = "";

  // Initialize GUI and OSG mutex
  pthread_mutex_t graphLock;
  pthread_mutex_init(&graphLock, NULL);

  Rcs::CmdLineParser argP;
  argP.getArgument("-horizon", &horizon, "Trajectory horizon (default is %f)",
                   horizon);
  argP.getArgument("-algo", &algo, "IK algorithm: 0: left inverse, 1: "
                   "right inverse (default is %d)", algo);
  argP.getArgument("-alpha", &alpha,
                   "Null space scaling factor (default is %f)", alpha);
  argP.getArgument("-lambda", &lambda, "Regularization (default is %f)",
                   lambda);
  argP.getArgument("-f", xmlFileName, "Configuration file name (default "
                   "is \"%s\")", xmlFileName);
  argP.getArgument("-dir", directory, "Configuration file directory "
                   "(default is \"%s\")", directory);
  argP.getArgument("-dt", &dt, "Sampling time interval (default is %f)", dt);
  argP.getArgument("-staticEffort", effortBdyName,
                   "Body to map static effort");
  bool pause = argP.hasArgument("-pause", "Pause after each iteration");
  bool launchJointWidget = argP.hasArgument("-jointWidget",
                                            "Launch JointWidget");
  bool manipulability = argP.hasArgument("-manipulability",
                                         "Manipulability criterion in "
                                         "null space");
  bool cAvoidance = argP.hasArgument("-ca", "Collision avoidance in "
                                     "null space");
  bool constraintIK = argP.hasArgument("-constraintIK", "Use constraint IK"
                                       " solver");
  bool valgrind = argP.hasArgument("-valgrind",
                                   "Start without Guis and graphics");
  bool simpleGraphics = argP.hasArgument("-simpleGraphics", "OpenGL without fan"
                                         "cy stuff (shadows, anti-aliasing)");
  bool zigzag = argP.hasArgument("-zigzag", "ZigZag trajectory");
  bool showOnly = argP.hasArgument("-passiveGui", "Gui shows values but is not active");
  bool noTaskGui = argP.hasArgument("-noGui", "No task Gui");

  // Option without mutex for viewer
  pthread_mutex_t* mtx = &graphLock;
  if (argP.hasArgument("-nomutex", "Graphics without mutex"))
  {
    mtx = NULL;
  }

  if (argP.hasArgument("-h"))
  {
    printf("Resolved motion rate control test\n\n");
    pthread_mutex_destroy(&graphLock);
    return;
  }

  Rcs_addResourcePath(directory);

  // Create controller
  Rcs::ControllerBase controller(xmlFileName);
  Rcs::IkSolverRMR* ikSolver = NULL;

  if (constraintIK==true)
  {
    ikSolver = new Rcs::IkSolverConstraintRMR(&controller);
  }
  else
  {
    ikSolver = new Rcs::IkSolverRMR(&controller);
  }

  MatNd* dq_des     = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* q_dot_des  = MatNd_create(controller.getGraph()->dof, 1);
  MatNd* a_des      = MatNd_create(controller.getNumberOfTasks(), 1);
  MatNd* x_curr     = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des      = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des_prev = MatNd_create(controller.getTaskDim(), 1);
  MatNd* x_des_f    = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dx_des     = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dH         = MatNd_create(1, controller.getGraph()->nJ);
  MatNd* timings    = MatNd_create(5, 1);
  MatNd_set(timings, 0, 0, 1.0);
  MatNd_set(timings, 1, 0, 20.0);

  controller.readActivationsFromXML(a_des);
  controller.computeX(x_curr);
  MatNd_copy(x_des, x_curr);
  MatNd_copy(x_des_f, x_curr);
  MatNd_copy(x_des_prev, x_curr);

  std::shared_ptr<tropic::TrajectoryControllerBase> tc;

  if (zigzag)
  {
    tc = std::make_shared<tropic::TrajectoryController<tropic::ZigZagTrajectory1D>>(&controller, horizon);
  }
  else
  {
    tc = std::make_shared<tropic::TrajectoryController<tropic::ViaPointTrajectory1D>>(&controller, horizon);
  }

  // tc->setActivation(true);
  tc->setTurboMode(false);
  tc->takeControllerOwnership(true);


  // Body for static effort null space gradient
  const RcsBody* effortBdy = RcsGraph_getBodyByName(controller.getGraph(),
                                                    effortBdyName);
  MatNd* F_effort = NULL;
  MatNd_fromStack(F_effort, 4, 1);
  MatNd F_effort3 = MatNd_fromPtr(3, 1, F_effort->ele);


  // Create visualization
  Rcs::Viewer* v           = NULL;
  Rcs::KeyCatcher* kc      = NULL;
  Rcs::GraphNode* gn       = NULL;
  Rcs::HUD* hud            = NULL;
  Rcs::BodyPointDragger* dragger = NULL;
  Rcs::VertexArrayNode* cn = NULL;
  char hudText[2056];

  if (valgrind==false)
  {
    v       = new Rcs::Viewer(!simpleGraphics, !simpleGraphics);
    kc      = new Rcs::KeyCatcher();
    gn      = new Rcs::GraphNode(controller.getGraph());
    hud     = new Rcs::HUD();
    dragger = new Rcs::BodyPointDragger();
    dragger->scaleDragForce(0.01);
    v->add(gn);
    v->add(hud);
    v->add(kc);
    v->add(dragger);

    if (controller.getCollisionMdl() != NULL)
    {
      cn = new Rcs::VertexArrayNode(controller.getCollisionMdl()->cp,
                                    osg::PrimitiveSet::LINES, "RED");
      cn->toggle();
      v->add(cn);
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
        v->add(pn);
      }
    }



    v->runInThread(mtx);

    // Launch the task widget
    if (!noTaskGui)
    {
      if (showOnly)
      {
        Rcs::ControllerWidgetBase::create(&controller, (MatNd*)tc->getActivationPtr(), x_des_f,
                                          x_curr, mtx, true);
      }
      else
      {
        Rcs::ControllerWidgetBase::create(&controller, a_des, x_des,
                                          x_curr, mtx, false);
      }
    }

    if (launchJointWidget==true)
    {
      Rcs::JointWidget::create(controller.getGraph(), mtx);
    }

    if (effortBdy != NULL)
    {
      std::vector<std::string> labels;
      Rcs::MatNdWidget* mw;
      mw = Rcs::MatNdWidget::create(F_effort, F_effort, -1.0, 1.0,
                                    "F_effort", mtx);
      labels.push_back("Fx");
      labels.push_back("Fy");
      labels.push_back("Fz");
      labels.push_back("gain");
      mw->setLabels(labels);
    }

    {
      std::vector<std::string> labels;
      Rcs::MatNdWidget* mw;
      mw = Rcs::MatNdWidget::create(timings, timings, 0.0, 40.0,
                                    "Timings", mtx);
      labels.push_back("t0");
      labels.push_back("t1");
      labels.push_back("t2");
      labels.push_back("t3");
      labels.push_back("t4");
      mw->setLabels(labels);
    }



  }

  unsigned int loopCount = 0;





  // Endless loop
  while (runLoop == true)
  {
    pthread_mutex_lock(&graphLock);

    dt_calc = Timer_getTime();

    //////////////////////////////////////////////////////////////////
    // Step trajectories
    //////////////////////////////////////////////////////////////////
    MatNd* h1 = MatNd_clone(x_des_prev);
    MatNd* h2 = MatNd_clone(x_des);
    controller.compressToActiveSelf(h1, a_des);
    controller.compressToActiveSelf(h2, a_des);
    bool guiChanged = !MatNd_isEqual(h1, h2, 1.0e-8);
    MatNd_destroy(h1);
    MatNd_destroy(h2);

    if (guiChanged && (!showOnly))
    {
      // tc->clear();

      auto tVec = tc->getTrajectories();
      for (size_t i=0; i<tVec.size(); ++i)
      {
        tVec[i]->removeConstraintsAfter(0.5*horizon);
      }

      auto guiConstraint = std::make_shared<tropic::MultiGoalConstraint>(horizon, x_des, tc->getTrajectories());
      tc->addAndApply(guiConstraint);
    }

    MatNd_copy(x_des_prev, x_des);

    double endTime = tc->step(dt);
    tc->getPosition(0.0, x_des_f);

    if (showOnly || noTaskGui)
    {
      tc->getActivation(a_des);
    }

    controller.computeDX(dx_des, x_des_f);
    double clipLimit = 0.1;
    MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
    MatNd_saturateSelf(dx_des, &clipArr);

    controller.computeJointlimitGradient(dH);

    if (calcDistance==true)
    {
      controller.computeCollisionCost();
    }

    if (cAvoidance==true)
    {
      MatNd* dH_ca = MatNd_create(1, controller.getGraph()->dof);
      controller.getCollisionGradient(dH_ca);
      RcsGraph_limitJointSpeeds(controller.getGraph(), dH_ca,
                                1.0, RcsStateIK);
      MatNd_constMulSelf(dH_ca, 0.01);
      MatNd_addSelf(dH, dH_ca);
    }

    if (manipulability)
    {
      MatNd_setZero(dH);
      controller.computeManipulabilityGradient(dH, a_des);
      MatNd_constMulSelf(dH, 100.0);
    }

    if (effortBdy != NULL)
    {
      MatNd* W_ef = MatNd_create(controller.getGraph()->dof, 1);
      RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
      {
        W_ef->ele[JNT->jointIndex] = 1.0/JNT->maxTorque;
      }

      RcsGraph_stateVectorToIKSelf(controller.getGraph(), W_ef);
      MatNd* effortGrad = MatNd_create(1, controller.getGraph()->nJ);
      RcsGraph_staticEffortGradient(controller.getGraph(), effortBdy,
                                    &F_effort3, W_ef, NULL, effortGrad);
      MatNd_destroy(W_ef);
      MatNd_constMulSelf(effortGrad, 1000.0*MatNd_get(F_effort, 3, 0));
      MatNd_addSelf(dH, effortGrad);

      MatNd_destroy(effortGrad);
    }

    MatNd_constMulSelf(dH, alpha*tc->computeBlending());

    if (valgrind==false)
    {
      dragger->addJointTorque(dH, controller.getGraph());
    }

    switch (algo)
    {
      case 0:
        ikSolver->solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
        break;

      case 1:
      {
        // MatNd* lambdaA = MatNd_create(controller.getTaskDim(), 1);
        // const double reg = 2.0;

        // int tix = controller.getTaskIndex("Left Elbow");
        // if (tix != -1)
        // {
        //   size_t xIdx = controller.getTaskArrayIndex(tix);
        //   double* Wx_i = &lambdaA->ele[xIdx];
        //   VecNd_setElementsTo(Wx_i, reg, controller.getTaskDim(tix));
        // }

        // tix = controller.getTaskIndex("Left ABC");
        // if (tix != -1)
        // {
        //   size_t xIdx = controller.getTaskArrayIndex(tix);
        //   double* Wx_i = &lambdaA->ele[xIdx];
        //   VecNd_setElementsTo(Wx_i, reg, controller.getTaskDim(tix));
        // }

        // controller.compressToActiveSelf(lambdaA, a_des);

        // ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambdaA);
        // MatNd_destroy(lambdaA);

        ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
      }
      break;

      default:
        RFATAL("No such algorithm; %d", algo);
    }

    MatNd_constMul(q_dot_des, dq_des, 1.0/dt);

    MatNd_addSelf(controller.getGraph()->q, dq_des);
    RcsGraph_setState(controller.getGraph(), NULL, q_dot_des);
    bool poseOK = controller.checkLimits();
    controller.computeX(x_curr);

    dJlCost = -jlCost;
    jlCost = controller.computeJointlimitCost();
    dJlCost += jlCost;

    dt_calc = Timer_getTime() - dt_calc;

    pthread_mutex_unlock(&graphLock);

    if (kc && kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc && kc->getAndResetKey('a'))
    {
      algo++;
      if (algo>1)
      {
        algo = 0;
      }

      RLOGS(0, "Switching to IK algorithm %d", algo);
    }
    else if (kc && kc->getAndResetKey('T'))
    {
      RLOGS(0, "Running controller test");
      controller.test(true);
    }
    else if (kc && kc->getAndResetKey('p'))
    {
      pause = !pause;
      RMSG("Pause modus is %s", pause ? "ON" : "OFF");
    }
    else if (kc && kc->getAndResetKey('d'))
    {
      auto ts = tropic::ConstraintFactory::create("traj.xml");

      if (!ts)
      {
        RMSG("Failed to load trajectory from file \"traj.xml\"");
      }
      else
      {
        bool success = tc->addAndApply(ts, true);
        RMSG("%s loading trajectory from file \"traj.xml\"", success ? "Success" : "Failure");
        ts->print();
      }
    }
    else if (kc && kc->getAndResetKey('D'))
    {
      RMSG("Writing trajectory to \"traj_out.dat\"");
      tc->toXML("traj_out.xml");
    }
    else if (kc && kc->getAndResetKey('t'))
    {
      RMSG("Loading Johannes's class");
      auto ts = std::make_shared<tropic::PouringConstraint>();
      tc->addAndApply(ts, true);
      ts->print();
      tc->toXML("traj_out.xml");
      RMSG("Done loading Johannes's class");
    }
    else if (kc && kc->getAndResetKey('u'))
    {
      RMSG("Swapping Generic Bodies");
      static bool swap = false;
      if (swap)
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
      swap = !swap;
      RMSG("Done swapping Generic Bodies");
      tc->getInternalController()->toXML("controller.xml");
    }
    else if (kc && kc->getAndResetKey('l'))
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
      tc->addAndApply(tSet, true);
      tc->toXML("traj_out.xml");
    }
    else if (kc && kc->getAndResetKey('y'))
    {
      auto tSet = tropic::LiftObjectConstraint::pourWithOneHand(tc->getController(),
                                                                "GenericBody0",
                                                                "GenericBody3",
                                                                "GenericBody4",
                                                                "GenericBody2",
                                                                "GenericBody6",
                                                                "GenericBody7", 1.0, 8.0);
      tc->addAndApply(tSet, true);
    }
    else if (kc && kc->getAndResetKey('n'))
    {
      RMSG("Resetting");
      RcsGraph_setDefaultState(controller.getGraph());
      tc->clear(true);
      tc->setActivation(false);
      MatNd_setZero(a_des);
      controller.computeX(x_curr);
      MatNd_copy(x_des, x_curr);
      MatNd_copy(x_des_f, x_curr);
      MatNd_copy(x_des_prev, x_curr);
    }
    else if (kc && kc->getAndResetKey('C') && cn)
    {
      RMSG("Toggle closest points visualization");
      cn->toggle();
    }
    else if (kc && kc->getAndResetKey('o'))
    {
      calcDistance = !calcDistance;
      RMSG("Distance calculation is %s", calcDistance ? "ON" : "OFF");
    }
    else if (kc && kc->getAndResetKey('m'))
    {
      manipulability = !manipulability;
      RMSG("Manipulation index nullspace is %s",
           manipulability ? "ON" : "OFF");
    }
    else if (kc && kc->getAndResetKey('v'))
    {
      RcsGraph_fprintModelState(stdout, controller.getGraph(),
                                controller.getGraph()->q);
    }
    else if (kc && kc->getAndResetKey('t'))
    {
      // auto set1 = std::make_shared<tropic::ConstraintSet>();
      // auto set2 = std::make_shared<tropic::ActivationSet>();
      // auto ts = std::make_shared<tropic::ConnectBodyConstraint>(2.0, "Glas", "PowerGrasp_L");
      // set1->add(set2);
      // set2->add(ts);
      // tc->addAndApply(set1, true);

      // RLOG(0, "Adding Polar constraint in 2 seconds");
      // auto ts = std::make_shared<tropic::PolarConstraint>(2.0, 0.0, 0.0, "Polar_L");
      // tc->addAndApply(ts, true);

      // RLOG(0, "Adding PositionConstraint in 2 seconds");
      // auto ts = std::make_shared<tropic::PositionConstraint>(2.0, 0.0, 0.0, 0.0, "XYZ_L");
      // tc->addAndApply(ts, true);



      {
        RLOG(0, "Adding Polar constraint in 2 seconds");
        auto as = std::make_shared<tropic::ActivationSet>();
        as->addActivation(0.5, true, 0.5, "Polar_L");
        auto ts = std::make_shared<tropic::PolarConstraint>(2.0, 0.0, 0.0, "Polar_L");
        as->add(ts);
        tc->addAndApply(as, true);
      }

      {
        RLOG(0, "Adding PositionConstraint in 2 seconds");
        auto as = std::make_shared<tropic::ActivationSet>();
        as->addActivation(0.5, true, 0.5, "XYZ_L");
        auto ts = std::make_shared<tropic::PositionConstraint>(2.0, 0.55, 0.2, 1.0, "XYZ_L");
        as->add(ts);
        tc->addAndApply(as, true);
        as->print();
      }


    }


    sprintf(hudText, "IK calculation: %.1f us\ndof: %d nJ: %d "
            "nqr: %d nx: %d\nJL-cost: %.6f dJL-cost: %.6f %s %s"
            "\nalgo: %d lambda:%g alpha: %g\n"
            "Manipulability index: %.6f\n"
            "Static effort: %.6f\n"
            "Robot pose %s   Constraints: %d\nend time: %.3f",
            1.0e6*dt_calc, controller.getGraph()->dof,
            controller.getGraph()->nJ, ikSolver->getInternalDof(),
            (int) controller.getActiveTaskDim(a_des),
            jlCost, dJlCost,
            ikSolver->getDeterminant()==0.0?"SINGULAR":"",
            ((dJlCost > 1.0e-8) && (MatNd_getNorm(dx_des) == 0.0)) ?
            "COST INCREASE" : "",
            algo, lambda, alpha,
            controller.computeManipulabilityCost(a_des),
            RcsGraph_staticEffort(controller.getGraph(),
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

    if (pause==true || ikSolver->getDeterminant()==0.0)
    {
      RPAUSE();
    }

    loopCount++;
    Timer_waitDT(dt);
  }



  // Clean up
  if (valgrind==false)
  {
    delete v;
    RcsGuiFactory_shutdown();
  }

  tc->takeControllerOwnership(false);

  MatNd_destroy(dq_des);
  MatNd_destroy(q_dot_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x_curr);
  MatNd_destroy(x_des);
  MatNd_destroy(x_des_f);
  MatNd_destroy(x_des_prev);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(timings);

  pthread_mutex_destroy(&graphLock);

  delete ikSolver;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void testCopying()
{
  double horizon = 1.0;
  char xmlFileName[128] = "cSimpleHands.xml";
  char directory[128] = "config/xml/ECS";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", xmlFileName, "Configuration file (default is \"%s\")",
                   xmlFileName);
  argP.getArgument("-dir", directory, "Config-directory (default is \"%s\")",
                   directory);
  bool zigzag = argP.hasArgument("-zigzag", "ZigZag trajectory");

  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory);

  Rcs::ControllerBase controller(xmlFileName);
  tropic::TrajectoryControllerBase* traj = NULL;

  if (zigzag)
  {
    traj = new tropic::TrajectoryController<tropic::ZigZagTrajectory1D>(&controller, horizon);
  }
  else
  {
    traj = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(&controller, horizon);
  }

  tropic::TrajectoryControllerBase* trajCopy = new tropic::TrajectoryControllerBase(*traj);
  RPAUSE();
  delete traj;
  RPAUSE();
  delete trajCopy;
  RPAUSE();
}

/*******************************************************************************
 *
 ******************************************************************************/
// The below comment is for astyle, since otherwise it screws up the formatting
// *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
const char* xmlFileBuffer =
  MULTI_LINE_STRING(<ConstraintSet type="ActivationSet" >

                    <Activation t="0" switchesOn="true" horizon="1" trajectory="XYZ Object1" />
                    <Activation t="0" switchesOn="true" horizon="1" trajectory="Polar Object1" />
                    <Activation t="0" switchesOn="false" horizon="1" trajectory="XYZ Object2" />
                    <Activation t="0" switchesOn="false" horizon="1" trajectory="Polar Object2" />
                    <Activation t="0" switchesOn="false" horizon="1" trajectory="XYZ World" />
                    <Activation t="0" switchesOn="false" horizon="1" trajectory="Polar World" />
                    <Activation t="0" switchesOn="true" horizon="1" trajectory="Jaco1 Fingers" />

                    <ConstraintSet type="PositionConstraint" t="6" pos="0.1 0 0.1" flag="1"
                     trajectory="XYZ Object1" />
                    <ConstraintSet type="PositionConstraint" t="8" pos="0.1 0 0" trajectory="XYZ Object1" />
                    <ConstraintSet type="PolarConstraint" t="8" pos="0 0" trajectory="Polar Object1" />
                    <ConstraintSet type="ActivationSet" >

                    <Activation t="0" switchesOn="true" horizon="1" trajectory="Jaco1 Fingers" />

                    <ConstraintSet type="PositionConstraint" t="8" pos="0 0 0" trajectory="Jaco1 Fingers" />
                    </ConstraintSet>

                    <ConstraintSet type="ActivationSet" >

                    <Activation t="0" switchesOn="true" horizon="1" trajectory="Jaco1 Fingers" />

                    <ConstraintSet type="PositionConstraint" t="10" pos="0.698132 0.698132 0.698132"
                     trajectory="Jaco1 Fingers" />
                    </ConstraintSet>

                    </ConstraintSet>);
// *INDENT-ON*

static int testXML()
{
  int nErrors = 0;
  tropic::ConstraintFactory::print();
  char xmlFileName[128] = "ConstraintCollection.xml";
  char directory[128] = "config/xml/ECS";
  Rcs::CmdLineParser argP;
  argP.getArgument("-f", xmlFileName, "Configuration file (default is \"%s\")",
                   xmlFileName);
  argP.getArgument("-dir", directory, "Config-directory (default is \"%s\")",
                   directory);

  Rcs_addResourcePath("config");
  Rcs_addResourcePath(directory);

  xmlDocPtr doc;
  xmlNodePtr node = parseXMLMemory(xmlFileBuffer, strlen(xmlFileBuffer), &doc);
  //xmlNodePtr node = parseXMLFile(xmlFileName, "ConstraintSet", &doc);
  if (node==NULL)
  {
    RLOG(1, "Failed to parsed xml buffer - see xmlDump.txt for details");
    return 1;

    REXEC(1)
    {
      FILE* out = fopen("xmlDump.txt", "w+");
      RCHECK(out);
      RCHECK(doc);
      xmlDocFormatDump(out,doc,1);
    }
  }

  auto tSet = tropic::ConstraintFactory::create(node);
  if (!tSet)
  {
    RLOG(0, "Failed to create ConstraintSet from file \"%s\"", xmlFileName);
    nErrors++;
  }
  else
  {
    RLOG(1, "SUCCESS in testing xml file parsing");
    REXEC(1)
    {
      tSet->print();
    }
  }

  return nErrors;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int testXML2()
{
  int nErrors = 0;
  tropic::ConstraintFactory::print();
  Rcs::ControllerBase controller(xmlFileExplore);
  tropic::TrajectoryControllerBase* traj = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(&controller, 1.0);

  auto tt = std::make_shared<RelGrip>(traj, RCS_DEG2RAD(5.0));
  tt->addTransition(0, 14, 8, 0.0, 2.0);

  if (tt->testXML() == false)
  {
    RLOG(1, "Failure in testing xml I/O: files are different");
    nErrors++;
  }
  else
  {
    RLOG(1, "SUCCESS in testing xml I/O: files are identical");
  }

  delete traj;

  return nErrors;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int testEndTime()
{
  Rcs::CmdLineParser argP;
  double horizon = 1.0;
  argP.getArgument("-horizon", &horizon, "Trajectory horizon (default is %f)",
                   horizon);

  Rcs::ControllerBase controller(xmlFileExplore);
  tropic::TrajectoryController<tropic::ViaPointTrajectory1D> tc(&controller, horizon);

  auto moveSet = std::make_shared<tropic::ActivationSet>();
  auto moveSet2 = std::make_shared<tropic::ActivationSet>();

  moveSet2->addActivation(5.0, false, horizon, "XYZ_L");
  moveSet->add(moveSet2);
  tc.add(moveSet);
  moveSet->ConstraintSet::apply(tc.getTrajectoriesRef());

  RLOG(0, "End time 1: %f", moveSet->getEndTime());
  RLOG(0, "End time 2: %f", moveSet->compute(0.0));
  RLOG(0, "End time 3: %f", moveSet->getEndTime());

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
// The below comment is for astyle, since otherwise it screws up the formatting
// *INDENT-OFF*
#define MULTI_LINE_STRING(a) #a
const char* activationController =
MULTI_LINE_STRING(
<Controller >

<Graph >

<Body name="Ground Plane" color="PEWTER" >
<Shape type="BOX" extents="6.0 6.0 0.04" transform="0 0 -0.02 0 0 0"
       graphics="true" />
<Shape type="FRAME" scale="0.3" />
</Body>

<Body name="b1" color="RANDOM" >
<Joint name="j1" type="RotX" range="-360 20 360" />
<Shape type="SSL" radius="0.05" length="0.3" color="RANDOM" graphics="true" />
</Body>

<Body name="b2" prev="b1" color="RANDOM" >
<Joint name="j2" type="RotX" range="-360 20 360" transform="0 0 0.3 0 0 0" />
<Shape type="SSL" radius="0.05" length="0.25" graphics="true" />
</Body>

<Body name="b3" prev="b2" color="RANDOM" >
<Joint name="j3" type="RotX" range="-360 30 360" transform="0 0 0.3 0 0 0" />
<Shape type="SSL" radius="0.05" length="0.25" graphics="true" />
</Body>

<Body name="ee" prev="b3" transform="0 0 0.3 0 0 0" >
<Shape type="FRAME" scale="0.25" />
</Body>

</Graph>

<Task name="y" controlVariable="Y" effector="ee" />
<Task name="z" controlVariable="Z" effector="ee" />
<Task name="jnts1" controlVariable="Joints" jnts="j1 j2 j3" />

</Controller>);
// *INDENT-ON*

static tropic::TCS_sptr genTraj1(const std::string& fileName, double activationHorizon)
{

  if (fileName=="1")
  {
    auto moveSet = std::make_shared<tropic::ActivationSet>();

    moveSet->addActivation(0.0, false, activationHorizon, "y");
    moveSet->addActivation(0.0, false, activationHorizon, "z");
    moveSet->addActivation(0.0, true, activationHorizon, "jnts1");

    for (size_t i=0; i<3; ++i)
    {
      moveSet->add(4.0, RCS_DEG2RAD(30.0), 0.0, 0.0, 7, "jnts1 " + std::to_string(i));
    }

    moveSet->addActivation(2.0, true, activationHorizon, "y");
    moveSet->addActivation(2.0, true, activationHorizon, "z");
    moveSet->addActivation(2.0, false, activationHorizon, "jnts1");
    moveSet->add(6.0, 0.0, 0.0, 0.0, 7, "z 0");

    moveSet->addActivation(4.0, false, activationHorizon, "y");
    moveSet->addActivation(4.0, false, activationHorizon, "z");
    moveSet->addActivation(4.0, true, activationHorizon, "jnts1");

    for (size_t i=0; i<3; ++i)
    {
      moveSet->add(8.0, RCS_DEG2RAD(-30.0), 0.0, 0.0, 7, "jnts1 " + std::to_string(i));
    }


    moveSet->addActivation(7.0, true, activationHorizon, "y");
    moveSet->addActivation(7.0, true, activationHorizon, "z");
    moveSet->addActivation(7.0, false, activationHorizon, "jnts1");
    moveSet->add(10.0, 0.0, 0.0, 0.0, 7, "z 0");


    return moveSet;
  }
  if (fileName=="2")
  {
    auto moveSet = std::make_shared<tropic::ActivationSet>();

    double t_start = 0.0;
    moveSet->addActivation(t_start, false, activationHorizon, "y");
    moveSet->addActivation(t_start, false, activationHorizon, "z");
    moveSet->addActivation(t_start, true, activationHorizon, "jnts1");
    moveSet->addActivation(t_start, false, activationHorizon, "jnts2");

    double t1 = 2.0;
    for (size_t i=0; i<7; ++i)
    {
      moveSet->add(t1, RCS_DEG2RAD(10.0), 0.0, 0.0, 7, "jnts1 " + std::to_string(i));
    }

    double t2 = 4.0;
    moveSet->addActivation(t1, true, activationHorizon, "y");
    moveSet->addActivation(t1, true, activationHorizon, "z");
    moveSet->addActivation(t1, false, activationHorizon, "jnts1");
    moveSet->addActivation(t1, false, activationHorizon, "jnts2");
    moveSet->add(t2, 0.0, 0.0, 0.0, 7, "z 0");

    double t3 = 6.0;
    moveSet->addActivation(t2, false, activationHorizon, "y");
    moveSet->addActivation(t2, false, activationHorizon, "z");
    moveSet->addActivation(t2, false, activationHorizon, "jnts1");
    moveSet->addActivation(t2, true, activationHorizon, "jnts2");
    for (size_t i=0; i<7; ++i)
    {
      moveSet->add(t3, RCS_DEG2RAD(0.0), 0.0, 0.0, 7, "jnts2 " + std::to_string(i));
    }

    return moveSet;
  }
  else if (fileName=="3")
  {
    auto moveSet = std::make_shared<tropic::ActivationSet>();

    moveSet->addActivation(0.0, true, activationHorizon, "z");
    moveSet->add(2.0, 0.0, 0.0, 0.0, 7, "z 0");

    return moveSet;
  }
  else if (!fileName.empty())
  {
    return tropic::ConstraintFactory::create(fileName);
  }


  return std::make_shared<tropic::ActivationSet>();
}

static void testDynamicActivation(int argc, char** argv)
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");

  pthread_mutex_t mtx;
  pthread_mutex_init(&mtx, NULL);

  char hudText[4096] = "";
  double horizon=1.0, activationHorizon=0.5, time=0.0, dt=0.01, alpha=0.0, lambda=1.0e-6;
  double qFilt = 0.0, qFiltDecay = 0.0, clipLimit = DBL_MAX;
  std::string trajName = "1";
  Rcs::CmdLineParser argP;
  argP.getArgument("-lambda", &lambda, "IK regularization (default: %f)", lambda);
  argP.getArgument("-alpha", &alpha, "Null space factor (default: %f)", alpha);
  argP.getArgument("-dt", &dt, "Time step (default: %f)", dt);
  argP.getArgument("-f", &trajName, "Trajectory file name (default: %s)", trajName.c_str());
  argP.getArgument("-horizon", &horizon, "Trajectory horizon (default is %f)", horizon);
  argP.getArgument("-clipLimit", &clipLimit, "Clip limit for dx (default: none)");
  argP.getArgument("-filtDecay", &qFiltDecay, "Joint velocity decay filter (default: %f sec)", qFiltDecay);
  bool valgrind = argP.hasArgument("-valgrind", "Start without Guis and graphics");
  bool zeroVel = argP.hasArgument("-zeroVel", "Reactivate with zero velocity");

  tropic::ViaTrajectoryController tc(activationController, horizon);
  tc.setActivation(true);
  tc.reactivateWithZeroVelAcc(zeroVel);

  Rcs::IkSolverRMR ikSolver(tc.getInternalController());
  ikSolver.setActivationBlending("IndependentTasks");
  Rcs::ControllerBase* cntrl = tc.getInternalController();

  const unsigned int nx = cntrl->getTaskDim();
  const unsigned int nt = cntrl->getNumberOfTasks();
  const unsigned int nq = cntrl->getGraph()->dof;
  MatNd* a_prev     = MatNd_create(nt, 1);
  MatNd* a_cont     = MatNd_create(nt, 1);
  MatNd* x_curr     = MatNd_create(nx, 1);
  MatNd* x_dot_curr = MatNd_create(nx, 1);
  MatNd* x_des      = MatNd_create(nx, 1);
  MatNd* dx_des     = MatNd_create(nx, 1);
  MatNd* dH         = MatNd_create(1, nq);
  MatNd* dq_des     = MatNd_createLike(cntrl->getGraph()->q);
  MatNd* plotMe     = MatNd_create(1000, 1+2*nx+2*nq+nt+2*nx+nt);// time, x, dx, q, qd, a, x_curr, x_dot_curr
  MatNd* q          = cntrl->getGraph()->q;
  MatNd* q_dot      = cntrl->getGraph()->q_dot;
  plotMe->m = 0;

  cntrl->computeX(x_curr);
  MatNd_copy(x_des, x_curr);
  MatNd_copy(a_prev, tc.getActivationPtr());

  auto moveSet = genTraj1(trajName, activationHorizon);
  RCHECK(moveSet);
  tc.addAndApply(moveSet);
  moveSet->toXML("ActivationTrajectory.xml");
  double endTime = moveSet->getEndTime();



  std::unique_ptr<Rcs::Viewer> viewer;
  osg::ref_ptr<Rcs::KeyCatcher> kc;
  osg::ref_ptr<Rcs::HUD> hud;

  if (!valgrind)
  {
    viewer.reset(new Rcs::Viewer());
    Rcs::GraphNode* gn = new Rcs::GraphNode(cntrl->getGraph());
    gn->toggleReferenceFrames();
    kc = new Rcs::KeyCatcher();
    viewer->add(kc);
    viewer->add(gn);
    hud = new Rcs::HUD();
    viewer->add(hud);
    viewer->setUpdateFrequency(60.0);
    viewer->runInThread(&mtx);
    Rcs::ControllerWidgetBase::create(cntrl, (MatNd*)tc.getActivationPtr(),
                                      x_des, x_curr, &mtx, true);
  }


  while (runLoop)
  {
    pthread_mutex_lock(&mtx);
    RLOG(5, "Starting time step %.3f", time);

    /////////////////////////////////////////////////////////////////
    // Trajectory
    /////////////////////////////////////////////////////////////////
    MatNd_copy(a_prev, tc.getActivationPtr());
    endTime = tc.step(dt);
    tc.getContinuousActivation(a_cont);
    tc.getPosition(x_des);
    bool taskSwitch = !MatNd_isEqual(a_prev, tc.getActivationPtr(), 1.0e-3);
    if (taskSwitch)
    {
      qFilt = 1.0;
      RLOG(0, "Switch-point at t=%f", time);
    }
    if (qFiltDecay<=0.0)
    {
      qFilt = 0.0;
    }
    else
    {
      qFilt = Math_clip(qFilt-(1.0/qFiltDecay)*dt, 0, 1.0);
    }

    /////////////////////////////////////////////////////////////////
    // Inverse kinematics
    /////////////////////////////////////////////////////////////////
    const double blending = 1.0;//tc.computeBlending();
    cntrl->computeDX(dx_des, x_des);
    MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
    MatNd_saturateSelf(dx_des, &clipArr);
    //MatNd_eleMulSelf(dx_des, ikSolver.getCurrentActivation());
    cntrl->computeJointlimitGradient(dH);
    MatNd_constMulSelf(dH, alpha*blending);
    ikSolver.solveRightInverse(dq_des, dx_des, dH, tc.getActivationPtr(), lambda);
    //ikSolver.solveLeftInverse(dq_des, dx_des, dH, a_cont, lambda);
    RCHECK(ikSolver.getDeterminant()>0.0);
    MatNd_constMulSelf(dq_des, 1.0/dt);

    // First order filter with decaying time constant at task switch points
    for (unsigned int i=0; i<dq_des->m; ++i)
    {
      q_dot->ele[i] = qFilt*q_dot->ele[i] + (1.0-qFilt)*dq_des->ele[i];
    }

    MatNd_constMulSelf(q_dot, dt);
    MatNd_addSelf(q, q_dot);
    MatNd_constMulSelf(q_dot, 1.0/dt);
    RcsGraph_setState(cntrl->getGraph(), q, q_dot);
    cntrl->computeX(x_curr);
    cntrl->computeXp(x_dot_curr);
    pthread_mutex_unlock(&mtx);

    /////////////////////////////////////////////////////////////////
    // Plotting
    /////////////////////////////////////////////////////////////////
    double* q_row = new double[plotMe->n];
    MatNd qi = MatNd_fromPtr(1, plotMe->n, q_row);
    q_row[0] = time;
    VecNd_copy(&q_row[1], x_des->ele, nx);                           // 2-6
    VecNd_copy(&q_row[nx+1], dx_des->ele, nx);                       // 7-11
    VecNd_copy(&q_row[2*nx+1], q->ele, nq);                          // 12-14
    VecNd_copy(&q_row[2*nx+nq+1], q_dot->ele, nq);                   // 15-17
    VecNd_copy(&q_row[2*nx+2*nq+1], tc.getActivationPtr()->ele, nt); // 18-20
    VecNd_copy(&q_row[nt+2*nx+2*nq+1], x_curr->ele, nx);             // 21-25
    VecNd_copy(&q_row[nx+nt+2*nx+2*nq+1], x_dot_curr->ele, nx);      // 26-30
    VecNd_copy(&q_row[nx+nx+nt+2*nx+2*nq+1], a_cont->ele, nt);       // 31-33
    MatNd_appendRows(plotMe, &qi);
    MatNd_appendToFile(&qi, "motion2.dat");
    delete [] q_row;

    /////////////////////////////////////////////////////////////////
    // Keycatcher
    /////////////////////////////////////////////////////////////////
    if (kc.valid() && kc->getAndResetKey('q'))
    {
      runLoop = false;
    }
    else if (kc.valid() && kc->getAndResetKey('t'))
    {
      moveSet->toXML("traj.xml");
    }

    //////////////////////////////////////////////////////////////
    // HUD
    /////////////////////////////////////////////////////////////////
    sprintf(hudText, "Time: %.3f  end-time: %.3f  blending: %.3f  filt: %.3f",
            time, endTime, blending, qFilt);

    if (hud.valid())
    {
      hud->setText(hudText);
    }
    else
    {
      std::cout << hudText;
    }

    //////////////////////////////////////////////////////////////
    // Prepare next step
    /////////////////////////////////////////////////////////////////
    if (!valgrind)
    {
      Timer_waitDT(dt);
    }
    else
    {
      if (endTime<=0.0)
      {
        runLoop = false;
      }
    }

    time += dt;
    RPAUSE_DL(2);
  }

  MatNd_toFile(plotMe, "motion.dat");
  printf("Here is some useful plot commands:\n\n");

  printf("plot \"motion.dat\" u 1:7 w lp title \"vel y\", \"motion.dat\" u 1:8 w lp title \"vel z\", "
         "\"motion.dat\" u 1:9 w lp title \"vel j1\", \"motion.dat\" u 1:10 w lp title \"vel j2\", "
         "\"motion.dat\" u 1:11 w lp title \"vel j3\"\n");

  printf("plot \"motion.dat\" u 1:7 w lp title \"vel y\", \"motion.dat\" u 1:8 w lp title \"vel z\", "
         "\"motion.dat\" u 1:9 w lp title \"vel j1\", \"motion.dat\" u 1:10 w lp title \"vel j2\", "
         "\"motion.dat\" u 1:11 w lp title \"vel j3\"\n");

  printf("plot \"motion.dat\" u 1:12 w lp title \"q1\", \"motion.dat\" u 1:13 w lp title \"q2\", "
         "\"motion.dat\" u 1:14 w lp title \"q3\"\n");

  printf("plot \"motion.dat\" u 1:($15*180.0/pi) w lp title \"qd1\", "
         "\"motion.dat\" u 1:($16*180.0/pi) w lp title \"qd2\", "
         "\"motion.dat\" u 1:($17*180.0/pi) w lp title \"qd3\"\n");

  printf("plot \"motion.dat\" u 1:18 w lp title \"ay\", \"motion.dat\" u 1:19 w lp title \"az\", "
         "\"motion.dat\" u 1:20 w lp title \"aj\"\n");

  printf("plot \"motion.dat\" u 1:21 w lp title \"pos y curr\", \"motion.dat\" u 1:22 w lp title \"pos z curr\", "
         "\"motion.dat\" u 1:23 w lp title \"pos j1 curr\", \"motion.dat\" u 1:24 w lp title \"pos j2 curr\", "
         "\"motion.dat\" u 1:25 w lp title \"pos j3 curr\"\n");

  printf("plot \"motion.dat\" u 1:26 w lp title \"vel y curr\", \"motion.dat\" u 1:27 w lp title \"vel z curr\", "
         "\"motion.dat\" u 1:28 w lp title \"vel j1 curr\", \"motion.dat\" u 1:29 w lp title \"vel j2 curr\", "
         "\"motion.dat\" u 1:30 w lp title \"vel j3 curr\"\n");

  printf("plot \"motion.dat\" u 1:31 w lp title \"ay\", \"motion.dat\" u 1:32 w lp title \"az\", "
         "\"motion.dat\" u 1:33 w lp title \"aj\"\n");

  RcsGuiFactory_shutdown();

  MatNd_destroy(a_cont);
  MatNd_destroy(x_curr);
  MatNd_destroy(x_dot_curr);
  MatNd_destroy(x_des);
  MatNd_destroy(dx_des);
  MatNd_destroy(dq_des);
  MatNd_destroy(dH);
  MatNd_destroy(plotMe);

  pthread_mutex_destroy(&mtx);
}


/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  int mode = 0, nErrors = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default: %d)", RcsLogLevel);
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  Rcs_addResourcePath(RCS_CONFIG_DIR);

  switch (mode)
  {
    case 0:
      testInteractive();
      break;

    case 1:
      testViaPointTrajectory1D();
      break;

    case 2:
      testTrajectory1D();
      break;

    case 3:
      testExplore();
      break;

    case 5:
      testIK();
      break;

    case 6:
      testCopying();
      break;

    case 7:
      nErrors += testXML();
      break;

    case 8:
      nErrors += testXML2();
      break;

    case 9:
      testEndTime();
      break;

    case 10:
      testLinearAccelerationTrajectory(argc, argv);
      break;

    case 11:
      testDynamicActivation(argc, argv);
      break;

    case 12:
      testIK2();
      break;

    default:
      RMSG("No mode %d", mode);
  }

  if (argP.hasArgument("-h", "Show help message"))
  {
    Rcs::KeyCatcherBase::printRegisteredKeys();
    argP.print();
    Rcs_printResourcePath();
    tropic::ConstraintFactory::print();

    printf("\nHere's some useful testing modes:\n\n");
    printf("\t-m");
    printf("\t0   3d trajectory test with viewer (default)\n");
    printf("\t\t1   ViaPointTrajectory plotter test\n");
    printf("\t\t2   Trajectory1D plotter test\n");
    printf("\t\t3   Box planner trajectory\n");
    printf("\t\t5   Trajectories as input to IK\n");
    printf("\t\t6   Copying TrajectoryController test\n");
    printf("\t\t7   Loading from xml file test\n");
    printf("\t\t8   Loading from xml file test\n");
    printf("\t\t9   Test computation of end time\n");
    printf("\t\t10  Test trajectories with linear acceleration segments\n");
    printf("\t\t11  Test \"ballistic\" activation\n");
    printf("\n");
  }

  xmlCleanupParser();
  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return nErrors;
}
