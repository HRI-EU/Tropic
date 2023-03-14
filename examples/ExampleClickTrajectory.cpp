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

#include "ExampleClickTrajectory.h"

#include <PositionTrajectoryNode.h>
#include <ActivationSet.h>
#include <PositionTrajectoryNode.h>
#include <ConstraintFactory.h>
#include <AnchoredPositionConstraint.h>

#include <GraphNode.h>
#include <KeyCatcher.h>

#include <ExampleFactory.h>
#include <IkSolverConstraintRMR.h>
#include <TaskPosition3D.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_body.h>



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
    for (size_t j=0; j<this->children.size(); ++j)
    {
      for (size_t i = 0; i< children[j]->numConstraints(false); ++i)
      {
        if ((children[j]->getConstraint(i)->getTime()<0.1) &&
            (children[j]->getConstraint(i)->getFlag()==1))
        {
          children[j]->getConstraint(i)->setFlag(0);
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

namespace tropic
{

RCS_REGISTER_EXAMPLE(ExampleClickTrajectory, "Trajectory", "Click");

ExampleClickTrajectory::ExampleClickTrajectory(int argc, char** argv) :
  Rcs::ExampleBase(argc, argv)
{
  pthread_mutex_init(&mtx, NULL);

  freeze = false;
  zigzag = false;
  valgrind = false;
  time = 0.0;
  dt = 0.01;
  motionEndTime = 0.0;
  horizon = 1.0;
  ttc = 2.0;
  cursor = NULL;
  box = NULL;
  sphere = NULL;
  torus = NULL;
  boxJnt = NULL;
  sphereJnt = NULL;
  torusJnt = NULL;
  cursorJnt = NULL;
  tc = NULL;
  viewer = NULL;
}

ExampleClickTrajectory::~ExampleClickTrajectory()
{
  delete tc;
  delete viewer;
  pthread_mutex_destroy(&mtx);
}

bool ExampleClickTrajectory::initParameters()
{
  Rcs::KeyCatcherBase::registerKey("q", "Quit");
  Rcs::KeyCatcherBase::registerKey("d", "Clear trajectories");
  Rcs::KeyCatcherBase::registerKey("p", "Print Trajectories to stdout");
  Rcs::KeyCatcherBase::registerKey("f", "Freeze time");
  Rcs::KeyCatcherBase::registerKey("x", "Create constraint under mouse");

  return true;
}

bool ExampleClickTrajectory::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-ttc", &ttc, "Time to contact (default: %f)", ttc);
  parser->getArgument("-horizon", &horizon, "Time horizon (default: %f)", horizon);
  parser->getArgument("-zigzag", &zigzag, "ZigZag trajectory");
  parser->getArgument("-valgrind", &valgrind, "Start without Guis and graphics");

  return true;
}

bool ExampleClickTrajectory::initAlgo()
{
  RcsGraph* graph = RcsGraph_create(interactiveGraph);
  Rcs::ControllerBase* controller = new Rcs::ControllerBase(graph);

  cursor = RcsGraph_getBodyByName(graph, "cursor");
  box = RcsGraph_getBodyByName(graph, "box");
  sphere = RcsGraph_getBodyByName(graph, "sphere");
  torus = RcsGraph_getBodyByName(graph, "torus");
  RCHECK(cursor && box && sphere && torus);

  boxJnt = RCSJOINT_BY_ID(graph, box->jntId);
  sphereJnt = RCSJOINT_BY_ID(graph, sphere->jntId);
  torusJnt = RCSJOINT_BY_ID(graph, torus->jntId);
  cursorJnt = RCSJOINT_BY_ID(graph, cursor->jntId);
  RCHECK(boxJnt && sphereJnt && torusJnt && cursorJnt);

  Rcs::TaskPosition3D* posTask = new Rcs::TaskPosition3D(graph, cursor, NULL, NULL);
  posTask->setName("Trajectory position");
  controller->add(posTask);

  if (zigzag)
  {
    tc = new tropic::TrajectoryController<tropic::ZigZagTrajectory1D>(controller, horizon);
  }
  else
  {
    tc = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(controller, horizon);
  }

  tc->setActivation(true);
  tc->setActivation(0, true);
  tc->setTurboMode(false);
  tc->takeControllerOwnership(true);

  return true;
}

bool ExampleClickTrajectory::initGraphics()
{
  if (valgrind)
  {
    return true;
  }

  viewer = new Rcs::Viewer();
  viewer->add(new Rcs::GraphNode(tc->getController()->getGraph()));
  kc = new Rcs::KeyCatcher();
  hud = new Rcs::HUD();
  viewer->add(hud.get());
  viewer->add(kc.get());
  tropic::TrajectoryND* posTraj = tc->getTrajectory(0);
  RCHECK(posTraj);
  osg::ref_ptr<tropic::PositionTrajectoryNode> pn = new tropic::PositionTrajectoryNode(posTraj);
  //pn->setPointSize(10.0);
  viewer->add(pn.get());
  viewer->runInThread(&mtx);

  return true;
}

void ExampleClickTrajectory::step()
{
  MatNd* x = MatNd_create((int) tc->getController()->getTaskDim(), 1);
  RcsGraph* graph = tc->getInternalController()->getGraph();
  double t_loop = Timer_getSystemTime();

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
  // HUD
  //////////////////////////////////////////////////////////////////
  char hudText[1024] = "";
  snprintf(hudText, 1024, "Time: %.3f calculation: %.2f msec\n"
           "dt: %.1f msec %s\nConstraints: %d (%d %d)\n"
           "Sets: %d\n"
           "Motion end: %.3f", time, t_calc*1000.0, freeze ? 0.0 : 1000.0*dt,
           freeze ? "(frozen)" : "",
           (int) tc->getNumberOfSetConstraints(),
           (int) tc->getNumberOfConstraints(),
           (int) tropic::Constraint1D::getNumConstraints(),
           (int) tc->getNumberOfSets(),
           motionEndTime);

  if (hud.valid())
  {
    hud->setText(hudText);
  }

  //////////////////////////////////////////////////////////////////
  // Timing
  //////////////////////////////////////////////////////////////////
  time += dt;
  t_loop = Timer_getSystemTime() - t_loop;
  Timer_waitDT(dt-t_loop);

  if ((valgrind==true) && (time>1.0))
  {
    RLOG(0, "Quitting run loop in valgrind mode");
    runLoop = false;
  }

  MatNd_destroy(x);
}

void ExampleClickTrajectory::handleKeys()
{
  if (!kc.valid())
  {
    return;
  }

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
      tropic::TrajectoryND* posTraj = tc->getTrajectory(0);
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

  }

}

}   // namespace tropic
