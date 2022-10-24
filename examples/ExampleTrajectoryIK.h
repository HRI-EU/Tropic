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

#ifndef RCS_EXAMPLETRAJECTORYIK_H
#define RCS_EXAMPLETRAJECTORYIK_H

#include "TrajectoryController.h"

#include <ExampleBase.h>
#include <IkSolverRMR.h>
#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <GraphNode.h>
#include <HUD.h>
#include <BodyPointDragger.h>
#include <VertexArrayNode.h>
#include <ControllerWidgetBase.h>
#include <JointWidget.h>
#include <MatNdWidget.h>


extern "C" {
  void TropicExampleInfo();
}

namespace tropic
{

class ExampleTrajectoryIK : public Rcs::ExampleBase
{
public:

  int algo;
  unsigned int loopCount;
  double alpha, lambda, dt, dt_calc, determinant;
  double jlCost, dJlCost, horizon;
  bool calcDistance;
  bool pause, launchJointWidget, manipulability, cAvoidance, constraintIK;
  bool valgrind, simpleGraphics, zigzag, showOnly, noTaskGui, permissive;
  bool nomutex;
  bool showTimingsGui;
  std::string xmlFileName;
  std::string directory;
  std::string effortBdyName;
  int effortBdyId;

  pthread_mutex_t graphLock;
  pthread_mutex_t* mtx;

  Rcs::ControllerBase* controller;
  Rcs::IkSolverRMR* ikSolver;

  MatNd* dq_des;
  MatNd* q_dot_des;
  MatNd* a_des;
  MatNd* x_curr;
  MatNd* x_des;
  MatNd* x_des_prev;
  MatNd* x_des_f;
  MatNd* dx_des;
  MatNd* dH;
  MatNd* timings;
  MatNd* F_effort;

  std::shared_ptr<tropic::TrajectoryControllerBase> tc;

  // Visualization
  Rcs::Viewer* viewer;
  osg::ref_ptr<Rcs::KeyCatcher> kc;
  osg::ref_ptr<Rcs::GraphNode> gn;
  osg::ref_ptr<Rcs::HUD> hud;
  osg::ref_ptr<Rcs::BodyPointDragger> dragger;
  osg::ref_ptr<Rcs::VertexArrayNode> cn;
  char hudText[2056];

  // Guis
  Rcs::ControllerGui* cGui;
  Rcs::JointGui* jGui;
  Rcs::MatNdGui* mGuiTimings;
  Rcs::MatNdGui* mGuiEffort;


  ExampleTrajectoryIK(int argc, char** argv);
  virtual ~ExampleTrajectoryIK();
  virtual void initParameters();
  virtual void parseArgs(Rcs::CmdLineParser* parser);
  virtual bool initAlgo();
  virtual void initGraphics();
  virtual void initGuis();
  virtual void step();
  virtual void handleKeys();
  virtual std::string help();
  virtual void clear();
};

}   // namespace tropic

#endif   // RCS_EXAMPLEFK_H
