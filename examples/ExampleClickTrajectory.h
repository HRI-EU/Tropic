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

#ifndef TROPIC_CLICKTRAJECTORY_H
#define TROPIC_CLICKTRAJECTORY_H

#include "TrajectoryController.h"

#include <ExampleBase.h>
#include <IkSolverRMR.h>
#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <HUD.h>


namespace tropic
{
class ContactConstraint;

class ExampleClickTrajectory : public Rcs::ExampleBase
{
public:

  bool freeze;
  bool zigzag;
  bool valgrind;
  double time;
  double dt;
  double motionEndTime;
  double horizon;
  double ttc;
  const RcsBody* cursor;
  const RcsBody* box;
  const RcsBody* sphere;
  const RcsBody* torus;
  const RcsJoint* boxJnt;
  const RcsJoint* sphereJnt;
  const RcsJoint* torusJnt;
  const RcsJoint* cursorJnt;
  TrajectoryControllerBase* tc;

  Rcs::Viewer* viewer;
  osg::ref_ptr<Rcs::KeyCatcher> kc;
  osg::ref_ptr<Rcs::HUD> hud;

  std::shared_ptr<tropic::ContactConstraint> prevContact, lastContact;

  pthread_mutex_t mtx;


  ExampleClickTrajectory(int argc, char** argv);
  virtual ~ExampleClickTrajectory();
  virtual bool initParameters();
  virtual bool parseArgs(Rcs::CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual void step();
  virtual void handleKeys();
  /* virtual std::string help(); */
  /* virtual void clear(); */
};

}   // namespace tropic

#endif   // TROPIC_CLICKTRAJECTORY_H
