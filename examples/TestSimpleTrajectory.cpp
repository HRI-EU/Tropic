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

#include <ConstraintSet.h>

#include <TrajectoryPlotter1D.h>
#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_parser.h>
#include <SegFaultHandler.h>

#include <iostream>
#include <csignal>

RCS_INSTALL_ERRORHANDLERS

bool runLoop = true;



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

  tropic::ViaPointTrajectory1D via(1.0, horizon);
  RCHECK(via.check());
  via.addConstraint(0.5, 2.0, 0.0, 0.0, 1);
  via.addConstraint(0.25, -2.0, 0.0, 0.0, 7);
  via.addConstraint(0.75, -1.0, 0.0, 0.0, 1);
  via.addConstraint(1.0, 5.0, 0.0, 0.0, 7);
  via.addConstraint(1.95, 5.0, 0.0, 0.0, 7);
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
      plotter->plot2(*viaSeq, -0.2, horizon+0.2, dt, flag);
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


  traj->addConstraint(0.5, 2.0, 0.0, 0.0, 1);
  traj->addConstraint(0.25, -2.0, 0.0, 0.0, 7);
  traj->addConstraint(0.75, -1.0, 0.0, 0.0, 1);
  traj->addConstraint(1.0, 5.0, 0.0, 0.0, 7);
  traj->addConstraint(1.95, 5.0, 0.0, 0.0, 7);
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
int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  // This initialize the xml library and check potential mismatches between
  // the version it was compiled for and the actual shared library used.
  LIBXML_TEST_VERSION;

  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default: %d)", RcsLogLevel);
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);
  Rcs_addResourcePath(RCS_CONFIG_DIR);

  switch (mode)
  {
    case 0:
    {
      printf("\nHere's some useful testing modes:\n\n");
      printf("\t-m");
      printf("\t1   Via point trajectory test\n");
      printf("\t\t2   Trajectory1D test\n");
      printf("\n");
      break;
    }

    case 1:
      testViaPointTrajectory1D();
      break;

    case 2:
      testTrajectory1D();
      break;

    default:
      RMSG("No mode %d", mode);
  }

  if (argP.hasArgument("-h", "Show help message"))
  {
    argP.print();
    Rcs_printResourcePath();
  }

  xmlCleanupParser();
  fprintf(stderr, "Thanks for using the Rcs libraries\n");

  return 0;
}
