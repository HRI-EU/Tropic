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

#include "TrajectoryPlotter1D.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>

#include <algorithm>



namespace tropic
{

/*******************************************************************************
 * Gnuplot class for Trajectory1D class
 ******************************************************************************/
TrajectoryPlotter1D::TrajectoryPlotter1D():
  pipe(NULL),
  fixAxes(true),
  lowerLimitX(0.0),
  upperLimitX(0.0),
  lowerLimitY(0.0),
  upperLimitY(0.0)
{
#if defined(_MSC_VER)
  this->pipe = _popen("pgnuplot.exe -persist", "w");
#else
  this->pipe = popen("gnuplot -persist", "w");
#endif

  if (this->pipe == NULL)
  {
    RLOG(1, "Couldn't open pipe to gnuplot");
    throw (std::string("Couldn't open pipe to gnuplot"));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
TrajectoryPlotter1D::~TrajectoryPlotter1D()
{
  pclose(this->pipe);
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryPlotter1D::setRangeX(double lowerLimit, double upperLimit)
{
  this->lowerLimitX = lowerLimit;
  this->upperLimitX = upperLimit;
  this->fixAxes = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryPlotter1D::setRangeY(double lowerLimit, double upperLimit)
{
  this->lowerLimitY = lowerLimit;
  this->upperLimitY = upperLimit;
  this->fixAxes = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void TrajectoryPlotter1D::plot(const Trajectory1D& via,
                               double t0, double t1, double dt)
{
  if (t1 <= t0)
  {
    RLOG(1, "t1 <= t0: t1=%f t0=%f", t1, t0);
    return;
  }

  // Calculate trajectory and write it to file in gnuplot-compatible conventions
  unsigned int nSteps = 1+lround((t1-t0)/dt);
  MatNd* traj = MatNd_create(nSteps, 2);

  for (unsigned int i=0; i<nSteps; ++i)
  {
    double t = t0+i*dt;
    MatNd_set(traj, i, 0, t);
    MatNd_set(traj, i, 1, via.getPosition(t));
  }

  MatNd_transposeSelf(traj);
  const double* pos = MatNd_getRowPtr(traj, 1);
  lowerLimitY = std::min(lowerLimitY, VecNd_minEle(pos, traj->n)-1.0);
  upperLimitY = std::max(upperLimitY, VecNd_maxEle(pos, traj->n)+1.0);
  MatNd_transposeSelf(traj);

  lowerLimitX = std::min(lowerLimitX, t0);
  upperLimitX = std::max(upperLimitX, t1);

  // Write gnuplot command strings to pipe
  if (this->fixAxes == true)
  {
    fprintf(this->pipe, "set xrange [%f:%f]\n", lowerLimitX, upperLimitX);
    fprintf(this->pipe, "set yrange [%f:%f]\n", lowerLimitY, upperLimitY);
  }
  else
  {
    fprintf(this->pipe, "set autoscale\n");
  }

  fprintf(this->pipe, "set grid\n");
  fprintf(this->pipe, "plot '-' w l title \"x\"\n");

  for (unsigned int i=0; i<traj->m; ++i)
  {
    fprintf(pipe,"%f, %f\n", MatNd_get(traj, i, 0), MatNd_get(traj, i, 1));
  }

  fprintf(this->pipe,"e\n");
  fflush(this->pipe);
}

}   // namespace tropic
