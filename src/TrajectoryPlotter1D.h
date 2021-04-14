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

#ifndef TROPIC_TRAJECTORYPLOTTER1D_H
#define TROPIC_TRAJECTORYPLOTTER1D_H

#include "Trajectory1D.h"



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for plotting 1-dimensional trajectories to gnuplot.
 *
 *         This convenience class opens a pipe to gnuplot and sends the
 *         trajectory data through it in each plot() call.
 */
class TrajectoryPlotter1D
{
public:
  TrajectoryPlotter1D();
  virtual ~TrajectoryPlotter1D();
  void plot(const Trajectory1D& via, double t0, double t1, double dt=0.01);
  void setRangeY(double lowerLimit, double upperLimit);
  void setRangeX(double lowerLimit, double upperLimit);

protected:
  FILE* pipe;
  bool fixAxes;
  double lowerLimitX;
  double upperLimitX;
  double lowerLimitY;
  double upperLimitY;
};

}   // namespace tropic


#endif   // TROPIC_TRAJECTORYPLOTTER1D_H
