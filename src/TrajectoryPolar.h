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

#ifndef TROPIC_TRAJECTORYPOLAR_H
#define TROPIC_TRAJECTORYPOLAR_H


namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for computing 2-dimensional trajectories of Polar angles.
 *
 *         The class implements some methods to convert the internal
 *         3-dimensional axis representation from and to 2d Polar angles.
 *         This is a prerequisite for computing 2d orientation trajectories
 *         that have no issues with flipping directions etc.
 */
template<class T>
class TrajectoryPolar : public TrajectoryND
{
public:
  TrajectoryPolar(const double* x0, double horizon=1.0) : TrajectoryND()
  {
    double axis[3];
    Vec3d_getPolarAxis(axis, x0[0], x0[1]);

    addTrajectory1D(new T(axis[0], horizon));
    addTrajectory1D(new T(axis[1], horizon));
    addTrajectory1D(new T(axis[2], horizon));
  }

  TrajectoryPolar(const TrajectoryPolar& copyFromMe) : TrajectoryND(copyFromMe)
  {
  }

  virtual TrajectoryPolar* clone() const
  {
    return new TrajectoryPolar(*this);
  }

  virtual ~TrajectoryPolar()
  {
  }

  virtual unsigned int getDim() const
  {
    return 2;
  }

  virtual const char* getClassName() const
  {
    return "TrajectoryPolar";
  }

  void addConstraint(double t,
                     const double* x,
                     const double* x_dot,
                     const double* x_ddot,
                     int flag)
  {
    double polarAxis[3];
    toInternalCoords(polarAxis, x);
    TrajectoryND::addConstraint(t, polarAxis, Vec3d_zeroVec(),
                                Vec3d_zeroVec(), flag);
  }

  void toInternalCoords(double* polarAxis, const double* polarAngles) const
  {
    Vec3d_getPolarAxis(polarAxis, polarAngles[0], polarAngles[1]);
  }

  bool fromInternalCoords(double* polarAngles, const double* polarAxis) const
  {
    double I_axis[3];
    double len = Vec3d_normalize(I_axis, polarAxis);

    if (len==0.0)
    {
      polarAngles[0] = 0.0;
      polarAngles[1] = 0.0;
      return false;
    }

    Vec3d_getPolarAngles(polarAngles, I_axis);
    return true;
  }

};

}   // namespace tropic


#endif   // TROPIC_TRAJECTORYPOLAR_H
