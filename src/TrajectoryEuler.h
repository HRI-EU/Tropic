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

#ifndef TROPIC_TRAJECTORYEULER_H
#define TROPIC_TRAJECTORYEULER_H

#include <Rcs_quaternion.h>
#include <Rcs_math.h>


namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for computing 3-dimensional trajectories of Euler angles.
 *
 *         The class implements some methods to convert the internal
 *         4-dimensional quaternion representation from and to 3d Euler angles.
 *         This is a prerequisite for computing 3d orientation trajectories
 *         that have no issues with flipping directions etc.
 */
template<class T>
class TrajectoryEuler : public TrajectoryND
{
public:
  TrajectoryEuler(const double* x0, double horizon) : TrajectoryND()
  {
    double quat[4];
    toInternalCoords(quat, x0);

    addTrajectory1D(new T(quat[0], horizon));
    addTrajectory1D(new T(quat[1], horizon));
    addTrajectory1D(new T(quat[2], horizon));
    addTrajectory1D(new T(quat[3], horizon));
  }

  TrajectoryEuler(const TrajectoryEuler& copyFromMe) : TrajectoryND(copyFromMe)
  {
  }

  TrajectoryEuler* clone() const
  {
    return new TrajectoryEuler(*this);
  }

  virtual ~TrajectoryEuler()
  {
  }

  unsigned int getDim() const
  {
    return 3;
  }

  const char* getClassName() const
  {
    return "TrajectoryEuler";
  }

  void addConstraint(double t,
                     const double* x,
                     const double* x_dot,
                     const double* x_ddot,
                     int flag)
  {
    double quat[4];
    toInternalCoords(quat, x);

    // Here we check the magnitude of the intermediate angle between the
    // previous and current quaternion. If the dot product of the quaternions
    // in negative, the intermediate angle is not the shortest connection of
    // the rotations. In this case, we negate the target quaternion. This will
    // still be the same rotation, however the interpolation will be the
    // shortest one.
    double qPrev[4];
    TrajectoryND::getPositionConstraintBefore(qPrev, t);
    double dotPrd = Quat_dot(quat, qPrev);

    if (dotPrd<0.0)
    {
      VecNd_constMulSelf(quat, -1.0, 4);
    }

    TrajectoryND::addConstraint(t, quat, Vec3d_zeroVec(),
                                Vec3d_zeroVec(), flag);
  }

  void getPositionConstraintBefore(double* x, double t) const
  {
    double quat[4];
    TrajectoryND::getPositionConstraintBefore(quat, t);
    fromInternalCoords(x, quat);
  }

  void toInternalCoords(double* quat, const double* eulerAngles) const
  {
    Quat_fromEulerAngles(quat, eulerAngles);
  }

  bool fromInternalCoords(double* eulerAngles, const double* quat) const
  {
    double normalizedQuat[4];

    VecNd_normalize(normalizedQuat, quat, 4);
    Quat_toEulerAngles(eulerAngles, normalizedQuat);

    return true;
  }

};

}   // namespace tropic



#endif   // TROPIC_TRAJECTORYEULER_H
