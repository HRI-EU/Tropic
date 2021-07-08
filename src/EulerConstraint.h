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

#ifndef TROPIC_EULERCONSTRAINT_H
#define TROPIC_EULERCONSTRAINT_H

#include "ConstraintSet.h"



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for computing 3d orientation trajectories
 *
 *         The EulerConstraint class is a 4-dimensinal constraint to create
 *         trajectories between 3d rotations. Internally, it maintains four
 *         constraints that represent a unit quaternion. Interpolation between
 *         two rotations is performed with a linear interpolation (LERP) of
 *         the quaternion components. To extract the interpolated rotations,
 *         the LERPed values are nomalized to a unit quaternion and converted
 *         to the x-y-z- Euler angles (rotating axes version). Therefore the
 *         interface to the class is the only place where Euler angles are used.
 *
 *         You now might think "why the heck does this class do a LERP and not
 *         a SLERP". It is commonly aggrred that a SLERP is the better way to
 *         perform SO(3) interpolation. However, it introduces additional
 *         state, such as maintaining a progress variable between start and
 *         goal rotation. In this library, we want to be able to change the
 *         constraints at any point in time. Therefore we took the design
 *         decision to not depend on the start state, but only on the terminal
 *         rotation. This is why LERP was the method of choice.
 *
 *         The above line of argument is a bit weak, since this class lacks
 *         a few things. Firstly, it is one of very few constraint classes that
 *         stores a reference to the trajectory. It is unfortunately required
 *         to allow adding arbitrary constraints at arbitrary times. This
 *         implies that consecutive quaternions need to be checked if they
 *         are still the shortest path between two rotations, or if they need
 *         to be flipped.
 */
class EulerConstraint : public ConstraintSet
{
public:

  EulerConstraint();

  EulerConstraint(xmlNode* node);

  EulerConstraint(double t, const double I_eulerXYZ[3],
                  const std::string& trajNameND);

  EulerConstraint(double t, double thx, double thy, double thz,
                  const std::string& trajNameND);

  EulerConstraint(double t, const HTr* A_BI_, const double I_eulerXYZ[3],
                  const std::string& trajNameND);

  EulerConstraint(const EulerConstraint& other);

  virtual EulerConstraint* clone() const;

  virtual ~EulerConstraint();

  /*! \brief Here we assume that the apply() function adds the constraints in
   *         increasingtime. If not, we might miss to flip a quaternion
   *         according to an earlier constraint that is added later in the
   *         apply() recursion. This can be a problem.
   */
  virtual void apply(std::vector<TrajectoryND*>& trajectory,
                     std::map<std::string, Trajectory1D*>& tMap,
                     bool permissive) override;

  /*! \brief Checks if the quaternion constraint is the shortest path one with
   *         respect to the previous quaternion. In case it is not, the
   *         constraint quaternion is flipped.
   */
  double compute(double dt);

  void getQuaternion(double quat[4]) const;

  std::string getTrajectoryName() const;

protected:

  void getPreviousQuaternion(double quat[4]) const;
  void setQuaternion(const double quat[4]);
  bool makeShortestPath(double qCurr[4]);
  void getWorldQuaternion(double quat[4]);
  virtual void fromXML(xmlNode* node);
  virtual void toXML(std::ostream& out, size_t indent = 0) const;

  const HTr* A_BI;
  double A_PB[3][3];   // Relative rotation from (B)ody to (P)revious
  TrajectoryND* oriTrj;
  std::string oriTrjName;
};


}   // namespace tropic





#endif   // TROPIC_EULERCONSTRAINT_H
