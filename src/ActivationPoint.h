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

#ifndef RCS_ACTIVATIONPOINT_H
#define  RCS_ACTIVATIONPOINT_H



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Constraint class for organizing task activations
 *
 *         The ActivationPoint class represents a time point that can be added
 *         to TrajectoryND classes. It contains a time field that is decremented
 *         with each step() call of the trajectory it is added to. Once it
 *         traverses the time 0, it will activate (switchOn=true) or deactivate
 *         (switchOn=false) the activation state of the trajectory. This will
 *         for instance be used in the TrajectoryController class, where the
 *         state of the activation point is mapped to a task's activation. The
 *         horizon field determines a time horizon that is used to blend the
 *         activation once the time gets closer to the activation point. See
 *         method \ref computeBlending() for more details.
 */
class ActivationPoint
{
public:
  ActivationPoint(double time, bool switchOn, double horizon);

  /*! \brief Just exists to enable derieving from this class.
   */
  virtual ~ActivationPoint();

  /*! \brief True if all member variables t, switchOn and horizon are equal.
   */
  bool operator == (const ActivationPoint& other) const;

  /*! \brief Convenience function returning a pointer to a newly created
   *         instance.
  */
  ActivationPoint* clone() const;

  /*! \brief Returns the time of the activation point.
   */
  double getTime() const;

  /*! \brief Returns true if the activation point activates a task,
   *         false otherwise.
   */
  bool switchesOn() const;

  /*! \brief Returns the time blending horizon.
   */
  double getHorizon() const;

  /*! \brief Adds the given dt to the class's time.
   */
  void shiftTime(double dt);

  /*! \brief Computes a blending value that is 0 at the time of the activation
   *         point, and 1 when being farther away than the given horizon. In
   *         between, it is linearly blended.
  */
  double computeBlending() const;

  /*! \brief Console output on debug level 0.
   */
  void print() const;

  /*! \brief Sets the switchOn member to the given value.
   */
  void setSwitchOn(bool switchOn);

  /*! \brief Sets the horizon member to the given value. No checking is
   *         performed. The horizon should be >=0, otherwise the behavior
   *         is undefined.
   */
  void setHorizon(double horizon);

private:
  double t;
  bool switchOn;
  double horizon;
};

}

#endif   // RCS_ACTIVATIONPOINT_H
