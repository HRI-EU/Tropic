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

#ifndef TROPIC_LIFTOBJECTCONSTRAINT_H
#define TROPIC_LIFTOBJECTCONSTRAINT_H

#include "ConstraintSet.h"

#include <ControllerBase.h>



namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for lifting an object from a surface
 *
 */
class LiftObjectConstraint
{
public:

  LiftObjectConstraint(const Rcs::ControllerBase* controller,
                       const std::string& handName,
                       const std::string& objectName,
                       const std::string& surfaceName);

  LiftObjectConstraint(xmlNode* node);

  LiftObjectConstraint(const LiftObjectConstraint& other);

  virtual LiftObjectConstraint* clone() const;

  virtual ~LiftObjectConstraint();

  static std::shared_ptr<tropic::ConstraintSet>
  pourWithTwoHands(const Rcs::ControllerBase* controller,
                   std::string rHand,
                   std::string lHand,
                   std::string bottle,
                   std::string glas,
                   std::string table,
                   std::string bottleTip,
                   std::string glasTip,
                   double t_start, double t_end);

  static std::shared_ptr<tropic::ConstraintSet>
  pourWithOneHand(const Rcs::ControllerBase* controller,
                  std::string hand,
                  std::string bottle,
                  std::string glas,
                  std::string table,
                  std::string bottleTip,
                  std::string glasTip,
                  double t_start, double t_end);

  static std::shared_ptr<tropic::ConstraintSet>
  screwWithTwoHands(const Rcs::ControllerBase* controller,
                    std::string rHand,
                    std::string lHand,
                    std::string bottle,
                    std::string glas,
                    std::string table,
                    std::string bottleTip,
                    std::string glasTip,
                    double t_start, double t_end);

  std::shared_ptr<tropic::ConstraintSet> lift(double t_start,
                                              double t_grasp,
                                              double t_end,
                                              double liftHeight,
                                              double graspHeight) const;

  std::shared_ptr<tropic::ConstraintSet> tilt(const Rcs::ControllerBase* cntrl,
                                              std::string objToPourFrom,
                                              std::string objToPourInto,
                                              double t_start,
                                              double t_pour,
                                              double t_end,
                                              bool bimanual) const;

  std::shared_ptr<tropic::ConstraintSet> put(double t_start,
                                             double t_put,
                                             double t_release,
                                             const HTr* putPose=NULL) const;

  std::shared_ptr<tropic::ConstraintSet> screw(const Rcs::ControllerBase* cntrl,
                                               std::string objToPourFrom,
                                               std::string objToPourInto,
                                               double t_start,
                                               double t_end) const;

protected:

  virtual void fromXML(xmlNode* node);

private:

  static std::string getEffectorName(const Rcs::Task* tsk);

  static std::string getRefBodyName(const Rcs::Task* tsk);

  static bool getLiftPutTasks(const Rcs::ControllerBase* controller,
                              std::string& hand,
                              std::string& object,
                              std::string& surface,
                              std::string& tskObjHandPos,
                              std::string& tskHandObjPolar,
                              std::string& tskObjSurfacePosX,
                              std::string& tskObjSurfacePosY,
                              std::string& tskObjSurfacePosZ,
                              std::string& tskObjPolar,
                              std::string& tskFingers);

  static HTr getGraspFrame(const Rcs::ControllerBase* controller,
                           std::string object);

  std::string taskObjHandPos;
  std::string taskObjSurfacePosZ;
  std::string taskObjSurfacePosX;
  std::string taskObjSurfacePosY;
  std::string taskObjPolar;
  std::string taskHandObjPolar;
  std::string taskFingers;

  std::string handName;
  std::string objectName;
  std::string surfaceName;

  HTr A_OT; // transformation of the object with respect to the support surface
};

}   // namespace tropic


#endif   // TROPIC_LIFTOBJECTCONSTRAINT_H
