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

/*
Composite pattern Beispiel Klasse:

class Person
{
std::vector<Person> children;
bool is18()
{
if (age<18) return false;
else
return true;
}

int age;
}

class Teenager : public Person
{
bool is18()
{
return false;
}
}


*/

#ifndef TROPIC_POURINGCONSTRAINT_H
#define TROPIC_POURINGCONSTRAINT_H

#include "ConstraintSet.h"
#include "PoseConstraint.h"
#include "PolarConstraint.h"
#include "ConnectBodyConstraint.h"
#include "VectorConstraint.h"
#include "EulerConstraint.h"

#include <Rcs_macros.h>

namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for pouring a bottle into a glas
 *
 */
class PouringConstraint : public ConstraintSet
{
public:

  /*
  <ConstraintSet type="ActivationSet" >   a1
  <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="XYZ_R Bottle"  />   a2
  <Activation t="2" switchesOn="false" horizon="0.5" trajectory="XYZ_R Bottle"  />    a3
  <ConstraintSet type="PositionConstraint" t="1.4" pos="0.15 0.0015 -0.15" trajectory="XYZ_R Bottle" />
  <ConstraintSet type="PositionConstraint" t="2" pos="-0.015 -0.00015 -0.11" trajectory="XYZ_R Bottle" />
  </ConstraintSet>

  <ConstraintSet type="ActivationSet" >
  <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="Polar_R"  />
  <ConstraintSet type="PolarConstraint" t="2" pos="1 0" trajectory="Polar_R" />
  </ConstraintSet>
  */

  PouringConstraint() : ConstraintSet()
  {

    /* double duration = 18.0; */
    double pouringDuration = 2.0;


    /*
    graspBottle     xxxxxxxx
    liftBottle                 xxxxxxxxxxx*
    graspGlas               xxxxxxx*
    liftGlas                        *xxxxx*
    Pouring                                *xxxxxxxxxxxxxxxx
    placeBottle                                           xxxxxxxxx
    placeGlas                                             xxxxxxxxxxx
     */


    RLOG(0, "Here is the pouring constraint");

    add(graspBottle(0.0, 6.0));
    //       add(Gazing(0.5, 1.5, "Gaze Bottle"));
    add(graspGlas(0.0, 8.0));
    //       add(Gazing(2.0, 2.5, "Gaze Glas"));
    add(liftBottle(6.0, 4.0, 5.5));
    //       add(Gazing(4.5, 5.0, "Gaze BottleTip"));
    add(liftGlas(8.0, 3.5, 3.5));
    add(Pouring(11.5, pouringDuration));
    add(placeBottle(14.5 + pouringDuration, 6.5));
    add(placeGlas(14.5 + pouringDuration, 7.0));
    //       add(Gazing(9.5 + pouringDuration, 2.5, "Gaze Glas"));
    //add(placeGlas(2.0*duration/3.0, duration/3.0));


    /*
          add(graspBottle(0.0, 2.0));
          add(openBottle(2.0, 4.5));
          add(liftBottle(7.0, 1.0, 3.0));
          add(graspGlas(7.0, 2.0));
          add(liftGlas(9.0, 1.5, 1.5));
          add(Pouring(10.5, pouringDuration));
          add(placeBottle(14.5 + pouringDuration, 5.0));
          add(placeGlas(14.5 + pouringDuration, 6.0));
    */

    /*std::shared_ptr<tropic::ActivationSet> b1 = std::make_shared<tropic::ActivationSet>();

    std::shared_ptr<tropic::ActivationSet> b2 = std::make_shared<tropic::ActivationSet>();
    b2->addActivation(0.5, true, 0.5, "Polar_R");
    b1->add(b2);

    b1->add(std::make_shared<tropic::PolarConstraint>(2.0, 1.0, 0.0, "Polar_R"));

    add(b1);*/

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="Gaze Bottle"  />
    <Activation t="2" switchesOn="false" horizon="0.5" trajectory="Gaze Bottle"  />
    <ConstraintSet type="VectorConstraint" t="1.0" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze Bottle" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="2" switchesOn="true" horizon="0.5" trajectory="Gaze Glas"  />
    <Activation t="3" switchesOn="false" horizon="0.5" trajectory="Gaze Glas"  />
    <ConstraintSet type="VectorConstraint" t="2.5" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze Glas" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="3" switchesOn="true" horizon="0.5" trajectory="Gaze BottleTip"  />
    <Activation t="10" switchesOn="false" horizon="0.5" trajectory="Gaze BottleTip"  />
    <ConstraintSet type="VectorConstraint" t="3.5" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze BottleTip" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="10" switchesOn="true" horizon="0.5" trajectory="Gaze Glas"  />
    <Activation t="12.5" switchesOn="false" horizon="0.5" trajectory="Gaze Glas"  />
    <ConstraintSet type="VectorConstraint" t="10.5" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze Glas" />
    </ConstraintSet>
    */


  }

  std::shared_ptr<tropic::ConstraintSet> graspBottle(double t_start, double duration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration - 1.0;

    // Hand position with respect to bottle
    a1->addActivation(t_start, true, 0.5, "XYZ_R Bottle");
    a1->addActivation(t_end + 1.0, false, 0.5, "XYZ_R Bottle");

    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 2*duration/3 - 1.0, 0.15, 0.0015, -0.15, "XYZ_R Bottle", 5));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_end, -0.015, -0.00015, -0.11, "XYZ_R Bottle"));

    // Hand orientation
    a1->addActivation(t_start+0*duration/4, true, 0.5, "Polar_R");
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 0.99*duration, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_R"));

    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_end + 1.0, "Bottle", "PowerGrasp_R"));

    //a1->addActivation(t_start+duration/4, true, 0.5, "FingerJoints_R");

    //a1->add(std::make_shared<tropic::VectorConstraint>(t_end - 0.2, std::vector<double>{0.0, 0.0, 0.0}, "FingerJoints_R"));
    //a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double>{0.5, 0.5, 0.5}, "FingerJoints_R"));

    a1->add(Grasp(t_end, "FingerJoints_R"));

    /*
    std::vector<double> myPosition;
    myPosition.push_back(1.0);
    myPosition.push_back(1.0);
    myPosition.push_back(1.0);
    a1->add(std::make_shared<tropic::VectorConstraint>(t_end - 0.2, myPosition, "FingerJoints_R"));
    */

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >   a1
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="XYZ_R Bottle"  />   a2
    <Activation t="2" switchesOn="false" horizon="0.5" trajectory="XYZ_R Bottle"  />    a3
    <ConstraintSet type="PositionConstraint" t="1.4" pos="0.15 0.0015 -0.15" trajectory="XYZ_R Bottle" />
    <ConstraintSet type="PositionConstraint" t="2" pos="-0.015 -0.00015 -0.11" trajectory="XYZ_R Bottle" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="Polar_R"  />
    <ConstraintSet type="PolarConstraint" t="2" pos="1 0" trajectory="Polar_R" />
    </ConstraintSet>

    <ConstraintSet type="ConnectBodyConstraint" t="2" parent="PowerGrasp_R" child="Bottle" />

    <ConstraintSet type="ActivationSet" >
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="FingerJoints_R"  />
    <Activation t="13.5" switchesOn="false" horizon="0.5" trajectory="FingerJoints_R"  />
    <ConstraintSet type="VectorConstraint" t="1.8" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    <ConstraintSet type="VectorConstraint" t="2" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    <ConstraintSet type="VectorConstraint" t="10.2" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    <ConstraintSet type="VectorConstraint" t="10.4" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    </ConstraintSet>
    */

  }

  std::shared_ptr<tropic::ConstraintSet> graspGlas(double t_start, double duration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration - 1.0;

    a1->addActivation(t_start, true, 0.5, "XYZ_L Glas");
    a1->addActivation(t_end + 1.0, false, 0.5, "XYZ_L Glas");

    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.67*duration - 1.0, 0.15, 0.0015, -0.15, "XYZ_L Glas", 5));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_end, -0.009, -0.00009, -0.11, "XYZ_L Glas"));

    a1->addActivation(t_start, true, 0.5, "Polar_L");

    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 0.5*duration, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_L"));

    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_end + 1.0, "Glas", "PowerGrasp_L"));

    a1->add(Grasp(t_end, "FingerJoints_L"));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="2" switchesOn="true" horizon="0.5" trajectory="XYZ_L Glas"  />
    <Activation t="3.5" switchesOn="false" horizon="0.5" trajectory="XYZ_L Glas"  />
    <ConstraintSet type="PositionConstraint" t="2.9" pos="0.15 0.0015 -0.15" trajectory="XYZ_L Glas" />
    <ConstraintSet type="PositionConstraint" t="3.5" pos="-0.009 -0.00009 -0.11" trajectory="XYZ_L Glas" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="2" switchesOn="true" horizon="0.5" trajectory="Polar_L"  />
    <ConstraintSet type="PolarConstraint" t="3.5" pos="1 0" trajectory="Polar_L" />
    </ConstraintSet>

    <ConstraintSet type="ConnectBodyConstraint" t="3.5" parent="PowerGrasp_L" child="Glas" />

    <ConstraintSet type="ActivationSet" >
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="FingerJoints_L"  />
    <Activation t="13.5" switchesOn="false" horizon="0.5" trajectory="FingerJoints_L"  />
    <ConstraintSet type="VectorConstraint" t="3.3" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    <ConstraintSet type="VectorConstraint" t="3.5" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    <ConstraintSet type="VectorConstraint" t="12.3" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    <ConstraintSet type="VectorConstraint" t="12.5" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    </ConstraintSet>
    */

  }


  std::shared_ptr<tropic::ConstraintSet> liftBottle(double t_start, double duration, double activationDuration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration;
    double t_endActivation = t_start + activationDuration;

    a1->addActivation(t_start, true, 0.5, "XYZ_R");
    a1->addActivation(t_endActivation, false, 0.5, "XYZ_R");

    a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.35, -0.2, 1.05, "XYZ_R"));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="2.5" switchesOn="true" horizon="0.5" trajectory="XYZ_R"  />
    <Activation t="5.5" switchesOn="false" horizon="0.5" trajectory="XYZ_R"  />
    <ConstraintSet type="PositionConstraint" t="3.5" pos="0.542 -0.2 1.05" trajectory="XYZ_R" />
    </ConstraintSet>
    */

  }

  std::shared_ptr<tropic::ConstraintSet> liftGlas(double t_start, double duration, double activationDuration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration;
    double t_endActivation = t_start + activationDuration;

    a1->addActivation(t_start, true, 0.5, "XYZ_L");
    a1->addActivation(t_endActivation, false, 0.5, "XYZ_L");

    a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.35, 0.2, 1.05, "XYZ_L"));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="4" switchesOn="true" horizon="0.5" trajectory="XYZ_L"  />
    <Activation t="5.5" switchesOn="false" horizon="0.5" trajectory="XYZ_L"  />
    <ConstraintSet type="PositionConstraint" t="5.5" pos="0.5 0.2 1.05" trajectory="XYZ_L" />
    </ConstraintSet>
    */

  }

  std::shared_ptr<tropic::ConstraintSet> Pouring(double t_start, double duration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    //double t_end = t_start + duration;

    a1->addActivation(t_start, true, 0.5, "XYZ_GlasBottle");
    a1->addActivation(t_start + 3.0 + duration, false, 0.5, "XYZ_GlasBottle");   // 11.5 -> 16.5

    // Keep right hand at its current x-location so that it does not bump into the screen
    a1->addActivation(t_start, true, 0.5, "X_R");
    a1->addActivation(t_start + 3.0 + duration, false, 0.5, "X_R");   // 11.5 -> 16.5


    //a1->add(std::make_shared<tropic::PositionConstraint>(6.5, 0.09, -0.15, 0.01, "XYZ_GlasBottle"));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 2.05, 0, 0, 0.01, "XYZ_GlasBottle"));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 2.5, 0, 0, 0.01, "XYZ_GlasBottle"));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 2.5 + duration, 0, 0, 0.01, "XYZ_GlasBottle"));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 3.0 + duration, 0, 0, 0.01, "XYZ_GlasBottle"));

    a1->addActivation(t_start, true, 0.5, "Polar_R");

    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 1.0, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_R"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 2.5, RCS_DEG2RAD(90.0), RCS_DEG2RAD(90.0), "Polar_R"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 2.5 + duration, RCS_DEG2RAD(110.0), RCS_DEG2RAD(90.0), "Polar_R"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 4.0 + duration, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_R"));

    a1->addActivation(t_start, true, 0.5, "Polar_L");

    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 1.0, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_L"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 2.5, RCS_DEG2RAD(15.0), RCS_DEG2RAD(-90.0), "Polar_L"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 2.5 + duration, RCS_DEG2RAD(5.0), RCS_DEG2RAD(-90.0), "Polar_L"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 4.0 + duration, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_L"));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="5.5" switchesOn="true" horizon="0.5" trajectory="XYZ_GlasBottle"  />
    <Activation t="8.5" switchesOn="false" horizon="0.5" trajectory="XYZ_GlasBottle"  />
    <!--<ConstraintSet type="PositionConstraint" t="6.5" pos="0.09 -0.15 0.01" flag="5" trajectory="XYZ_GlasBottle" />-->
    <ConstraintSet type="PositionConstraint" t="7.55" pos="0 0 0.01" trajectory="XYZ_GlasBottle" />
    <ConstraintSet type="PositionConstraint" t="z8.5" pos="0 0 0.01" trajectory="XYZ_GlasBottle" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <ConstraintSet type="PolarConstraint" t="6.5" pos="0 0" trajectory="Polar_R" />
    <ConstraintSet type="PolarConstraint" t="8" pos="90 90" trajectory="Polar_R" />
    <ConstraintSet type="PolarConstraint" t="9.5" pos="1 0" trajectory="Polar_R" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <ConstraintSet type="PolarConstraint" t="6.5" pos="0 0" trajectory="Polar_L" />
    <ConstraintSet type="PolarConstraint" t="8" pos="15 -90" trajectory="Polar_L" />
    <ConstraintSet type="PolarConstraint" t="9.5" pos="1 0" trajectory="Polar_L" />
    </ConstraintSet>
    */

  }

  std::shared_ptr<tropic::ConstraintSet> placeBottle(double t_start, double duration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration;

    a1->addActivation(t_start, true, 0.5, "XYZ_R Table");
    a1->addActivation(t_start + 0.34*duration, false, 0.5, "XYZ_R Table");   // 16 -> 16.68

    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.3*duration, 0.06, 0.2, -0.16, "XYZ_R Table", 5));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.34*duration, 0.06, 0.2, -0.11, "XYZ_R Table"));

    a1->addActivation(t_start, true, 0.5, "Polar_R");
    a1->addActivation(t_start + 0.67*duration, false, 0.5, "Polar_R");

    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 0.5*duration, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_R"));

    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_start + 0.34*duration, "Bottle", "Table"));

    a1->addActivation(t_start + 0.34*duration, true, 0.5, "XYZ_R Bottle");
    a1->addActivation(t_end, false, 0.5, "XYZ_R Bottle");   // 16 -> 16.68

    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.9*duration, 0.15, 0.0015, -0.11, "XYZ_R Bottle"));

    //a1->addActivation(t_end, false, 0.5, "FingerJoints_R");

    //a1->add(std::make_shared<tropic::VectorConstraint>(t_start + 0.38*duration - 0.2, 0.5, 0.5, 0.5, "FingerJoints_R"));
    //a1->add(std::make_shared<tropic::VectorConstraint>(t_start + 0.38*duration, 0.0, 0.0, 0.0, "FingerJoints_R"));


    //a1->add(std::make_shared<tropic::VectorConstraint>(t_start + 0.38*duration - 0.2, std::vector<double>{0.5, 0.5, 0.5}, "FingerJoints_R"));
    //a1->add(std::make_shared<tropic::VectorConstraint>(t_start + 0.38*duration, std::vector<double>{0.0, 0.0, 0.0}, "FingerJoints_R"));

    a1->add(Unhand(t_start + 0.38*duration, "FingerJoints_R"));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="8.5" switchesOn="true" horizon="0.5" trajectory="XYZ_R Table"  />
    <Activation t="11" switchesOn="false" horizon="0.5" trajectory="XYZ_R Table"  />
    <!--     <ConstraintSet type="PositionConstraint" t="6.5" pos="0.25 0.3778 0.243" trajectory="XYZ_R Table" /> -->
    <ConstraintSet type="PositionConstraint" t="10" pos="0.06 0.2 -0.16" flag="5" trajectory="XYZ_R Table" />
    <ConstraintSet type="PositionConstraint" t="10.2" pos="0.06 0.2 -0.11" trajectory="XYZ_R Table" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="13.5" switchesOn="false" horizon="0.5" trajectory="Polar_R"  />
    <ConstraintSet type="PolarConstraint" t="11" pos="1 0" trajectory="Polar_R" />
    </ConstraintSet>

    <ConstraintSet type="ConnectBodyConstraint" t="10.2" parent="Table" child="Bottle" />

    <ConstraintSet type="ActivationSet" >
    <Activation t="10.2" switchesOn="true" horizon="0.5" trajectory="XYZ_R Bottle"  />
    <Activation t="13" switchesOn="false" horizon="0.5" trajectory="XYZ_R Bottle"  />
    <ConstraintSet type="PositionConstraint" t="13" pos="0.15 0.0015 -0.11" trajectory="XYZ_R Bottle" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="FingerJoints_R"  />
    <Activation t="13.5" switchesOn="false" horizon="0.5" trajectory="FingerJoints_R"  />
    <ConstraintSet type="VectorConstraint" t="1.8" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    <ConstraintSet type="VectorConstraint" t="2" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    <ConstraintSet type="VectorConstraint" t="10.2" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    <ConstraintSet type="VectorConstraint" t="10.4" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_R" />
    </ConstraintSet>

    */

  }

  std::shared_ptr<tropic::ConstraintSet> placeGlas(double t_start, double duration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration;

    a1->addActivation(t_start, true, 0.5, "XYZ_L Table");
    a1->addActivation(t_start + 0.34*duration, false, 0.5, "XYZ_L Table");

    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.34*duration, 0.1, -0.2, -0.11, "XYZ_L Table"));

    a1->addActivation(t_start, true, 0.5, "Polar_L");
    a1->addActivation(t_start + 0.67*duration, false, 0.5, "Polar_L");

    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 0.5*duration, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_L"));

    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_start + 0.34*duration, "Glas", "Table"));

    a1->addActivation(t_start + 0.34*duration, true, 0.5, "XYZ_L Glas");
    a1->addActivation(t_end, false, 0.5, "XYZ_L Glas");

    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.9*duration, 0.15, 0.0015, -0.11, "XYZ_L Glas"));

    a1->add(Unhand(t_start + 0.38*duration, "FingerJoints_L"));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="8.5" switchesOn="true" horizon="0.5" trajectory="XYZ_L Table"  />
    <Activation t="12.5" switchesOn="false" horizon="0.5" trajectory="XYZ_L Table"  />
    <ConstraintSet type="PositionConstraint" t="12.5" pos="0.1 -0.2 -0.11" trajectory="XYZ_L Table" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="8.5" switchesOn="true" horizon="0.5" trajectory="Polar_L"  />
    <Activation t="12.5" switchesOn="false" horizon="0.5" trajectory="Polar_L"  />
    <ConstraintSet type="PolarConstraint" t="12.5" pos="1 0" trajectory="Polar_L" />
    </ConstraintSet>

    <ConstraintSet type="ConnectBodyConstraint" t="12.5" parent="Table" child="Glas" />

    <ConstraintSet type="ActivationSet" >
    <Activation t="12.5" switchesOn="true" horizon="0.5" trajectory="XYZ_L Glas"  />
    <Activation t="14.5" switchesOn="false" horizon="0.5" trajectory="XYZ_L Glas"  />
    <ConstraintSet type="PositionConstraint" t="14.5" pos="0.15 0.0015 -0.11" trajectory="XYZ_L Glas" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="FingerJoints_L"  />
    <Activation t="13.5" switchesOn="false" horizon="0.5" trajectory="FingerJoints_L"  />
    <ConstraintSet type="VectorConstraint" t="3.3" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    <ConstraintSet type="VectorConstraint" t="3.5" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    <ConstraintSet type="VectorConstraint" t="12.3" pos="0.5 0.5 0.5" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    <ConstraintSet type="VectorConstraint" t="12.5" pos="0 0 0" vel="0 0 0" acc="0 0 0" trajectory="FingerJoints_L" />
    </ConstraintSet>
    */

  }

  std::shared_ptr<tropic::ConstraintSet> Grasp(double t_start, std::string Hand) const /////////////////////////////////////////////////////
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    a1->addActivation(t_start, true, 0.5, Hand);

    a1->add(std::make_shared<tropic::VectorConstraint>(t_start, std::vector<double> {0.01, 0.01, 0.01}, Hand));
    a1->add(std::make_shared<tropic::VectorConstraint>(t_start + 1.0, std::vector<double> {0.5, 0.5, 0.5}, Hand));

    return a1;

    /*

    */

  }

  std::shared_ptr<tropic::ConstraintSet> Unhand(double t_start, std::string Hand) const /////////////////////////////////////////////////////
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    //a1->addActivation(t_start, false, 0.5, Hand);

    a1->add(std::make_shared<tropic::VectorConstraint>(t_start, std::vector<double> {0.5, 0.5, 0.5}, Hand));
    a1->add(std::make_shared<tropic::VectorConstraint>(t_start + 1.5, std::vector<double> {0.01, 0.01, 0.01}, Hand));

    return a1;

    /*

    */

  }

  std::shared_ptr<tropic::ConstraintSet> Gazing(double t_start, double duration, std::string View) const /////////////////////////////////////////////////////
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration;

    a1->addActivation(t_start, true, 0.5, View);
    a1->addActivation(t_end, false, 0.5, View);

    a1->add(std::make_shared<tropic::VectorConstraint>(t_start + 0.5, std::vector<double> {0.0, 0.0}, View));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >
    <Activation t="0.5" switchesOn="true" horizon="0.5" trajectory="Gaze Bottle"  />
    <Activation t="2" switchesOn="false" horizon="0.5" trajectory="Gaze Bottle"  />
    <ConstraintSet type="VectorConstraint" t="1.0" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze Bottle" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="2" switchesOn="true" horizon="0.5" trajectory="Gaze Glas"  />
    <Activation t="3" switchesOn="false" horizon="0.5" trajectory="Gaze Glas"  />
    <ConstraintSet type="VectorConstraint" t="2.5" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze Glas" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="3" switchesOn="true" horizon="0.5" trajectory="Gaze BottleTip"  />
    <Activation t="10" switchesOn="false" horizon="0.5" trajectory="Gaze BottleTip"  />
    <ConstraintSet type="VectorConstraint" t="3.5" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze BottleTip" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="10" switchesOn="true" horizon="0.5" trajectory="Gaze Glas"  />
    <Activation t="12.5" switchesOn="false" horizon="0.5" trajectory="Gaze Glas"  />
    <ConstraintSet type="VectorConstraint" t="10.5" pos="0 0" vel="0 0" acc="0 0" trajectory="Gaze Glas" />
    </ConstraintSet>
    */

  }

  std::shared_ptr<tropic::ConstraintSet> openBottle(double t_start, double duration) const
  {
    std::shared_ptr<tropic::ActivationSet> a1 = std::make_shared<tropic::ActivationSet>();

    double t_end = t_start + duration;

    a1->addActivation(t_start, true, 0.5, "Polar_R");
    a1->addActivation(t_end, false, 0.5, "Polar_R");

    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 1*duration/3, RCS_DEG2RAD(45.0), RCS_DEG2RAD(90.0), "Polar_R"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 2*duration/3, RCS_DEG2RAD(45.0), RCS_DEG2RAD(90.0), "Polar_R"));
    a1->add(std::make_shared<tropic::PolarConstraint>(t_end, RCS_DEG2RAD(0.0), RCS_DEG2RAD(0.0), "Polar_R"));

    a1->addActivation(t_start, true, 0.5, "XYZ_L BottleTip");
    a1->addActivation(t_end, false, 0.5, "XYZ_L BottleTip");

    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 1*duration/3, 0.0, 0.0, 0.0, "XYZ_L BottleTip"));

    a1->addActivation(t_start, true, 0.5, "Euler_L BottleTip");
    a1->addActivation(t_end, false, 0.5, "Euler_L BottleTip");




    a1->add(std::make_shared<tropic::EulerConstraint>(t_start + 1*duration/3, RCS_DEG2RAD(0.0), RCS_DEG2RAD(-90.0), RCS_DEG2RAD(100.0), "Euler_L BottleTip"));
    a1->add(std::make_shared<tropic::EulerConstraint>(t_start + 4*duration/9, RCS_DEG2RAD(0.0), RCS_DEG2RAD(-90.0), RCS_DEG2RAD(-60.0), "Euler_L BottleTip"));
    a1->add(std::make_shared<tropic::EulerConstraint>(t_start + 5*duration/9, RCS_DEG2RAD(0.0), RCS_DEG2RAD(-90.0), RCS_DEG2RAD(100.0), "Euler_L BottleTip"));
    a1->add(std::make_shared<tropic::EulerConstraint>(t_start + 2*duration/3, RCS_DEG2RAD(0.0), RCS_DEG2RAD(-90.0), RCS_DEG2RAD(-60.0), "Euler_L BottleTip"));
    a1->add(std::make_shared<tropic::EulerConstraint>(t_start + 7*duration/9, RCS_DEG2RAD(0.0), RCS_DEG2RAD(-90.0), RCS_DEG2RAD(0.0), "Euler_L BottleTip"));

    a1->add(Unhand(t_start + 2.0*duration/3.0 - 0.2, "FingerJoints_L"));

    return a1;

    /*
    <ConstraintSet type="ActivationSet" >

    <ConstraintSet type="PolarConstraint" t="3.5" pos="45 90" trajectory="Polar_R" />
    <ConstraintSet type="PolarConstraint" t="5" pos="45 90" trajectory="Polar_R" />
    <ConstraintSet type="PolarConstraint" t="6.5" pos="1 0" trajectory="Polar_R" />
    </ConstraintSet>

    <ConstraintSet type="ActivationSet" >
    <Activation t="2" switchesOn="true" horizon="0.5" trajectory="XYZ_L BottleTip"  />
    <Activation t="5.5" switchesOn="false" horizon="0.5" trajectory="XYZ_L BottleTip"  />
    <ConstraintSet type="PositionConstraint" t="3.5" pos="0 0 0" trajectory="XYZ_L BottleTip" />
    </ConstraintSet>


    <ConstraintSet type="ActivationSet" >
    <Activation t="2" switchesOn="true" horizon="0.5" trajectory="Euler_L BottleTip"  />
    <Activation t="5.5" switchesOn="false" horizon="0.5" trajectory="Euler_L BottleTip"  />
    <ConstraintSet type="EulerConstraint" t="3.5" pos="0 -90 100" trajectory="Euler_L BottleTip" />
    <ConstraintSet type="EulerConstraint" t="4" pos="0 -90 -60" trajectory="Euler_L BottleTip" />
    <ConstraintSet type="EulerConstraint" t="4.5" pos="0 -90 100" trajectory="Euler_L BottleTip" />
    <ConstraintSet type="EulerConstraint" t="5" pos="0 -90 -60" trajectory="Euler_L BottleTip" />
    <ConstraintSet type="EulerConstraint" t="5.5" pos="0 -90 0" trajectory="Euler_L BottleTip" />
    </ConstraintSet>
    */

  }




  PouringConstraint(xmlNode* node)
  {
  }

  PouringConstraint(double t, const double I_eulerXYZ[3],
                    const std::string& trajNameND)
  {
  }

  PouringConstraint(const PouringConstraint& other)
  {
  }

  virtual PouringConstraint* clone() const
  {
    return nullptr;
  }

  virtual ~PouringConstraint()
  {
  }

protected:

  virtual void fromXML(xmlNode* node)
  {
  }

  virtual void toXML(std::ostream& out, size_t indent = 0) const
  {
  }

};


}   // namespace tropic





#endif   // TROPIC_POURINGCONSTRAINT_H
