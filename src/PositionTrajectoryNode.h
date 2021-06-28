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

#ifndef TROPIC_POSITIONTRAJECTORYNODE_H
#define TROPIC_POSITIONTRAJECTORYNODE_H

#include <TrajectoryND.h>
#include <CapsuleNode.h>
#include <VertexArrayNode.h>
#include <Rcs_macros.h>


namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for visualizing 3d position trajectories in OpenSceneGraph.
 *
 *         The class displays the trajectory as a line sequence connecting the
 *         trajectory points with a time spacing dt passed to the constructor.
 *         The class dynamically updates the trajectories. A usage example can
 *         be found in the TestTrajectory.cpp example.
 *
 *         This class exists as header-onny implementation so that the library
 *         can be built without dependencies to osg. Only the application
 *         program that instantiates the viewer needs to be build against the
 *         RcsGraphics library.
 */
class PositionTrajectoryNode: public Rcs::VertexArrayNode
{
public:

  PositionTrajectoryNode(TrajectoryND* traj_) :
    Rcs::VertexArrayNode(osg::PrimitiveSet::LINE_STRIP, "RED"),
    trajPos3D(traj_), dt(0.01), goalSphereRadius(0.03), traj(NULL)
  {
    RCHECK_MSG(trajPos3D->getDim() >= 3, "Dimension must be >=3, but is %d",
               trajPos3D->getDim());

    osg::StateSet* sSet = getOrCreateStateSet();
    sSet->setRenderingHint(osg::StateSet::OPAQUE_BIN);

    this->traj = MatNd_create(1, 3);

    this->currNd = new Rcs::CapsuleNode(NULL, NULL, 0.5*goalSphereRadius, 0.0);
    currNd->setWireframe(false);
    currNd->hide();
    currNd->setMaterial("GREEN");
    addChild(currNd.get());
  }

  ~PositionTrajectoryNode()
  {
    MatNd_destroy(this->traj);
  }

  bool frameCallback()
  {
    double t0 = 0.0;
    double t1 = trajPos3D->getTimeOfLastGoal();

    if (t1 < 0.0)
    {
      t1 = 0.0;
    }

    updateCurrent();
    updateTrajectory(t0, t1);
    updateGoalPoints(t0, t1);
    updateIntermediatePoints(t0, t1);

    return false;
  }

  //////////////////////////////////////////////////////////
  // Visualize trajectory
  //////////////////////////////////////////////////////////
  void updateTrajectory(double t0, double t1)
  {
    trajPos3D->computeTrajectory(traj, t0, t1, dt);
    setPoints(traj);
  }

  //////////////////////////////////////////////////////////
  // Visualize state of trajectory at t=0
  //////////////////////////////////////////////////////////
  void updateCurrent()
  {
    double pos[3];
    trajPos3D->getPosition(0.0, pos);
    currNd->setPosition(pos);
    currNd->show();
  }

  //////////////////////////////////////////////////////////
  // Visualize goal points
  //////////////////////////////////////////////////////////
  void updateGoalPoints(double t0, double t1)
  {
    std::map<double, std::vector<const Constraint1D*> > timeConstrMap;
    std::map<double, std::vector<const Constraint1D*> >::iterator cMapIt;

    // Through all goals in the trajectorie's x-component
    Trajectory1D* trajX = trajPos3D->getTrajectory1D(0);
    for (size_t i = 0; i < trajX->getNumberOfGoals(); ++i)
    {
      const Constraint1D* c = trajX->getGoalPtr(i).get();
      const double cTime = c->getTime();

      if ((cTime>=t0) && (cTime<=t1))
      {
        std::vector<const Constraint1D*> cMap(3);
        cMap[0] = c;
        cMap[1] = NULL;
        cMap[2] = NULL;
        timeConstrMap[cTime] = cMap;
      }
    }

    // Through all goals in the trajectorie's y-component
    Trajectory1D* trajY = trajPos3D->getTrajectory1D(1);
    for (size_t i = 0; i < trajY->getNumberOfGoals(); ++i)
    {
      const Constraint1D* c = trajY->getGoalPtr(i).get();
      const double cTime = c->getTime();

      if ((cTime>=t0) && (cTime<=t1))
      {
        cMapIt = timeConstrMap.find(cTime);

        if (cMapIt != timeConstrMap.end())
        {
          std::vector<const Constraint1D*> cMap = cMapIt->second;
          cMap[1] = c;
          timeConstrMap[cTime] = cMap;
        }
        else
        {
          std::vector<const Constraint1D*> cMap(3);
          cMap[0] = NULL;
          cMap[1] = c;
          cMap[2] = NULL;
          timeConstrMap[cTime] = cMap;
        }
      }

    }

    // Through all goals in the trajectorie's z-component
    Trajectory1D* trajZ = trajPos3D->getTrajectory1D(2);
    for (size_t i = 0; i < trajZ->getNumberOfGoals(); ++i)
    {
      const Constraint1D* c = trajZ->getGoalPtr(i).get();
      const double cTime = c->getTime();

      if ((cTime>=t0) && (cTime<=t1))
      {
        cMapIt = timeConstrMap.find(cTime);

        if (cMapIt != timeConstrMap.end())
        {
          std::vector<const Constraint1D*> cMap = cMapIt->second;
          cMap[2] = c;
          timeConstrMap[cTime] = cMap;
        }
        else
        {
          std::vector<const Constraint1D*> cMap(3);
          cMap[0] = NULL;
          cMap[1] = NULL;
          cMap[2] = c;
          timeConstrMap[cTime] = cMap;
        }

      }
    }

    // Goal node vector only grows
    if (timeConstrMap.size() < goalNd.size())
    {
      // Hide all nodes that exceed the size of the map.
      for (size_t i=timeConstrMap.size(); i<goalNd.size(); ++i)
      {
        goalNd[i]->hide();
      }
    }
    else
    {
      // Increase capacity to match all nodes to be displayed
      for (size_t i=goalNd.size(); i<timeConstrMap.size(); ++i)
      {
        Rcs::CapsuleNode* cn = new Rcs::CapsuleNode(NULL, NULL, goalSphereRadius, 0.0);
        cn->setWireframe(false);
        addChild(cn);
        goalNd.push_back(cn);
      }
    }

    // show content:
    size_t goalIdx = 0;
    for (cMapIt=timeConstrMap.begin(); cMapIt!=timeConstrMap.end(); ++cMapIt)
    {
      double t = cMapIt->first;
      std::vector<const Constraint1D*> cMap = cMapIt->second;
      NLOG(0, "Map[%f] = %s %s %s",
           t, cMap[0] ? "1" : "0", cMap[1] ? "1" : "0", cMap[2] ? "1" : "0");

      double pos[3];
      pos[0] = cMap[0] ? cMap[0]->getPosition() : trajX->getPosition(t);
      pos[1] = cMap[1] ? cMap[1]->getPosition() : trajY->getPosition(t);
      pos[2] = cMap[2] ? cMap[2]->getPosition() : trajZ->getPosition(t);

      if ((cMap[0]!=NULL) && (cMap[1]==NULL) && (cMap[2]==NULL))
      {
        goalNd[goalIdx]->setMaterial("RED");
      }
      else if ((cMap[0]==NULL) && (cMap[1]!=NULL) && (cMap[2]==NULL))
      {
        goalNd[goalIdx]->setMaterial("GREEN");
      }
      else if ((cMap[0]==NULL) && (cMap[1]==NULL) && (cMap[2]!=NULL))
      {
        goalNd[goalIdx]->setMaterial("BLUE");
      }
      else
      {
        goalNd[goalIdx]->setMaterial("YELLOW_TRANS");
      }

      goalNd[goalIdx]->show();
      goalNd[goalIdx]->setPosition(pos);
      goalIdx++;
    }

  }

  //////////////////////////////////////////////////////////
  // Visualize intermediate points
  //////////////////////////////////////////////////////////
  void updateIntermediatePoints(double t0, double t1)
  {
    std::map<double, std::vector<const Constraint1D*> > timeConstrMap;
    std::map<double, std::vector<const Constraint1D*> >::iterator cMapIt;

    // Through all goals in the trajectorie's x-component
    Trajectory1D* trajX = trajPos3D->getTrajectory1D(0);
    for (size_t i = 0; i < trajX->getNumberOfViaPoints(); ++i)
    {
      const Constraint1D* c = trajX->getViaPtr(i).get();
      const double cTime = c->getTime();

      if ((cTime>=t0) && (cTime<=t1))
      {
        std::vector<const Constraint1D*> cMap(3);
        cMap[0] = c;
        cMap[1] = NULL;
        cMap[2] = NULL;
        timeConstrMap[cTime] = cMap;
      }
    }

    // Through all goals in the trajectorie's y-component
    Trajectory1D* trajY = trajPos3D->getTrajectory1D(1);
    for (size_t i = 0; i < trajY->getNumberOfViaPoints(); ++i)
    {
      const Constraint1D* c = trajY->getViaPtr(i).get();
      const double cTime = c->getTime();

      if ((cTime>=t0) && (cTime<=t1))
      {
        cMapIt = timeConstrMap.find(cTime);

        if (cMapIt != timeConstrMap.end())
        {
          std::vector<const Constraint1D*> cMap = cMapIt->second;
          cMap[1] = c;
          timeConstrMap[cTime] = cMap;
        }
        else
        {
          std::vector<const Constraint1D*> cMap(3);
          cMap[0] = NULL;
          cMap[1] = c;
          cMap[2] = NULL;
          timeConstrMap[cTime] = cMap;
        }
      }
    }

    // Through all goals in the trajectorie's z-component
    Trajectory1D* trajZ = trajPos3D->getTrajectory1D(2);
    for (size_t i = 0; i < trajZ->getNumberOfViaPoints(); ++i)
    {
      const Constraint1D* c = trajZ->getViaPtr(i).get();
      const double cTime = c->getTime();

      if ((cTime>=t0) && (cTime<=t1))
      {
        cMapIt = timeConstrMap.find(cTime);

        if (cMapIt != timeConstrMap.end())
        {
          std::vector<const Constraint1D*> cMap = cMapIt->second;
          cMap[2] = c;
          timeConstrMap[cTime] = cMap;
        }
        else
        {
          std::vector<const Constraint1D*> cMap(3);
          cMap[0] = NULL;
          cMap[1] = NULL;
          cMap[2] = c;
          timeConstrMap[cTime] = cMap;
        }

      }
    }

    // Goal node vector only grows
    if (timeConstrMap.size() < viaNd.size())
    {
      // Hide all nodes that exceed the size of the map.
      for (size_t i=timeConstrMap.size(); i<viaNd.size(); ++i)
      {
        viaNd[i]->hide();
      }
    }
    else
    {
      // Increase capacity to match all nodes to be displayed
      for (size_t i=viaNd.size(); i<timeConstrMap.size(); ++i)
      {
        Rcs::CapsuleNode* cn = new Rcs::CapsuleNode(NULL, NULL, 0.5*goalSphereRadius, 0.0);
        cn->setWireframe(false);
        addChild(cn);
        viaNd.push_back(cn);
      }
    }

    // show content:
    size_t idx = 0;
    for (cMapIt=timeConstrMap.begin(); cMapIt!=timeConstrMap.end(); ++cMapIt)
    {
      double t = cMapIt->first;
      std::vector<const Constraint1D*> cMap = cMapIt->second;
      NLOG(0, "via Map[%f] = %s %s %s",
           t, cMap[0] ? "1" : "0", cMap[1] ? "1" : "0", cMap[2] ? "1" : "0");

      double pos[3];
      pos[0] = cMap[0] ? cMap[0]->getPosition() : trajX->getPosition(t);
      pos[1] = cMap[1] ? cMap[1]->getPosition() : trajY->getPosition(t);
      pos[2] = cMap[2] ? cMap[2]->getPosition() : trajZ->getPosition(t);

      if ((cMap[0]!=NULL) && (cMap[1]==NULL) && (cMap[2]==NULL))
      {
        viaNd[idx]->setMaterial("RED");
      }
      else if ((cMap[0]==NULL) && (cMap[1]!=NULL) && (cMap[2]==NULL))
      {
        viaNd[idx]->setMaterial("GREEN");
      }
      else if ((cMap[0]==NULL) && (cMap[1]==NULL) && (cMap[2]!=NULL))
      {
        viaNd[idx]->setMaterial("BLUE");
      }
      else
      {
        viaNd[idx]->setMaterial("YELLOW_TRANS");
      }

      viaNd[idx]->show();
      viaNd[idx]->setPosition(pos);
      idx++;
    }

  }

  TrajectoryND* trajPos3D;
  osg::ref_ptr<Rcs::CapsuleNode> currNd;
  std::vector<Rcs::CapsuleNode*> goalNd, viaNd;
  double dt;
  double goalSphereRadius;
  MatNd* traj;
};

}   // namespace tropic

#endif   // TROPIC_POSITIONTRAJECTORYNODE_H
