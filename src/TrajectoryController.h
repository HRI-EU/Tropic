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

#ifndef TROPIC_TRAJECTORYCONTROLLER_H
#define TROPIC_TRAJECTORYCONTROLLER_H

#include "ConstraintSet.h"

#include <ControllerBase.h>
#include <StackVec.h>


namespace tropic
{

/*! \ingroup Tropic
 *  \brief Class for task-level trajectory generation.
 *
 *         This class is the trajectory-equivalent to the ControllerBase class.
 *         It owns a pointer to a ControllerBase instance. For each task of the
 *         controller, a trajectory is generated. Trajectory constraints can be
 *         added by TrajectoryConstraintSets (and derived). The
 *         \ref addAndApply() methods adds all the given constraints to the
 *         class's set, and links the set's trajectories to the constraints
 *         based on their names. In the step call, all trajectories are
 *         iterated, and the compute() methods of all sets is called.
 */
class TrajectoryControllerBase
{
public:

  /*! \brief Default constructor, initializes all unknown references to NULL.
   */
  TrajectoryControllerBase();

  /*! \brief Deep copy of an instance. All members are cloned. There will be no
   *         dangling references to copyFromMe.
   */
  TrajectoryControllerBase(const TrajectoryControllerBase& copyFromMe);

  /*! \brief Assignment operator, same behavior as copy constructor.
   */
  TrajectoryControllerBase& operator=(const TrajectoryControllerBase& rhs);

  /*! \brief Deletes all dynamic memory of the instance.
   */
  virtual ~TrajectoryControllerBase();

  /*! \brief Clone method, calls copy constructor.
   */
  virtual TrajectoryControllerBase* clone() const;

  /*! \brief Returns "TrajectoryController", to be re-implemented for each
   *         derieved class.
   */
  virtual const char* getClassName() const;

  virtual void populateTasks(double horizon);
  /*! \brief Convenience method to access the dimension of the overall task
   *         vector.
   */
  virtual unsigned int getDim() const;

  /*! \brief Performs a receding horizon trajectory step by calling all
   *         TrajectoryND's step function. This function also handles the
   *         activation of previously deactivated tasks so that the
   *         corresponding trajectories are initialized with the kinematic
   *         state of the system, so that no jumps on any levels are
   *         introduced.
   */
  virtual double step(double dt);

  /*! \brief Computes the trajectory coordinates at time t=0 and copies them
   *         into the vector pos. This gets the values from the now-point,
   *         and does not perform any trajectory point calculation.
   */
  virtual void getPosition(MatNd* pos);

  /*! \brief Computes the trajectory coordinates at time t and copies them into
   *         the vector pos. The caller needs to make sure that the trajectory
   *         at the given time point can be evaluated at all. For instance if
   *         the turbo mode is switched on, there will be no correct result for
   *         querying the position after the first full constraint point.
   */
  virtual void getPosition(double t, MatNd* pos);

  /*! \brief Adds a full constraint with the coordinates given in pos to all
   *         trajectories.
   */
  virtual void addGoalConstraint(double t, const MatNd* pos);

  /*! \brief Computes trajectory only up to the first full constraint.
   */
  virtual void setTurboMode(bool enable);

  /*! \brief Appends the cSet to the rootSet vector.
   */
  virtual void add(std::shared_ptr<ConstraintSet> cSet);

  /*! \brief Appends the cSet to the rootSet vector and applies the constraints
   *         to the trajectories.
   */
  virtual bool addAndApply(std::shared_ptr<ConstraintSet> cSet,
                           bool permissive=false);

  /*! \brief Returns the time of the latest goal constraints for all
   *         trajectories.
   */
  double getTimeOfLastGoal() const;

  /*! \brief Clears all constraints and initializes the trajectories with the
   *         graph's current kinematics.
   */
  virtual void init();

  /*! \brief Clears all trajectory constraints from trajectories and
   *         recursively out of all added TrajectoryConstraintSet
   */
  virtual void clear(bool clearActivations=false);

  /*! \brief Clears all constraint from the rootSet, however lets them
   *         untouched within the trajectories. It means that the
   *         trajectories will further step them, but the set's compute
   *         function will not be called any more.
   */
  void clearConstraintSet();

  /*! \brief Clears all constraint from the rootSet and deletes all
   *         trajectories. The controller remains untouched.
   */
  void eraseTrajectories();

  /*! \brief Calls the print() method of all TrajectoryND members.
   */
  virtual void print() const;

  virtual double getActivation(int idx) const;
  virtual void getActivation(MatNd* activation) const;
  virtual void getContinuousActivation(MatNd* activation) const;
  virtual void setActivation(int idx, bool activity);
  virtual void setActivation(bool activity);
  virtual void setActivation(const MatNd* activation);

  /*! \brief Returns the number of goal and via points,
   *         excluding the now-point and horizon point.
   */
  unsigned int getNumberOfConstraints() const;
  unsigned int getNumberOfSetConstraints() const;
  unsigned int getNumberOfSets() const;

  /*! \brief Returns the smallest blending value by going through all
   *         trajectories.
   */
  double computeBlending() const;
  TrajectoryND* getTrajectory(const std::string& name) const;
  TrajectoryND* getTrajectory(size_t index) const;
  std::vector<TrajectoryND*> getTrajectories() const;
  std::vector<TrajectoryND*>& getTrajectoriesRef();
  const Rcs::Task* getTask(const std::string& name) const;

  /*! \brief Returns a pointer to the controller that the instance is operating
   *         on. This pointer never points to NULL. Either, it returns the
   *         pointer that has been pased in through the constructor, or it
   *         returns the pointer to the internal controller if the first one
   *         does not exist.
   *
   *  \ return Pointer to controller used for kinematics calculations.
   */
  const Rcs::ControllerBase* getController() const;

  /*! \brief Returns a pointer to the controller that the instance "owns". This
   *         is for instance the case for instances that are cloned or copied,
   *         since they do not depend on the controller passed in through the
   *         constructor.
   *
   *  \ return Pointer to internal controller, or NULL if there is none.
   */
  Rcs::ControllerBase* getInternalController();

  /*! \brief Returns a non-modifyable reference to the class's rootSet.
   */
  const ConstraintSet& getRootSet() const;

  /*! \brief Under certain circumstances, re-activating an already moving
   *         trajectory may lead to bendy and weird trajectories due to the
   *         initial constraints. Calling this function with enable being true
   *         leads to initializing the trajectory with only the correct
   *         positional state. The velocities and accelerations are set to
   *         zero. This will lead to a jumpy motion if the velocity and
   *         acceleration is not zero.
   */
  void reactivateWithZeroVelAcc(bool enable);

  /*! \brief Writes the controller's TrajectorySet into a xml file with the
   *         given name.
   *
   *  \param[in] xmlFileName   File name where set is written to.
   *  \return True for success, false otherwise (for instance file cannot be
   *          created).
   */
  bool toXML(const std::string& xmlFileName) const;

  /*! \brief Takes or gives away the ownership of the class's controller. If
   *         the class takes ownership, it means that the class can apply
   *         modifications to the underlying graph, and will delete the
   *         controller in the destructor.
   */
  void takeControllerOwnership(bool ownership);

protected:

  virtual void stepTrajectoryND(TrajectoryND* tnd, const Rcs::Task* task,
                                double dt);

  const Rcs::ControllerBase* controller;
  Rcs::ControllerBase* ownController;
  std::vector<TrajectoryND*> trajectory;
  ConstraintSet rootSet;
  double reactivationScaling;
};



template<class T>
class TrajectoryController : public TrajectoryControllerBase
{
public:

  TrajectoryController(Rcs::ControllerBase* controller_, double horizon) :
    TrajectoryControllerBase()
  {
    this->controller = controller_;
    populateTasks(horizon);
  }

  TrajectoryController(const std::string& cfgFile, double horizon) :
    TrajectoryControllerBase()
  {
    this->ownController = new Rcs::ControllerBase(cfgFile);
    this->controller = ownController;
    populateTasks(horizon);
  }


  void populateTasks(double horizon)
  {
    typedef Rcs::StackVec<double, 16> TaskVec;

    for (size_t i=0; i<controller->getNumberOfTasks(); ++i)
    {
      const Rcs::Task* task_i = controller->getTask(i);
      TaskVec x(task_i->getDim());
      task_i->computeX(x);

      if (task_i->getClassName()=="POLAR")
      {
        TrajectoryPolar<T>* ti = new TrajectoryPolar<T>(x, horizon);
        ti->setName(task_i->getName());
        trajectory.push_back(ti);
      }
      else if (task_i->getClassName()=="ABC")
      {
        TrajectoryEuler<T>* ti = new TrajectoryEuler<T>(x, horizon);
        ti->setName(task_i->getName());
        trajectory.push_back(ti);
      }
      else if (task_i->getClassName()=="XYZ")
      {
        TrajectoryPos3D<T>* ti = new TrajectoryPos3D<T>(x, horizon);
        ti->setName(task_i->getName());
        trajectory.push_back(ti);
      }
      else
      {
        TrajectoryPosND<T>* ti = new TrajectoryPosND<T>(x, x.size(), horizon);
        ti->setName(task_i->getName());
        trajectory.push_back(ti);
      }

    }   // for (size_t i=0; i<controller->getNumberOfTasks(); ++i)

  }

  virtual ~TrajectoryController()
  {
  }

};

typedef TrajectoryController<ViaPointTrajectory1D> ViaTrajectoryController;

}   // namespace tropic

#endif   // TROPIC_TRAJECTORYCONTROLLER_H
