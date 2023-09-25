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

#ifndef TROPIC_CONSTRAINTSET_H
#define TROPIC_CONSTRAINTSET_H

#include "TrajectoryND.h"

#include <libxml/tree.h>

#include <map>
#include <iosfwd>



namespace tropic
{
/*! \ingroup Tropic
 *  \brief Base class for collections of constraints over a set of trajectories
 *
 *         The ConstraintSet class is a recursive class that contains
 *         a vector of trajectory constraints, and possibly other child sets. It
 *         follows the "Composite Design Pattern" by owning a vector of itself.
 *         It allows to organize constraints that span a number of different
 *         trajectories, and is therefore a class that can model "meaningful"
 *         movements. It is for instance possible to create a "reach and grasp"
 *         motion by combining constraints for arm and finger movements.
 *
 *         The ConstraintSet class is modelled to be independent of
 *         any trajectory object at the time of construction. It stores names
 *         of trajectories during construction and configuration that are
 *         resolved when actually being applied to the trajectories. This makes
 *         this class copyable and serializeable without knowing the concrete
 *         trajectory objects that they are applied on.
 *
 *         Applying the class to a set of trajectories copies all of its
 *         constraints to the trajectories. These are responsible for stepping
 *         the time. The constraints are modelled as shared_ptrs so that they
 *         are not owned by any class. The trajectory classes relese their
 *         constraints for instance when their time is below zero. This class
 *         erases the constraints when they are not referenced by any other
 *         class.
 *
 *         The class also has a compute() method. It exists to perform
 *         calculations that cannot be done on the level of individual
 *         1-dimensional trjectories, but require a larger context. One example
 *         can be found in the EulerConstraint, where internally quaternions
 *         are updated to be compliant with the shortest path between two
 *         consecutive rotation constraints. The compute() method also deals
 *         with deletion of constraints, see its documentation for details.
 */

class ConstraintSet
{
public:

  /*! \brief Constructs a set with no children and no constraints.
   */
  ConstraintSet();

  /*! \brief Copy constructor (recursive)
   */
  ConstraintSet(const ConstraintSet& other);

  /*! \brief Construct from xml
   */
  ConstraintSet(xmlNode* node);

  /*! \brief We apply the clone idiom to realize polymorphism. Unfortunately,
   *         this doesn't work using copy constructors.
   */
  virtual ConstraintSet* clone() const;

  /*! \brief Recursive comparison. This includes all members, including the
   *         constraint ids. This function is used during the recursion.
   */
  virtual bool isEqual(std::shared_ptr<ConstraintSet> other) const;

  /*! \brief Recursive comparison. This includes all members, including the
   *         constraint ids.
   */
  virtual bool isEqual(const ConstraintSet& other) const;

  /*! \brief Is the same operator.
   */
  bool operator == (const ConstraintSet& other) const;

  /*! \brief Is the same operator, used during recursion.
   */
  bool operator == (const std::shared_ptr<ConstraintSet> other) const;

  /*! \brief Is the same operator.
   */
  bool operator != (const ConstraintSet& other) const;

  /*! \brief Is the same operator, used during recursion.
   */
  bool operator != (const std::shared_ptr<ConstraintSet> other) const;

  /*! \brief Virtual destructor to enable Polymorphism.
   */
  virtual ~ConstraintSet();

  /*! \brief Returns the number of constraints of this instance, and
   *         additionally of all of its children if recursive is true.
   */
  size_t numConstraints(bool recursive=true) const;

  /*! \brief Returns the number of sets of this instance, and
   *         additionally of all of its children if recursive is true
   */
  size_t numSets(bool recursive=true) const;

  /*! \brief Recursive function that calls each set's compute function. This
   *         class's compute function determines the time point of the last
   *         constraint, and deletes obsolete sets. A set is considered obsolete
   *         if it has no child sets, and if the use_count of all its
   *         constraints is one. This means that there is no other object that
   *         refers to the shared_ptr of the constraint.
   *
   *         Warning: This has a few implications you need to be aware of. For
   *         instance if you are constructing and populating a set without
   *         applying it to trajectories, then the use_count of each constraint
   *         is one. The call to compute() will therefore delete also sets with
   *         constraints that have future time points. This can be an issue if
   *         you call compute() before inspecting or writing a set to file.
   */
  virtual double compute(double dt);

  /*! \brief Returns the earliest time found in the set's constraints
   *         (including its children)
   */
  virtual double getStartTime() const;

  /*! \brief Returns the latest time found in the set's constraints
   *         (including its children)
   */
  virtual double getEndTime() const;

  /*! \brief Returns getEndTime() - getStartTime()
   */
  virtual double getDuration() const;

  /*! \brief Returns a reference to the idx-th constraint in the set, or nullptr
   *         if idx is out of range. In the latter case, a warning is displayed
   *         on debug level 1.
   */
  virtual std::shared_ptr<Constraint1D> getConstraint(size_t idx) const;

  /*! \brief Returns a reference to the idx-th child of the set, or nullptr
   *         if idx is out of range. In the latter case, a warning is displayed
   *         on debug level 1.
   */
  virtual std::shared_ptr<ConstraintSet> getSet(size_t idx) const;

  /*! \brief Returns the name of the idx-th trajectory in the set, or an empty
   *         string if idx is out of range or no trajectory has been given to
   *         the set. In the first case (index out of range), a warning is
   *         displayed on debug level 1.
   */
  virtual std::string getTrajectoryName(size_t idx) const;

  /*! \brief Recursively traverses all sets and adds all constraints to the
   *         corresponsing trajectory. Other than in apply(), the mapping
   *         between constraints and trajectory is done by name, and the
   *         trajectories are searched in the passed vector of trajectories.
   *         This allows the set to be created independent of any trajectory
   *         class, and to be passed / copied etc.
   */
  virtual void apply(std::vector<TrajectoryND*>& trajectory,
                     bool permissive=false);

  /*! \brief Recursively traverses all sets and adds all constraints to the
   *         corresponsing trajectory. The mapping between constraints and
   *         trajectory is done by name, and the trajectories are searched
   *         in the passed name - trajectory map. This function is called from
   *         apply(std::vector<TrajectoryND*>& trajectory).
   */
  virtual void apply(std::vector<TrajectoryND*>& trajectory,
                     std::map<std::string, Trajectory1D*>& tMap,
                     bool permissive);

  /*! \brief Recursively traverses all sets and adds removes all constraints.
   *         If the argument clearTrajectories is true, also all constraints
   *         will be erased from the trajectories associated with the set.
   */
  virtual void clear();

  /*! \brief Adds a child set to this instance.
   */
  virtual void add(std::shared_ptr<ConstraintSet> subSet);

  /*! \brief Adds a full constraint (position, velocity and acceleration level)
   *         to the set. Velocity and acceleration are zero. With the
   *         apply(std::vector<TrajectoryND*> trajectory) function, the
   *         constraint will be added to the the trajectory with the name
   *         trajName.
   */
  virtual void add(double t, double x, const std::string& trajName);

  /*! \brief Adds a constraint to the set. Position, velocity and acceleration
   *         are given as arguments. With the
   *         apply(std::vector<TrajectoryND*> trajectory) function, the
   *         constraint will be added to the the trajectory with the name
   *         trajName.
   */
  virtual void add(double t, double x, double x_dot, double x_ddot, int flag,
                   const std::string& trajName);

  /*! \brief Recursively shifts the time of all constraints by dt.
   */
  virtual void shiftTime(double dt);

  /*! \brief Computes the gradient grad = t^T B^-1, dimension is 1 x nVia .
   *         \todo: Is this correct?
   */
  virtual bool dxdPos(MatNd* grad,
                      const std::shared_ptr<Constraint1D> constraint,
                      double t0, double t1, double dt) const;

  /*! \brief Returns a class name for this class. Each derieved class should
   *         implement this function with a unique name (It's not fatal if it
   *         doesn't). This class's name is ConstraintSet.
   */
  virtual std::string getClassName() const;

  /*! \brief Sets the class name.
   */
  virtual void setClassName(const std::string& name);

  /*! \brief Prints out information to stdout for this set and all its children.
   */
  virtual void print() const;

  /*! \brief True if all constraints have a use_count of 1, false otherwise.
   *         This means that in the case of true, no other class refers to
   *         any of the constraints (through the shared_ptr), and it's safe to
   *         get rid of them.
   */
  virtual bool inUse() const;

  /*! \brief Returns a string containing the c++ demangled class type.
   */
  virtual std::string getTypeName() const;

  /*! \brief Initializes the class from an XML node.
   */
  virtual void fromXML(xmlNode* node);

  /*! \brief Writes the class's xml descrition into the ostream.
   */
  virtual void toXML(std::ostream& out, size_t indent = 0) const;

  /*! \brief Writes the class's xml descrition into a file.
   */
  virtual bool toXML(std::string fileName) const;

  /*! \brief Writes the instance to an xml file, reads it in, writes it out
   *         to another one and does a character-wise comparison of the two
   *         files. The function returns true if the two files are identical,
   *         false otherwise. The file names are ConstraintSet1.xml
   *         and ConstraintSet2.xml, and are written to the directory
   *         the program is started from. They are not deleted automatically,
   *         so that they can be inspected manually if desired.
   */
  virtual bool testXML() const;

  /*! \brief Returns a string with the ConstraintSet's constraint ids:
   *         >id="186 187 188" <
   */
  std::string getIdsForXML() const;

  /*! \brief Collects all sets of the given template type and appends them to
     *         the vector setsOfType. No hierarchies are considered in the
     *         augmented vector, it is just a "linear" collection of the set
     *         hierarchy.
     */
  template <typename T>
  void findSetsOfType(std::vector<T*>& setsOfType)
  {
    T* self = dynamic_cast<T*>(this);
    if (self)
    {
      setsOfType.push_back(self);
    }

    for (size_t i=0; i< children.size(); ++i)
    {
      children[i]->findSetsOfType<T>(setsOfType);
    }

  }

protected:

  /*! \brief The Constraint1D struct combines a trajectory constraint with the
   *         name of the trajectory it is intended to be applied. Since the
   *         trajectory name has no meaning in the constraint itself, we keep
   *         it here and hide this not so beautiful struct as good as possible
   *         from the public interface.
   */
  struct NamedConstraint1D
  {
    NamedConstraint1D()
    {
    }

    NamedConstraint1D(std::shared_ptr<Constraint1D> constraint,
                      const std::string& trajName) : c(constraint),
      trajName1D(trajName)
    {
    }

    NamedConstraint1D(const NamedConstraint1D& other) :
      c(std::shared_ptr<Constraint1D>(other.c->clone())),
      trajName1D(other.trajName1D)
    {
    }

    bool operator == (const NamedConstraint1D& other) const
    {
      if (trajName1D != other.trajName1D)
      {
        return false;
      }

      return *(c.get()) == *(other.c.get());
    }

    std::shared_ptr<Constraint1D> c;
    std::string trajName1D;
  };

  virtual void add(NamedConstraint1D constraint);
  virtual double getStartTimeRecurse() const;

  std::vector<std::shared_ptr<ConstraintSet>> children;
  std::string className;
  std::vector<NamedConstraint1D> constraint;
};

typedef std::shared_ptr<ConstraintSet> TCS_sptr;

}   // namespace tropic



#endif   // TROPIC_CONSTRAINTSET_H
