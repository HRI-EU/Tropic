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

#ifndef TROPIC_CONSTRAINT1D_H
#define TROPIC_CONSTRAINT1D_H

#include <vector>
#include <ostream>
#include <memory>



namespace tropic
{

/*! \ingroup Tropic
 *  \brief 1-dimensional trajectory constraint
 *
 *         The Constraint1D class represents a 1-dimensional
 *         constraint that can be added to a Trajectory1D class or any of its
 *         children. The constraint is very simple. It has a time field that
 *         gets incremented on each step() call of the trajectory it is added
 *         to. The flag is a bit mask that determines to which levels the
 *         constraint is applied. This is what the flags mean:
 *
 *         0 nothing
 *         1 pos
 *         2 vel
 *         3 pos + vel
 *         4 acc
 *         5 pos + acc
 *         6 vel + acc
 *         7 pos + vel + acc
 *
 *         The fields x, x_dot, x_ddot mean the constraint's position, velocity
 *         and acceleration respectively. The class has some structs that help
 *         sorting constraints. This is needed on the trajectory level, where
 *         constraints are sorted over time for consistency. More details on the
 *         constraints and the underlying math can be found in the latex
 *         documentation, and in the implementation of the ViaPointSequence
 *         class of Rcs, which contains the underlying math for the polynomial
 *         trajectory generation.
 */
class Constraint1D
{
public:

  /*! \brief Constructs a constraint with flag, time and values zero. Also does
   *         some statistics (decreases global constraint count).
   */
  Constraint1D();

  /*! \brief Constructs a constraint with the given flag, time and values. Also
   *         does some statistics (decreases global constraint count).
   */
  Constraint1D(double t, double x, double x_dot, double x_ddot, int flag);

  /*! \brief Constructs a constraint with the given time and position. The
   *         velocity and acceleration are set to 0. All constraints are set
   *         to active (flag 7). This is what the API refers to as a
   *         goal constraint. Also does some statistics (decreases global
   *         constraint count).
   */
  Constraint1D(double t, double x);

  /*! \brief Deep copy from another constraint. Also the unique id is copied.
   *         Also does some statistics (decreases global constraint count).
   */
  Constraint1D(const Constraint1D& copyFromMe);

  /*! \brief Deep copy from another constraint. Also the unique id is copied.
   *         Also does some statistics (decreases global constraint count).
   */
  Constraint1D(const std::shared_ptr<Constraint1D> copyFromMe);

  /*! \brief Does some statistics (decreases global constraint count).
   */
  ~Constraint1D();

  /*! \brief Returns a clone of this. Internally this method calls the copy
   *         constructor.
   */
  Constraint1D* clone() const;

  /*! \brief Checks if the constraint's numbers are all finite.
   */
  bool check() const;

  /*! \brief Get and set methods for all member variables.
   */
  void set(double t, double x, double x_dot, double x_ddot, int flag);
  double getTime() const;
  double getPosition() const;
  double getVelocity() const;
  double getAcceleration() const;
  int getFlag() const;
  size_t getID() const;
  void setTime(double t);
  void setFlag(int flag);
  void setPosition(double x);
  void setVelocity(double x_dot);
  void setAcceleration(double x_ddot);

  /*! \brief Adds dt to the constraint's internal time.
   */
  void shiftTime(double dt);

  /*! \brief Sets the id value to a unique value so that it can unambiguously
   *         be retrieved.
   */
  void assignUniqueID();

  /*! \brief Two instances are equal if all members (including id) are
   *         identical.
   */
  bool operator == (const Constraint1D& other) const;

  /*! \brief Two instances are not equal if any members (including id)
   *         differs.
   */
  bool operator != (const Constraint1D& other) const;

  /*! \brief One instance is larger than another if the time is later.
   */
  bool operator > (const Constraint1D& other) const;

  /*! \brief One instance is larger than another if the time is earlier.
   */
  bool operator < (const Constraint1D& other) const;

  /*! \brief Overloaded ofstream operator so that the class can conveniently
   *         be printed using cout etc.
   */
  friend std::ostream& operator<<(std::ostream& output,
                                  const Constraint1D& viaPt);

  /*! \brief All constraints that have been constructed and not deleted
   *         within the running program.
   */
  static size_t getNumConstraints();

  struct lesser
  {
    bool operator()(const Constraint1D* struct1,
                    const Constraint1D* struct2)
    {
      return (struct1->t < struct2->t) ? true : false;
    }
  };

  struct lesser_sptr
  {
    bool operator()(const std::shared_ptr<Constraint1D> struct1,
                    const std::shared_ptr<Constraint1D> struct2)
    {
      return (struct1->t < struct2->t) ? true : false;
    }
  };

private:
  double t, x, x_dot, x_ddot;
  int flag;
  size_t id;
  static size_t constraintCount;
};

}   // namespace tropic

#endif   // TROPIC_CONSTRAINT1D_H
