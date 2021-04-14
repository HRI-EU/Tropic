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

#ifndef TROPIC_CONSTRAINTFACTORY_H
#define TROPIC_CONSTRAINTFACTORY_H

#include "ConstraintSet.h"



/*! \brief Convenience macro to register constraints in the factory. Here is an
 * example: For the constraint ABC, the macro expands to
 * static tropic::ConstraintFactoryRegistrar<ABC> ABC_("ABC")
 */
#define REGISTER_CONSTRAINT(T) static tropic::ConstraintFactoryRegistrar<T> T ## _(#T)


namespace tropic
{

/*! \brief Factory class for TrajectoryConstraintSet classes and its
 *         derieved classes. The factory implements methods to construct
 *         classes of type TrajectoryConstraintSet (and derieved from them)
 *         from an xml node or file containing an xml description. It is based
 *         on a registrar class that registers methods for reading and writing
 *         xml descriptions of such constraints. In order to enable a
 *         class derived from TrajectoryConstraintSet to be used with this
 *         factory class, the following needs to be provided:
 *
 *         - A constructor that constructs an instance from an xml node: e.g.
 *           MyNewSet::MyNewSet(xmlNode* node);
 *         - Inserting a macro to register the new set in the implementation
 *           file: REGISTER_CONSTRAINT(MyNewSet);
 *
 *         Most of the classes in this library have been implemented like this.
 *         See for instance \ref PositionConstraint.
 */
class ConstraintFactory
{
  template <class T> friend class ConstraintFactoryRegistrar;

public:

  /*! \brief Creates a new constraint set by name using the registered
   *         construction function.
   *
   * \param node      Xml configuration
   * \return          New TrajectoryConstraintSet instance or nullptr in
   *                  case of failure
   */
  static std::shared_ptr<ConstraintSet> create(xmlNode* node);

  /*! \brief Creates a new constraint set by name using the registered
   *         construction function.
   *
   * \param fileName      Xml configuration file name
   * \return              New TrajectoryConstraintSet instance or nullptr in
   *                      case of failure
   */
  static std::shared_ptr<ConstraintSet> create(std::string fileName);

  /*! \brief Prints out all registered trajectory constraints to the console
   */
  static void print();

private:

  /*! \brief Private constructor because ConstraintFactory is a singleton class
   */
  ConstraintFactory();

  /*! \brief Signature of constraint creation function.
   */
  typedef std::shared_ptr<ConstraintSet> (*ConstraintMaker)(xmlNode* node);

  /*! \brief Registers a new function for creating constraints. You can not
   *        call this function directly. Instead us the above macro.
   */
  static void registerConstraint(const char* name,
                                 ConstraintMaker createFunction);

  static std::map<std::string, ConstraintFactory::ConstraintMaker>& constructorMap();
};





/*! \brief Registrar class for trajectory constraint classes. Here is how to use
 *        it:
 *        - Implement a trajectory constraint derieved from TrajectoryConstraintSet
 *        - In the implementation of this class on the global scope, add:<br>
 *          REGISTER_CONSTRAINT(MyCoolNewConstraint);
 *        - This registers a constraint of type MyCoolNewConstraint that can be
 *          instantiated : <br>
 *          auto c = ConstraintFactory::create(node);
 */
template<class T>
class ConstraintFactoryRegistrar
{
public:

  /*! \brief Registers a new trajectory constraint with a given name. This line
   *         needs to be put into the cpp file:
   *         REGISTER_CONSTRAINT(MyCoolNewConstraint);
   *
   *         Then, you can create a MyCoolNewConstraint such as
   *         auto constraint = ConstraintFactory::create(node);
   *
   *  \param className The name that is used for instanciating a new
   *                   physics simulation by name
   */
  ConstraintFactoryRegistrar(const char* className)
  {
    // Register the function to create and check the physics simulation
    ConstraintFactory::registerConstraint(className,
                                          &ConstraintFactoryRegistrar::create);
  }

private:

  /*! \brief This function creates a new physics simulation instance of type T
   *         passing the given variables to the respective constructor. We call
   *         the empty constructor and the intialize() function separately,
   *         since we use polymorphism during the initialization (for instance
   *         during the construction of the bullet soft physics). This cannot
   *         be done inside the constructor, since it always will call the
   *         methods of the base class.
   *
   * \param node    xml configuration
   * \return        New TrajectoryConstraintSet of type T
   */
  static std::shared_ptr<ConstraintSet> create(xmlNode* node)
  {
    return std::make_shared<T>(node);
  }
};

}

#endif // TROPIC_CONSTRAINTFACTORY_H
