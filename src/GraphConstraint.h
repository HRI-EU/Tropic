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

#ifndef TROPIC_GRAPHCONSTRAINT_H
#define TROPIC_GRAPHCONSTRAINT_H

#include "ConstraintSet.h"

#include <Rcs_graph.h>



namespace tropic
{

/*! \brief Base class for constraint sets that own a reference to a graph.
 *         This class is not handled in the step() methods of the trajectories
 *         if no child sets or constraints are added. However, in the method
 *         TrajectoryControllerBase::addAndApply(), the graph of the
 *         TrajectoryControllerBase is set to this class's graph member, so
 *         that any derived class cann access the graph in the compute()
 *         method. This allows to perform consistent graph operations on the
 *         set when stepping trajectories. An example for this is to change the
 *         rigid body topology, to change distance calculation flags, etc.
 */
class GraphConstraint : public ConstraintSet
{
public:

  GraphConstraint();

  GraphConstraint(const GraphConstraint& other);

  GraphConstraint(xmlNode* node);

  virtual GraphConstraint* clone();

  virtual ~GraphConstraint();

  virtual void setGraph(RcsGraph* newGraph);

protected:

  RcsGraph* graph;
};

}   // namespace Rcs


#endif   // RCS_GRAPHCONSTRAINT_H
