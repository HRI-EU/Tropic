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

#include "GraphConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>

#include <string>



namespace tropic
{
REGISTER_CONSTRAINT(GraphConstraint);

GraphConstraint::GraphConstraint() : ConstraintSet(), graph(NULL)
{
  setClassName("GraphConstraint");
}

GraphConstraint::GraphConstraint(const GraphConstraint& other) :
  ConstraintSet(other), graph(NULL)
{
}

GraphConstraint::GraphConstraint(xmlNode* node) : ConstraintSet(node), graph(NULL)
{
  setClassName("GraphConstraint");
  fromXML(node);
}

GraphConstraint::~GraphConstraint()
{
}

void GraphConstraint::setGraph(RcsGraph* newGraph)
{
  this->graph = newGraph;
}

GraphConstraint* GraphConstraint::clone() const
{
  GraphConstraint* tSet = new GraphConstraint();
  tSet->constraint = constraint;
  tSet->className = className;
  tSet->graph = NULL;

  for (size_t i = 0; i < set.size(); ++i)
  {
    // Recursive cloning
    auto child = set[i]->clone();
    tSet->add(std::shared_ptr<ConstraintSet>(child));
  }

  return tSet;
}

}   // namespace tropic
