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

#include "ConstraintFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>


namespace tropic
{

/*******************************************************************************
 *
 ******************************************************************************/
ConstraintFactory::ConstraintFactory()
{
}

/*******************************************************************************
 * Print all registered constraints to the console
 ******************************************************************************/
void ConstraintFactory::print()
{
  auto it = constructorMap().begin();

  std::cout << constructorMap().size() << " registered constraints:\n";

  while (it != constructorMap().end())
  {
    std::cout << "  " << it->first << std::endl;
    it++;
  }
}

/*******************************************************************************
 * Creates the constraint for className and the given graph and xml content
 ******************************************************************************/
std::shared_ptr<ConstraintSet> ConstraintFactory::create(xmlNode* node)
{
  RCHECK(node);
  std::shared_ptr<ConstraintSet> newSet;

  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    RLOG(5, "XML node \"%s\" is not a \"ConstraintSet\" - giving up",
         (char*) node->name);
    return nullptr;
  }

  std::string className;
  unsigned int len = Rcs::getXMLNodePropertySTLString(node, "type", className);

  if (len == 0)
  {
    RLOG(1, "ConstraintSet has no type - giving up");
    return nullptr;
  }

  std::map<std::string, ConstraintMaker>::iterator it;
  it = constructorMap().find(className);

  if (it != constructorMap().end())
  {
    newSet = it->second(node);
  }
  else
  {
    RLOG(1, "Couldn't find constructor for class \"%s\"", className.c_str());
  }

  return newSet;
}

/*******************************************************************************
 * Creates the constraint for className and the given graph and xml content
 ******************************************************************************/
std::shared_ptr<ConstraintSet> ConstraintFactory::create(std::string xmlFile)
{
  xmlDocPtr xmlDoc;
  xmlNodePtr node = parseXMLFile(xmlFile.c_str(), "ConstraintSet", &xmlDoc);

  if (!node)
  {
    RLOG(1, "Failed to parse ConstraintSet from file \"%s\"", xmlFile.c_str());
    return nullptr;
  }

  auto tSet = ConstraintFactory::create(node);

  xmlFreeDoc(xmlDoc);

  return tSet;
}

/*******************************************************************************
 * This function is called through the registrar class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void ConstraintFactory::registerConstraint(const char* name,
                                           ConstraintMaker createFunction)
{
  auto it = constructorMap().find(name);
  if (it != constructorMap().end())
  {
    // No log level, this happens before main()
    RMSG("Overwriting a constraint creation function: \"%s\"", name);
  }

  constructorMap()[name] = createFunction;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::map<std::string, ConstraintFactory::ConstraintMaker>&
ConstraintFactory::constructorMap()
{
  static std::map<std::string, ConstraintFactory::ConstraintMaker> cm;
  return cm;
}



}   // namespace tropic
