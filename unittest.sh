#!/bin/bash
#
#  Copyright (c) 2020, Honda Research Institute Europe GmbH.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

set -euo pipefail

function testXML1()
{
    build/"${MAKEFILE_PLATFORM}"/TestTrajectory -m 7 -dl 1 &> UnitTestResults.txt
    testXMLResult1=$?
}

function testXML2()
{
    build/"${MAKEFILE_PLATFORM}"/TestTrajectory -m 8 -dl 1 &> UnitTestResults.txt
    testXMLResult2=$?
}

echo -n "Testing trajectory xml functions part 1 ... "
testXML1

if [ "${testXMLResult1}" -eq 0 ]
then
  echo "succeeded"
elif [ "${testXMLResult1}" -eq 255 ]
then
  echo "failed with more than 255 errors"
else
  echo "failed with ${testXMLResult1} errors" 
fi



echo -n "Testing trajectory xml functions part 2 ... "
testXML2

if [ "${testXMLResult2}" -eq 0 ]
then
  echo "succeeded"
elif [ "${testXMLResult2}" -eq 255 ]
then
  echo "failed with more than 255 errors"
else
  echo "failed with ${testXMLResult2} errors" 
fi

