################################################################################
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
################################################################################
CMAKE_MINIMUM_REQUIRED(VERSION 3.2)
SET(BST_INSTALL_CATEGORY Libraries) 


################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
SET(STANDALONE_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  PROJECT(Tropic)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release." FORCE)
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
  LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/Rcs/1.0/share/cmake/Rcs)
  FIND_PACKAGE(Rcs REQUIRED)
  SET(STANDALONE_PROJECT ON)
  FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/External/MuJoCo/1.0/$ENV{MAKEFILE_PLATFORM}/lib/cmake/mujoco)
  FIND_PACKAGE(mujoco QUIET)   # Needed if mujoco is compiled against RcsPhysics
  IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET(CMAKE_INSTALL_PREFIX "$ENV{SIT}/Libraries/Tropic/1.0" CACHE PATH "Default sand box directory" FORCE)
  ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
ELSE()
  SET(RCS_CONFIG_DIR "config")
  LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
ENDIF()

################################################################################
# Compile settings
################################################################################
SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
SET(BUILD_SHARED_LIBS ON)

################################################################################
# Setup exported target collection
################################################################################

# Setup TROPIC_EXPORT_LIBRARIES so that CMakeLists.txt can modify it
SET(TROPIC_EXPORT_LIBRARIES "" CACHE INTERNAL "")

# Create a macro for adding an export library
FUNCTION(TROPIC_ADD_EXPORT_LIBRARY library)
  SET(TROPIC_EXPORT_LIBRARIES ${TROPIC_EXPORT_LIBRARIES} ${library} CACHE INTERNAL "")
ENDFUNCTION()

IF (WIN32)
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)   # M_PI etc. for windows
ENDIF(WIN32)

################################################################################
# Compile settings
################################################################################
IF(UNIX)
  # SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -Wall -pedantic -fPIC -Wno-long-long -Wno-variadic-macros -std=c99")
  # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -fPIC -Wno-format -Wno-long-long -Wno-variadic-macros")
  # ADD_DEFINITIONS(-D__linux__)

  # Flag -rdynamic needed for backtrace()
  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -rdynamic")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
  ENDIF()
  
  IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    # setting march to core2 to enable valgrind debugging (also good for Xeon)
    # Low level optimization for debug mode, flag for checking stack corruption, flag for debug output
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=core2 -O0 -fstack-protector-all -ggdb")
  ELSE()
    # setting march to native for optimal performance on local machine
    # Strong optimization for release mode
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native -ggdb")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -ggdb")
  ENDIF()
  
ENDIF()

################################################################################
#
# Add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH. Required for Vortex and other
# dependencies from the SIT.
# Note: This only works with CMake >= 3.0. With 2.8, the library directories of
# the dependencies must be added to the LD_LIBRARY_PATH manually in order to
# use the installed executables. This has no effect on using the package in
# another CMake project.
#
################################################################################
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

################################################################################
# The RPATH to be used when installing, but only if it's not a system directory
# See CMake wiki:
# (https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling)
################################################################################
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib/$ENV{MAKEFILE_PLATFORM}" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/$ENV{MAKEFILE_PLATFORM}")
ENDIF("${isSystemDir}" STREQUAL "-1")

###############################################################################
# Library
###############################################################################
SET(TROPIC_SRCS
src/ActivationSet.cpp
src/Trajectory1D.cpp
src/TrajectoryPlotter1D.cpp
src/Constraint1D.cpp
src/ViaPointTrajectory1D.cpp
src/ZigZagTrajectory1D.cpp
src/TrajectoryND.cpp
src/TrajectoryController.cpp
src/ConstraintSet.cpp
src/ActivationPoint.cpp
src/PositionConstraint.cpp
src/KeepPositionConstraint.cpp
src/AnchoredPositionConstraint.cpp
src/EulerConstraint.cpp
src/PoseConstraint.cpp
src/PolarConstraint.cpp
src/MultiGoalConstraint.cpp
src/VectorConstraint.cpp
src/ConstraintFactory.cpp
src/GraphConstraint.cpp
src/ConnectBodyConstraint.cpp
src/JointWeightConstraint.cpp
src/LiftObject.cpp
)

ADD_LIBRARY(Tropic ${TROPIC_SRCS})

TARGET_INCLUDE_DIRECTORIES(Tropic PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    
    # Header install location
    $<INSTALL_INTERFACE:include>)

TARGET_LINK_LIBRARIES(Tropic RcsCore)

ADD_LIBRARY(TropicExamples examples/ExampleTrajectoryIK.cpp)# examples/ExampleActions.cpp)
TARGET_LINK_LIBRARIES(TropicExamples RcsGraphics RcsPhysics RcsGui Tropic)

# We add this option to force the linker to export the symbols. This
# solves the issue that it will not be exposed, since all initialization 
# happens inside statically. Under Unix, we can use -Wl,--no-as-needed
IF (WIN32)
  TARGET_LINK_OPTIONS(TropicExamples PUBLIC /INCLUDE:TropicExampleInfo)
ENDIF()

###############################################################################
# Install the libraries
###############################################################################
INSTALL(TARGETS Tropic TropicExamples EXPORT TropicExport DESTINATION lib/$ENV{MAKEFILE_PLATFORM})

###############################################################################
# Install the headers
###############################################################################
INSTALL(
  DIRECTORY src/ 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

###############################################################################
# Mark for export
###############################################################################
TROPIC_ADD_EXPORT_LIBRARY(Tropic)

###############################################################################
# copy the .dll file to the same folder as the executable
###############################################################################
IF (WIN32)
ADD_CUSTOM_COMMAND(
    TARGET Tropic POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Tropic.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Tropic.dll)
  ADD_CUSTOM_COMMAND(
      TARGET TropicExamples POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/TropicExamples.dll
      ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/TropicExamples.dll)
ENDIF()

###############################################################################
# Executables
###############################################################################
IF(NOT HEADLESS_BUILD)

  ADD_EXECUTABLE(TestTrajectory examples/TestTrajectory.cpp)
  TARGET_COMPILE_DEFINITIONS(TestTrajectory PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
  TARGET_LINK_LIBRARIES(TestTrajectory RcsGui RcsGraphics Tropic)
  IF(STANDALONE_PROJECT)
    TARGET_LINK_LIBRARIES(TestTrajectory Qt5::Core Qt5::Gui Qt5::Widgets)
  ENDIF()

  ADD_EXECUTABLE(TestSimpleTrajectory examples/TestSimpleTrajectory.cpp)
  TARGET_COMPILE_DEFINITIONS(TestSimpleTrajectory PRIVATE -DRCS_CONFIG_DIR="${RCS_CONFIG_DIR}")
  TARGET_LINK_LIBRARIES(TestSimpleTrajectory Tropic)
  IF(STANDALONE_PROJECT)
    TARGET_LINK_LIBRARIES(TestSimpleTrajectory Qt5::Core Qt5::Gui Qt5::Widgets)
  ENDIF()
  
  ADD_EXECUTABLE(ExampleRunnerTropic examples/ExampleRunner.cpp)

  IF (UNIX)
    TARGET_LINK_LIBRARIES(ExampleRunnerTropic -Wl,--no-as-needed RcsExamples TropicExamples -Wl,--as-needed)
  ELSE()
    TARGET_LINK_LIBRARIES(ExampleRunnerTropic RcsExamples TropicExamples)
  ENDIF()

  ###############################################################################
  # Install the binaries
  ###############################################################################
  INSTALL(TARGETS TestTrajectory TestSimpleTrajectory RUNTIME DESTINATION bin/$ENV{MAKEFILE_PLATFORM} LIBRARY DESTINATION lib/$ENV{MAKEFILE_PLATFORM})

ENDIF(NOT HEADLESS_BUILD)


################################################################################
# Copy config directory
# Deep copy of the config directory for windows
################################################################################
SET(CFG_FROM ${CMAKE_CURRENT_SOURCE_DIR}/config)
SET(CFG_TO ${PROJECT_BINARY_DIR}/config/xml/Tropic)

IF (WIN32 AND (NOT IS_DIRECTORY "${CFG_TO}") AND (EXISTS "${CFG_TO}"))
  MESSAGE(STATUS "Removing non-directory ${CONFIG_DST_DIR}")
  FILE(REMOVE ${CONFIG_DST_DIR})
ENDIF()

IF(NOT EXISTS "${CFG_TO}")

  IF (NOT EXISTS ${PROJECT_BINARY_DIR}/config/xml)
    FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/config/xml)
  ENDIF()
  
  IF(UNIX)
    SET(CONFIGDIR_COPY_COMMAND create_symlink)
    MESSAGE(STATUS "Linking config directory from ${CFG_FROM} to ${CFG_TO}")
  ELSE()
    SET(CONFIGDIR_COPY_COMMAND copy_directory)
    MESSAGE(STATUS "Copying config directory from ${CFG_FROM} to ${CFG_TO}")
  ENDIF()

  EXECUTE_PROCESS (COMMAND
   ${CMAKE_COMMAND} -E ${CONFIGDIR_COPY_COMMAND} ${CFG_FROM} ${CFG_TO})
ELSE()
  MESSAGE(STATUS "Config directory ${CFG_TO} already exists - skipping")
ENDIF()

################################################################################
# Target for Doxygen documentation
################################################################################
ADD_SUBDIRECTORY(doc)

################################################################################
# Configure unit test script
################################################################################
IF(UNIX)
  SET(TEST_EXE_DIR "${EXECUTABLE_OUTPUT_PATH}")
  SET(TESTSCRIPT_NAME "${TEST_EXE_DIR}/tropic_unittest.sh")
ELSEIF(WIN32)
  SET(TEST_EXE_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
  SET(TESTSCRIPT_NAME "${TEST_EXE_DIR}/tropic_unittest.bat")
ENDIF()

IF (UNIX)
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/unittest.in ${TESTSCRIPT_NAME} @ONLY)
  ADD_CUSTOM_TARGET(tropicunittest 
      COMMAND ${TESTSCRIPT_NAME}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Performing Tropic unittests" VERBATIM)
ELSE()
  MESSAGE(STATUS "Tropic unit tests for Linux only")
ENDIF()

################################################################################
# Target export script
################################################################################
INCLUDE(ExportTropic)
