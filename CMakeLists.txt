CMAKE_MINIMUM_REQUIRED(VERSION 3.4)
SET(BST_INSTALL_CATEGORY Libraries) 


################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
SET(STANDALONE_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  PROJECT(Tropic)
  LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
  LIST(APPEND CMAKE_PREFIX_PATH $ENV{SIT}/Libraries/Rcs/1.0/share/cmake/Rcs)
  FIND_PACKAGE(Rcs REQUIRED)
  MESSAGE(STATUS "Compiling Tropic as stand-alone project")
  SET(STANDALONE_PROJECT ON)
  SET(LIBRARY_LINKING_TYPE "SHARED")
  FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
  IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET(CMAKE_INSTALL_PREFIX "$ENV{SIT}/Libraries/Tropic/1.0" CACHE PATH "Default sand box directory" FORCE)
  ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
ELSE()
  SET(RCS_CONFIG_DIR "config")
  INCLUDE(CompileSettings)
  LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/Trajectory/cmake)
  MESSAGE(STATUS "Compiling ${PROJECT_NAME} as in-tree project")
ENDIF()

################################################################################
#
# Setup exported target collection
#
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
#
# The RPATH to be used when installing, but only if it's not a system directory
# See CMake wiki:
# (https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling)
#
################################################################################
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib/$ENV{MAKEFILE_PLATFORM}" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/$ENV{MAKEFILE_PLATFORM}")
ENDIF("${isSystemDir}" STREQUAL "-1")

###############################################################################
#
# CMakeLists file for the Tropic library
#
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
src/EulerConstraint.cpp
src/PoseConstraint.cpp
src/PolarConstraint.cpp
src/MultiGoalConstraint.cpp
src/ConstraintFactory.cpp
src/GraphConstraint.cpp
)

ADD_LIBRARY(Tropic ${LIBRARY_LINKING_TYPE} ${TROPIC_SRCS})

TARGET_INCLUDE_DIRECTORIES(Tropic PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    
    # Header install location
    $<INSTALL_INTERFACE:include>)

TARGET_LINK_LIBRARIES(Tropic RcsCore)

# Install the libraries
INSTALL(TARGETS Tropic EXPORT TropicExport DESTINATION lib/$ENV{MAKEFILE_PLATFORM})

# Install the headers
INSTALL(
  DIRECTORY src/ 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# Mark for export
TROPIC_ADD_EXPORT_LIBRARY(Tropic)

# copy the .dll file to the same folder as the executable
IF (WIN32)
ADD_CUSTOM_COMMAND(
    TARGET Tropic POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Tropic.dll
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Tropic.dll)
ENDIF()

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

  # Install the binaries
  INSTALL(TARGETS TestTrajectory TestSimpleTrajectory RUNTIME DESTINATION bin/$ENV{MAKEFILE_PLATFORM} LIBRARY DESTINATION lib/$ENV{MAKEFILE_PLATFORM})

ENDIF(NOT HEADLESS_BUILD)

ADD_SUBDIRECTORY(doc)

INCLUDE(ExportTropic)
