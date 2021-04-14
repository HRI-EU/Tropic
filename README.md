# Tropic

<p float="left">
<img src="doc/images/example.png" width="100%" > 
</p>

Tropic stands for "TRajectory Online Planning in InteraCtion". We selected the
name "tropic", because it's a library with a number of hot features:

  - Implementations of minimum-jerk trajectory profile with arbitrary number
    of constraints, to be flexibly applied to position, velocity or
    acceleration levels, or any combination of these
  - Flexible way of incorporating new kinds of trajectory profiles
  - ConstraintSet class implementing a hierarchically organized collection of
    constraints spanning several trajectories, similar to the concept of
    motion primitives
  - Implementation of trajectories in several orientation representations
    without any representational issues such as jumps at representation
    boundaries
  - Receding horizon implementation to step and re-plan trajectories at every
    time step
  - Class to couple trajectory generation to task-level control to easily run
    with inverse kinematics / inverse dynamics
  - XML factory to read and write trajectories from and to file
  - Classes to display trajectories in OpenScenegraph and for plotting

Tropic is written entirely in C++ and requires the standard C++11. It has
been successfully compiled and ran on several Linux and Windows systems
(Ubuntu 14, 16, 18, 20, Visual studio 2015, 2017). It depends on the Rcs
library, a BSD 3-clause licensed library that can freely be downloaded from
GitHub: https://github.com/HRI-EU/Rcs. It has been hardened in several
research projects, such as for instance this one: https://www.honda-ri.de/human-robot-cooperative-object-manipulation-with-contact-changes

## Getting Started

Tropic can be compiled with the cmake build system. To compile it, just type:

    cd <build-directory>
    cmake <source-directory>
    make 

To build the doxygen documentation, just type:

    make TrajectoryDoc

## Examples

The library comes with several examples. The file **TestSimpleTrajectory.cpp**
contains examples for plotting of single trajectories and stepping them over
time. The file  **TestTrajectory.cpp** contains examples for high-dimensional
trajectories, and how to step and modify them interactively at run-time. The
examples can be started with the command-line option "-h", which will display
a help message on all available command line options and graphics window
key interactions.

## License

This project is licensed under the BSD 3-clause license - see the [LICENSE.md](LICENSE.md) file for details

## To do

  - Rename to better wording:
  - fromXML() and toXML() should return false instead of exiting fatally
  - relPos / relRot in PositionConstraint / EulerConstraint
  - How to deal with transform pointers as anchors? Body names?
  - Fix activation access in testIK() of TestTrajectory.cpp
