osik-control - Operational Space Inverse Kinematics Control


Introduction
============

osik-control is a C++ library that allows to generate whole-body motion for
multi-articulated robots using inverse kinematics. It is based upon the Rigid
Body Dynamics Library (RBDL).

Setup: Building and Installation
================================

The osik-control library uses qpOASES to solve QPs. Since qpOASES does not
provide a pkg-config file, before compiling, please indicate the prefix that
was used when installing qpOASES:

    export qpOASES_INSTALL_PREFIX=/prefix/to/qpoasis/installation

The osik-control library is built using CMake. To compile the library in a
separate directory use:

    mkdir build
    cd build/
    cmake .. -DCMAKE_INSTALL_PREFIX=your_prefix
    make

where your_prefix is the path where the library will be installed. Please note
that CMake produces a `CMakeCache.txt` file which should be deleted to
reconfigure a package from scratch. To install the library use:
    
    make install

The documentation is contained in the code and can be extracted using
doxygen. To generate it, use:

    make doc

This will generate documentation in build/doc/doxygen-html. To see it, launch
index.html with your favorite browser.


### Dependencies

The osik-control library depends on the following libraries which have to be
available on your machine.

 - Libraries:
   - rbdl (with urdf support)
   - qpOASES
   - Eigen (>=3.2)
 - System tools:
   - CMake (>=2.8)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


