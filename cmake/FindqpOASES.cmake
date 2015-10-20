# FindqpOASES
# -----------
#
# Try to find qpOASES library.
#
# To find, it is assumed that qpOASES has been installed in a linux platform
# (using cmake install) to a given prefix. Please specify the installation
# prefix with the environmental variable
#
#    qpOASES_INSTALL_PREFIX
#
# This variable will be used as a hint to find the installed header files and
# libraries.
#
# Once done, the following variables are defined:
#
#    qpOASES_FOUND         - System has qpOASES
#    qpOASES_INCLUDE_DIRS  - qpOASES include directory
#    qpOASES_LIBRARIES     - qpOASES libraries
#

INCLUDE(FindPackageHandleStandardArgs)

FIND_PATH(qpOASES_INCLUDEDIR
  NAMES qpOASES.hpp
  HINTS "${qpOASES_INSTALL_PREFIX}"
  ENV qpOASES_INSTALL_PREFIX
  PATH_SUFFIXES include)
FIND_LIBRARY(qpOASES_LIB
  NAMES qpOASES
  HINTS "${qpOASES_INSTALL_PREFIX}"
  ENV qpOASES_INSTALL_PREFIX
  PATH_SUFFIXES lib
  libs)

SET(qpOASES_INCLUDE_DIRS ${qpOASES_INCLUDEDIR})
SET(qpOASES_LIBRARIES ${qpOASES_LIB})

FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  qpOASES DEFAULT_MSG
  qpOASES_LIBRARIES qpOASES_INCLUDE_DIRS)

SET(qpOASES_FOUND ${QPOASES_FOUND})
