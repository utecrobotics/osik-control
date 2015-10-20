CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)

PROJECT(OSIK-Control CXX)

SET(OSIK-Control_VERSION 0.0.1)
SET(PROJECT_VERSION ${OSIK-Control_VERSION})

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

FIND_PACKAGE(PkgConfig)
FIND_PACKAGE(qpOASES)

PKG_CHECK_MODULES(RBDL REQUIRED rbdl)
PKG_CHECK_MODULES(EIGEN3 eigen3>=3.2 REQUIRED)

# Only needed when loading a urdf model
PKG_CHECK_MODULES(URDF REQUIRED urdf)
# Add variable for rbdl urdf reader
IF (RBDL_FOUND)
  SET(RBDL_URDFReader_LIBRARIES rbdl_urdfreader)
ENDIF (RBDL_FOUND)


# Set default build type to 'Release' if none is specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Using build type: 'Release' (none was specified).")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
ENDIF()


ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTests)


# Set pkg-config
SET(CMAKE_INSTALL_FULL_LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
SET(CMAKE_INSTALL_FULL_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
CONFIGURE_FILE (
  ${CMAKE_SOURCE_DIR}/osik-control.pc.cmake
  ${CMAKE_BINARY_DIR}/osik-control.pc @ONLY
  )
# Install pkg-config file
INSTALL (
  FILES  ${CMAKE_BINARY_DIR}/osik-control.pc
  DESTINATION lib/pkgconfig
  )

# Doxygen for documentation
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  CONFIGURE_FILE(
    ${CMAKE_SOURCE_DIR}/doc/Doxyfile.in 
    ${CMAKE_BINARY_DIR}/doc/Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc
  ${DOXYGEN_EXECUTABLE} 
  ${CMAKE_BINARY_DIR}/doc/Doxyfile
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/doc
  COMMENT "Generating Doxygen documentation" VERBATIM
)

# Install documentation
# INSTALL(DIRECTORY ${CMAKE_BINARY_DIR}/doc/doxygen-html
#   DESTINATION share/doc/osik-control)
 
endif(DOXYGEN_FOUND)
