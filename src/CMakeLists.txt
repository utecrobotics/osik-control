
# Add the include directory to the include paths
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/include
  ${RBDL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${URDF_INCLUDE_DIRS}
  ${ROSPACK_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIRS}
  )
LINK_DIRECTORIES(
  ${RBDL_LIBRARY_DIRS}
  ${URDF_LIBRARY_DIRS}
)

ADD_LIBRARY(osik-control SHARED
  tools.cpp
  KinematicTask.cpp
  PoseTask.cpp
  KinematicSolver.cpp
  KinematicSolverNS.cpp
  KinematicSolverWQP.cpp
)

TARGET_LINK_LIBRARIES(osik-control
  ${qpOASES_LIBRARIES})


FILE(GLOB HEADERS
  ${CMAKE_SOURCE_DIR}/include/osik_control/*.h
)

# Installation targets
INSTALL(TARGETS osik-control
  DESTINATION lib
)
INSTALL(FILES ${HEADERS}
  DESTINATION include/osik_control
)


