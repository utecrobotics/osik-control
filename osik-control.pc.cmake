prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_FULL_LIBDIR@
includedir=@CMAKE_INSTALL_FULL_INCLUDEDIR@

Name: Operational Space Inverse Kinematics (OSIK) Control
Description: Kinematic Whole-body motion generation
URL: TODO
Version: @PROJECT_VERSION@
Requires: rbdl
Conflicts:
Libs: -L${libdir} -losik-control
Libs.private:
Cflags: -I${includedir}
