# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
set(EXE_TARGET_NAME JointControlModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Find required package
include(FindPackageHandleStandardArgs)
find_package(Threads REQUIRED)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/RobotHelper.cpp
  src/WalkingPIDHandler.cpp
  src/jointControl.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/RobotHelper.hpp
  include/WalkingPIDHandler.hpp
  include/jointControl.hpp
  )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

target_link_libraries(${EXE_TARGET_NAME}
  ${YARP_LIBRARIES}
  ${iDynTree_LIBRARIES}
  Threads::Threads
  UtilityLibrary)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
