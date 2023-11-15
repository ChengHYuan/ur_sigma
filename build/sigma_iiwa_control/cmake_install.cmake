# Install script for directory: /home/chy/sigma_iiwa_simulation/ur_sigma/src/sigma_iiwa_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/chy/sigma_iiwa_simulation/ur_sigma/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/sigma_iiwa_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sigma_iiwa_control/cmake" TYPE FILE FILES
    "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/sigma_iiwa_controlConfig.cmake"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/sigma_iiwa_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sigma_iiwa_control" TYPE FILE FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/src/sigma_iiwa_control/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/iiwa_sigma_pose2.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/iiwa_sigma_pose_position.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/iiwa_init_pose.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/iiwa_drag.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/iiwa_client.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/iiwa_client_normalmode.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/read_button.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/read_button_2.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/iiwa_velocity_rate.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/record_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sigma_iiwa_control" TYPE PROGRAM FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/sigma_iiwa_control/catkin_generated/installspace/read_iiwa_joint_velocity.py")
endif()

