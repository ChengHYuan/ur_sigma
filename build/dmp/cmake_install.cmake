# Install script for directory: /home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dmp/msg" TYPE FILE FILES
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dmp/srv" TYPE FILE FILES
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dmp/cmake" TYPE FILE FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/dmp/catkin_generated/installspace/dmp-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/devel/include/dmp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/devel/share/roseus/ros/dmp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/devel/share/common-lisp/ros/dmp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/devel/share/gennodejs/ros/dmp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/chy/sigma_iiwa_simulation/ur_sigma/devel/lib/python3/dist-packages/dmp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/devel/lib/python3/dist-packages/dmp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/dmp/catkin_generated/installspace/dmp.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dmp/cmake" TYPE FILE FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/build/dmp/catkin_generated/installspace/dmp-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dmp/cmake" TYPE FILE FILES
    "/home/chy/sigma_iiwa_simulation/ur_sigma/build/dmp/catkin_generated/installspace/dmpConfig.cmake"
    "/home/chy/sigma_iiwa_simulation/ur_sigma/build/dmp/catkin_generated/installspace/dmpConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dmp" TYPE FILE FILES "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/package.xml")
endif()

