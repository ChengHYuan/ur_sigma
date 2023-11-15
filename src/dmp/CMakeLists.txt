cmake_minimum_required(VERSION 2.8.3)
project(dmp)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
cmake_modules
roscpp
rospy
std_msgs
nav_msgs
message_generation
)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        geometry_msgs
        message_generation
        roscpp
        rospy
        std_msgs
        sensor_msgs
        control_msgs
        trajectory_msgs
        cmake_modules
        moveit_core
        moveit_msgs
        moveit_ros_planning_interface
        moveit_visual_tools
        rosbag
        iiwa_msgs
        )

#set(tinyxml2_DIR /usr/lib/x86_64-linux-gnu)
#set_target_properties( libtinyxml2
#        PROPERTIES IMPORTED_LOCATION
#        ${tinyxml2_DIR}/libtinyxml2.so)

LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu)
link_libraries(libxmlrpcpp.so)
link_directories(/opt/ros/noetic/lib)
link_directories(/home/jiyguo/ur_ws/devel/lib)
link_directories(/home/jiyguo/ur_ws/devel/lib/ati_force_sensor)

add_message_files(
   FILES
   DMPData.msg
   DMPPoint.msg
   DMPPointStamp.msg
   DMPTraj.msg
 )

add_service_files(
   FILES
   GetDMPPlan.srv
   LearnDMPFromDemo.srv
   SetActiveDMP.srv
   GetDMPStepPlan.srv
   GoalToPath.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs

)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS common_msgs image_common message_runtime
)

include_directories(
  ~/tinyxml
  include
  ../ati_force_sensor/src/MCC_ATI
  ../ati_force_sensor/include
        /usr/local/include/kdl
        /usr/include/python2.7
        /usr/include/eigen3
        /opt/ros/noetic/include

        ../../devel/include
        ../dmp/include
        ../gmm/include
        ../robot_kin/include
        ../my_controller/include
        ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(dmp src/dmp.cpp src/fourier_approx.cpp src/radial_approx.cpp src/linear_approx.cpp)
target_link_libraries(dmp tinyxml2)

# add_library(controller ../my_controller/src/robotController/robotController.cpp )
# target_link_libraries(controller  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} robot_kin  orocos-kdl )

add_executable(dmp_server nodes/dmp_server.cpp)
target_link_libraries(dmp_server dmp ${catkin_LIBRARIES} ${Eigen_LIBRARIES} dmp)
#
add_executable(dmp_test nodes/dmp_test.cpp)
target_link_libraries(dmp_test ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2)

add_executable(dmp_test_6 nodes/dmp_test_6.cpp)
target_link_libraries(dmp_test_6 ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2)

add_executable(dmp_test_6_angle nodes/dmp_test_6_angle.cpp)
target_link_libraries(dmp_test_6_angle ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2)

add_executable(dmp_traj_gen nodes/dmp_traj_gen.cpp)
target_link_libraries(dmp_traj_gen ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2 dmp)

#
#add_executable(dmp_learn nodes/dmp_learn.cpp)
#target_link_libraries(dmp_learn ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2)
#
#add_executable(dmp_gen nodes/dmp_gen.cpp)
#target_link_libraries(dmp_gen ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2)
#
#add_executable(dmp_step nodes/dmp_step.cpp)
#target_link_libraries(dmp_step ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2 dmp)
#
#add_executable(dmp_robot nodes/dmp_robot.cpp  ../my_controller/src/robotController/robotController.cpp)
#target_link_libraries(dmp_robot ${catkin_LIBRARIES} ${Boost_LIBRARIES} orocos-kdl robot_kin tinyxml2 dmp)
#
# add_executable(dmp_with_wsc nodes/dmp_with_wsc.cpp  ../my_controller/src/robotController/robotController.cpp)
# target_link_libraries(dmp_with_wsc ${catkin_LIBRARIES} ${Boost_LIBRARIES} orocos-kdl robot_kin tinyxml2 dmp)

# add_executable(mydmp_wsc nodes/mydmp_wsc.cpp src/dtw.cpp src/my_dmp.cpp)
# target_link_libraries(mydmp_wsc controller dmp)

# add_executable(traj_ur_wsc nodes/traj_ur_wsc.cpp)
# target_link_libraries(traj_ur_wsc controller)

#
#add_executable(classify_test nodes/classify_test.cpp src/confidance.cpp)
#target_link_libraries(classify_test ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2)
#
#add_executable(classify_online nodes/classify_online.cpp src/confidance.cpp)
#target_link_libraries(classify_online ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2 dmp)

add_executable(dtw_test nodes/dtw_test.cpp src/dtw.cpp src/my_dmp.cpp)
target_link_libraries(dtw_test ${catkin_LIBRARIES} ${Eigen_LIBRARIES} dmp)



# add_executable(my_dmp_test nodes/my_dmp_test.cpp src/my_dmp.cpp)
# target_link_libraries(my_dmp_test dmp controller)

# add_executable(arbitration_test nodes/arbitration.cpp src/dtw.cpp src/my_dmp.cpp )
# target_link_libraries(arbitration_test controller dmp ati_library)

# add_executable(traj_ur nodes/traj_ur.cpp)
# target_link_libraries(traj_ur controller)

#add_executable(classify nodes/classify.cpp src/dtw.cpp src/my_dmp.cpp)
#target_link_libraries(classify ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} dmp)