# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dmp: 4 messages, 5 services")

set(MSG_I_FLAGS "-Idmp:/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dmp_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg" ""
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg" ""
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg" "dmp/DMPPoint"
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv" "dmp/DMPPoint:dmp/DMPTraj"
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv" "dmp/DMPPoint:dmp/DMPData:dmp/DMPTraj"
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv" "dmp/DMPData"
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv" "dmp/DMPPoint"
)

get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv" NAME_WE)
add_custom_target(_dmp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dmp" "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv" "geometry_msgs/PoseStamped:geometry_msgs/Point:geometry_msgs/Pose:nav_msgs/Path:geometry_msgs/Quaternion:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)
_generate_msg_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)
_generate_msg_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)
_generate_msg_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)

### Generating Services
_generate_srv_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)
_generate_srv_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)
_generate_srv_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)
_generate_srv_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)
_generate_srv_cpp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
)

### Generating Module File
_generate_module_cpp(dmp
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dmp_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dmp_generate_messages dmp_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv" NAME_WE)
add_dependencies(dmp_generate_messages_cpp _dmp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dmp_gencpp)
add_dependencies(dmp_gencpp dmp_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dmp_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)
_generate_msg_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)
_generate_msg_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)
_generate_msg_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)

### Generating Services
_generate_srv_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)
_generate_srv_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)
_generate_srv_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)
_generate_srv_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)
_generate_srv_eus(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
)

### Generating Module File
_generate_module_eus(dmp
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dmp_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dmp_generate_messages dmp_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv" NAME_WE)
add_dependencies(dmp_generate_messages_eus _dmp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dmp_geneus)
add_dependencies(dmp_geneus dmp_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dmp_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)
_generate_msg_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)
_generate_msg_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)
_generate_msg_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)

### Generating Services
_generate_srv_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)
_generate_srv_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)
_generate_srv_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)
_generate_srv_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)
_generate_srv_lisp(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
)

### Generating Module File
_generate_module_lisp(dmp
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dmp_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dmp_generate_messages dmp_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv" NAME_WE)
add_dependencies(dmp_generate_messages_lisp _dmp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dmp_genlisp)
add_dependencies(dmp_genlisp dmp_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dmp_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)
_generate_msg_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)
_generate_msg_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)
_generate_msg_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)

### Generating Services
_generate_srv_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)
_generate_srv_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)
_generate_srv_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)
_generate_srv_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)
_generate_srv_nodejs(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
)

### Generating Module File
_generate_module_nodejs(dmp
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dmp_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dmp_generate_messages dmp_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv" NAME_WE)
add_dependencies(dmp_generate_messages_nodejs _dmp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dmp_gennodejs)
add_dependencies(dmp_gennodejs dmp_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dmp_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)
_generate_msg_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)
_generate_msg_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)
_generate_msg_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)

### Generating Services
_generate_srv_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)
_generate_srv_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg;/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)
_generate_srv_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)
_generate_srv_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv"
  "${MSG_I_FLAGS}"
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)
_generate_srv_py(dmp
  "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
)

### Generating Module File
_generate_module_py(dmp
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dmp_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dmp_generate_messages dmp_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPData.msg" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPoint.msg" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPPointStamp.msg" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/msg/DMPTraj.msg" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/LearnDMPFromDemo.srv" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/SetActiveDMP.srv" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GetDMPStepPlan.srv" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chy/sigma_iiwa_simulation/ur_sigma/src/dmp/srv/GoalToPath.srv" NAME_WE)
add_dependencies(dmp_generate_messages_py _dmp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dmp_genpy)
add_dependencies(dmp_genpy dmp_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dmp_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dmp
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dmp_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(dmp_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dmp
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dmp_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(dmp_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dmp
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dmp_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(dmp_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dmp
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dmp_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(dmp_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dmp
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dmp_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(dmp_generate_messages_py nav_msgs_generate_messages_py)
endif()
