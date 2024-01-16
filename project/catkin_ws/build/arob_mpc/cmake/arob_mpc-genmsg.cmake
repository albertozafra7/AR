# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arob_mpc: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iarob_mpc:/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arob_mpc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg" NAME_WE)
add_custom_target(_arob_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arob_mpc" "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg" "geometry_msgs/Pose:geometry_msgs/Point:std_msgs/Header:geometry_msgs/Twist:geometry_msgs/Quaternion:geometry_msgs/Accel:geometry_msgs/PoseStamped:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arob_mpc
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arob_mpc
)

### Generating Services

### Generating Module File
_generate_module_cpp(arob_mpc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arob_mpc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arob_mpc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arob_mpc_generate_messages arob_mpc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg" NAME_WE)
add_dependencies(arob_mpc_generate_messages_cpp _arob_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arob_mpc_gencpp)
add_dependencies(arob_mpc_gencpp arob_mpc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arob_mpc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arob_mpc
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arob_mpc
)

### Generating Services

### Generating Module File
_generate_module_eus(arob_mpc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arob_mpc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arob_mpc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arob_mpc_generate_messages arob_mpc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg" NAME_WE)
add_dependencies(arob_mpc_generate_messages_eus _arob_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arob_mpc_geneus)
add_dependencies(arob_mpc_geneus arob_mpc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arob_mpc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arob_mpc
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arob_mpc
)

### Generating Services

### Generating Module File
_generate_module_lisp(arob_mpc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arob_mpc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arob_mpc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arob_mpc_generate_messages arob_mpc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg" NAME_WE)
add_dependencies(arob_mpc_generate_messages_lisp _arob_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arob_mpc_genlisp)
add_dependencies(arob_mpc_genlisp arob_mpc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arob_mpc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arob_mpc
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arob_mpc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(arob_mpc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arob_mpc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arob_mpc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arob_mpc_generate_messages arob_mpc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg" NAME_WE)
add_dependencies(arob_mpc_generate_messages_nodejs _arob_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arob_mpc_gennodejs)
add_dependencies(arob_mpc_gennodejs arob_mpc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arob_mpc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arob_mpc
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arob_mpc
)

### Generating Services

### Generating Module File
_generate_module_py(arob_mpc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arob_mpc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arob_mpc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arob_mpc_generate_messages arob_mpc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/msg/vector_poses.msg" NAME_WE)
add_dependencies(arob_mpc_generate_messages_py _arob_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arob_mpc_genpy)
add_dependencies(arob_mpc_genpy arob_mpc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arob_mpc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arob_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arob_mpc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(arob_mpc_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arob_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arob_mpc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(arob_mpc_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arob_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arob_mpc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(arob_mpc_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arob_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arob_mpc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(arob_mpc_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arob_mpc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arob_mpc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arob_mpc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(arob_mpc_generate_messages_py geometry_msgs_generate_messages_py)
endif()
