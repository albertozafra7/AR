cmake_minimum_required(VERSION 3.0.2)
project(arob_mpc)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg mav_trajectory_generation_ros mav_trajectory_generation_example message_generation)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  vector_poses.msg
)

# ## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  INCLUDE_DIRS include
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

add_executable(drone_race src/drone_race.cpp)
add_dependencies(drone_race ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_race ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(mpc_custom src/mpc_custom.cpp)
add_dependencies(mpc_custom ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_custom ${catkin_LIBRARIES} ipopt Eigen3::Eigen)

add_executable(mpc_node_custom src/mpc_node_custom.cpp)
add_dependencies(mpc_node_custom ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_node_custom ${catkin_LIBRARIES} ipopt Eigen3::Eigen)

add_executable(ipopt_custom src/ipopt.cpp)
add_dependencies(ipopt_custom ${catkin_EXPORTED_TARGETS})
target_link_libraries(ipopt_custom ${catkin_LIBRARIES} ipopt Eigen3::Eigen)


install(TARGETS
    #list of nodes
    mpc_custom
    mpc_node_custom
    ipopt_custom
    
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)