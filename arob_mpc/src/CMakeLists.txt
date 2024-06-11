cmake_minimum_required(VERSION 3.0.2)
project(mpc)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg  geometry_msgs message_generation)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
)

# ## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs #   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  INCLUDE_DIRS include
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_node_custom src/mpc/mpc_node_custom.cpp)
add_dependencies(mpc_node_custom ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_node_custom ${catkin_LIBRARIES} ipopt Eigen3::Eigen)



install(TARGETS
    #list of nodes
    mpc_node_custom
    
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

