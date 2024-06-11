# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src"
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src-build"
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix"
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/tmp"
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp"
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/src"
  "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src-stamp${cfgdir}") # cfgdir has leading slash
endif()
