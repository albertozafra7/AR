execute_process(COMMAND "/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/geodesy/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/geodesy/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
