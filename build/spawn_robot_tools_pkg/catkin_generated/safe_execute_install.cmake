execute_process(COMMAND "/home/hcappel1/Multi_Agent_Systems/build/spawn_robot_tools_pkg/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hcappel1/Multi_Agent_Systems/build/spawn_robot_tools_pkg/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
