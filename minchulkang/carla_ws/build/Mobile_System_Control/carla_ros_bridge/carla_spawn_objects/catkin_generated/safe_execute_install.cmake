execute_process(COMMAND "/home/minchul/carla_ws/build/Mobile_System_Control/carla_ros_bridge/carla_spawn_objects/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/minchul/carla_ws/build/Mobile_System_Control/carla_ros_bridge/carla_spawn_objects/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
