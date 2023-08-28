cmake_minimum_required(VERSION 3.8)
project(quadrotor_simulator_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
	${PROJECT_NAME}/quadrotor_simulator_node
	${PROJECT_NAME}/vehicle_rviz_node
	DESTINATION lib/${PROJECT_NAME})

# Install launch and config files.
install(DIRECTORY
  config
  assets
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
