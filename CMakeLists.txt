cmake_minimum_required(VERSION 2.8.3)
project(omnicopter_ros)

add_definitions("-g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
)

add_message_files(
	FILES
    RCInput.msg
    MotorCommand.msg
)

#Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
	LIBRARIES rtimulib_ros
	CATKIN_DEPENDS message_runtime std_msgs
)



include_directories(include ${catkin_INCLUDE_DIRS})
#Add all executables
add_executable(commander src/commander.cpp)
add_executable(motor_i2c_driver src/motor_i2c_driver.cpp src/I2C.cpp src/PCA9685.cpp)
add_executable(rc_i2c_driver src/rc_i2c_driver.cpp)
add_executable(attitude_controller src/attitude_controller.cpp)
add_executable(position_controller src/position_controller.cpp)
add_executable(control_allocation src/control_allocation.cpp)
add_executable(rtimulib_ros src/rtimulib_ros.cpp)

add_dependencies(motor_i2c_driver ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(rc_i2c_driver ${${PROJECT_NAME}_EXPORTED_TARGETS})


target_link_libraries(commander ${catkin_LIBRARIES})
target_link_libraries(motor_i2c_driver ${catkin_LIBRARIES})
target_link_libraries(rc_i2c_driver ${catkin_LIBRARIES})
target_link_libraries(attitude_controller ${catkin_LIBRARIES})
target_link_libraries(position_controller ${catkin_LIBRARIES})
target_link_libraries(control_allocation ${catkin_LIBRARIES})
target_link_libraries(rtimulib_ros ${catkin_LIBRARIES} RTIMULib)


