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
    ControlMode.msg
)

#Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
	LIBRARIES rtimulib_ros
	CATKIN_DEPENDS message_runtime std_msgs
)



include_directories(include ${catkin_INCLUDE_DIRS})
#Add all executables
add_executable(motor_i2c_driver src/motor_i2c_driver.cpp src/I2C.cpp src/PCA9685.cpp)
add_executable(rc_i2c_driver src/rc_i2c_driver.cpp src/I2C.cpp)
add_executable(attitude_controller src/attitude_controller.cpp)
add_executable(position_controller src/position_controller.cpp)
add_executable(control_allocation src/control_allocation.cpp)
add_executable(sonar_driver src/sonar_driver.cpp src/I2C.cpp)
add_executable(motor_tester src/motor_tester.cpp)
add_executable(print_euler src/print_euler.cpp)

add_dependencies(motor_i2c_driver ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(rc_i2c_driver ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(attitude_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(motor_tester ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(control_allocation ${${PROJECT_NAME}_EXPORTED_TARGETS})


target_link_libraries(motor_i2c_driver ${catkin_LIBRARIES})
target_link_libraries(rc_i2c_driver ${catkin_LIBRARIES})
target_link_libraries(attitude_controller ${catkin_LIBRARIES} RTIMULib)
target_link_libraries(position_controller ${catkin_LIBRARIES})
target_link_libraries(control_allocation ${catkin_LIBRARIES})
target_link_libraries(sonar_driver ${catkin_LIBRARIES})
target_link_libraries(motor_tester ${catkin_LIBRARIES})
target_link_libraries(print_euler ${catkin_LIBRARIES})


