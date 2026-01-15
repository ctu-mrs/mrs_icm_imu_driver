cmake_minimum_required(VERSION 3.20.0)
project(mrs_icm_imu_driver)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_icm_imu_driver]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

SET(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  mrs_lib
)

SET(LIBRARIES
  MrsIcmImuDriver_ImuDriver
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

add_library(MrsIcmImuDriver_ImuDriver SHARED
  src/icm_imu_driver.cpp
)

target_link_libraries(MrsIcmImuDriver_ImuDriver
  PRIVATE
    mrs_lib::mrs_lib
    rclcpp::rclcpp
    rclcpp_components::component
)

rclcpp_components_register_nodes(MrsIcmImuDriver_ImuDriver "mrs_icm_imu_driver::MrsIcmImuDriver")

target_compile_definitions(MrsIcmImuDriver_ImuDriver PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS
  ${LIBRARIES} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
