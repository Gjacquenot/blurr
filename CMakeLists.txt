cmake_minimum_required(VERSION 3.5)
project(blurr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navio2_ros REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(pwm src/pwm.cpp)
ament_target_dependencies(pwm rclcpp sensor_msgs std_msgs navio2_ros class_loader)
    
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()


install(PROGRAMS
  scripts/bar30.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/ms5837-python/ms5837
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pwm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}) 
  
ament_package()
