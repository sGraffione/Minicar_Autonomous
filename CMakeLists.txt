cmake_minimum_required(VERSION 3.8)
project(minicar)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  rosidl_default_generators
  Eigen3
  minicar_interfaces
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


include_directories(include)

###### Add executables ######
#add_executable(controller src/control.cpp)
#add_executable(localization src/btsSensors.cpp)
add_executable(motorsManager src/motorsManager.cpp)
ament_target_dependencies(motorsManager rclcpp minicar_interfaces)
#add_executable(accelerometer src/accelerometer.cpp)
#add_executable(kalmanFilter src/EKF.cpp)

## NEED TO BE AFTER THE DEFINITION OF THE EXECUTABLE
find_library(MPU6050_LIB MPU6050 ./include/MPU6050/)

set_target_properties(motorsManager PROPERTIES COMPILE_FLAGS "-pthread -lrt -lpigpio -lbcm2835")

target_link_libraries(motorsManager
  bcm2835
  pigpio
)

install(TARGETS
motorsManager
DESTINATION lib/${PROJECT_NAME})

ament_package()