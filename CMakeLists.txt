cmake_minimum_required(VERSION 3.5)
project(fog_bumper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(fog_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED COMPONENTS)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

# include include
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                           BUMPER                           |
## --------------------------------------------------------------

add_library(Bumper SHARED
  src/bumper_nodelet.cpp
  )

ament_target_dependencies(Bumper
  PUBLIC
  rclcpp
  PCL
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_msgs
  fog_msgs
  )

target_link_libraries(Bumper ${Eigen_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

rclcpp_components_register_node(Bumper
  PLUGIN "fog_bumper::Bumper"
  EXECUTABLE bumper)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS fog_bumper
  EXPORT fog_bumper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
