cmake_minimum_required(VERSION 3.0.2)
project(visual_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  SetAbsolutePose.srv
  SetInitialPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    message_runtime
    eigen_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(visual_localization_node src/visual_localization_node.cpp)
target_link_libraries(visual_localization_node ${catkin_LIBRARIES})
add_dependencies(visual_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS visual_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
