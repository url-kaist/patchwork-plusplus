cmake_minimum_required(VERSION 3.16...3.26)
project(patchworkpp VERSION 1.0.4 LANGUAGES CXX)

set(ignore ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Parameters used in `patchworkpp` subdirectory.
# Thus, link should be `patchworkpp::ground_seg_cores`
# See https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp/CMakeLists.txt#L21
set(PARENT_PROJECT_NAME patchworkpp)
set(TARGET_NAME ground_seg_cores)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp ${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_cpp)
else()
  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Performing out-of-tree build, fetching Patchwork++ v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  # Currently, out-of-source build is not supported.
  # But, it will be updated soon!
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS 2 node. gseg: Abbrev. of `g`round `seg`mentation
add_library(gseg_component SHARED src/GroundSegmentationServer.cpp)
target_compile_features(gseg_component PUBLIC cxx_std_20)
target_include_directories(gseg_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gseg_component ${PARENT_PROJECT_NAME}::${TARGET_NAME})
ament_target_dependencies(
  gseg_component
  rcutils
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros)

rclcpp_components_register_node(gseg_component PLUGIN "patchworkpp_ros::GroundSegmentationServer" EXECUTABLE patchworkpp_node)

install(TARGETS gseg_component LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
