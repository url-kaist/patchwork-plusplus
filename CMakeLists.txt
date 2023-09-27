cmake_minimum_required(VERSION 3.5)
project(patchworkpp)
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DTARGET_ROS2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Boost 1.54 REQUIRED)

set(INCLUDE_DIRS include 
${ament_cmake_INCLUDE_DIRS} 
${rclcpp_INCLUDE_DIRS}
${pcl_ros_INCLUDE_DIRS} 
${pcl_conversions_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${rclcpp_components_INCLUDE_DIRS})
# Include
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS 
${ament_cmake_LIBRARY_DIRS} 
${rclcpp_LIBRARY_DIRS}
${pcl_ros_LIBRARY_DIRS} 
${pcl_conversions_LIBRARY_DIRS}
${std_msgs_LIBRARY_DIRS}
${sensor_msgs_LIBRARY_DIRS}
${rclcpp_components_LIBRARY_DIRS})
#  Linking
link_directories(${LIBRARY_DIRS})

set(LIBS 
${ament_cmake_LIBRARIES} 
${rclcpp_LIBRARIES} 
${pcl_ros_LIBRARIES} 
${pcl_conversions_LIBRARIES} 
${std_msgs_LIBRARIES} 
${sensor_msgs_LIBRARIES}
${rclcpp_components_LIBRARIES})


add_executable(demo src/demo.cpp)
target_link_libraries(demo ${LIBS} ${PCL_LIBRARIES} )

add_library(patchworkpp SHARED src/patchworkpp.cpp)
target_compile_definitions(patchworkpp PUBLIC "PATCHWORKPP_COMPOSITION_BUILDING_DLL")
target_link_libraries(patchworkpp ${LIBS})

install(TARGETS
patchworkpp
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

# ==== Install ====
install(TARGETS demo
ARCHIVE
LIBRARY
RUNTIME
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_package()
