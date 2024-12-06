cmake_minimum_required(VERSION 3.11)
project(patchworkpp VERSION 1.0.2)

option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" OFF)
option(INCLUDE_CPP_EXAMPLES "Include C++ example codes, which require Open3D for visualization" OFF)

set(CMAKE_CXX_STANDARD 20)
set(PYTHON_EXECUTABLE python3)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

if(CMAKE_VERSION VERSION_LESS 3.15)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ListPrepend.cmake")
  list_prepend(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
else()
  list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif()

if(CMAKE_VERSION VERSION_GREATER 3.24)
  cmake_policy(SET CMP0135 OLD)
endif()

# Originally, below code was from `find_dependencies.cmake`
# The below code is to make dependency of Eigen free
function(find_external_dependency PACKAGE_NAME TARGET_NAME INCLUDED_CMAKE_PATH)
  string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UP)
  set(USE_FROM_SYSTEM_OPTION "USE_SYSTEM_${PACKAGE_NAME_UP}")
  if(${${USE_FROM_SYSTEM_OPTION}})
    find_package(${PACKAGE_NAME} QUIET NO_MODULE)
  endif()
  if(NOT ${${USE_FROM_SYSTEM_OPTION}} OR NOT TARGET ${TARGET_NAME})
    set(${USE_FROM_SYSTEM_OPTION} OFF PARENT_SCOPE)
    include(${INCLUDED_CMAKE_PATH})
  endif()
endfunction()

find_external_dependency("Eigen3" "Eigen3::Eigen" "${CMAKE_CURRENT_LIST_DIR}/cmake/eigen.cmake")

# Parameters in `patchworkpp` subdirectory.
# Thus, link should be `patchworkpp::ground_seg_cores`
set(PARENT_PROJECT_NAME ${PROJECT_NAME})
set(TARGET_NAME ground_seg_cores)

add_subdirectory(patchworkpp)

if (INCLUDE_CPP_EXAMPLES)
  if(CMAKE_VERSION VERSION_LESS "3.15")
    # Just automatically update cmake version
    execute_process(COMMAND bash ../scripts/install_latest.cmake.bash
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  list(APPEND Open3D_LIBRARIES dl)

  message(STATUS "Building examples for c++")
  find_package(Open3D QUIET)
  if (NOT Open3D_FOUND)
    message(STATUS "Open3D not found, installing Open3D...")
    execute_process(COMMAND bash ../scripts/install_open3d.bash
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(Open3D REQUIRED HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
  else()
    message(STATUS "Found Open3D ${Open3D_VERSION}")
  endif()

  list(APPEND Open3D_LIBRARIES dl)
  link_directories(${Open3D_LIBRARY_DIRS})
  message(STATUS "Found Open3D ${Open3D_VERSION}")

  add_executable(demo_visualize ${CMAKE_CURRENT_SOURCE_DIR}/patchworkpp/examples/demo_visualize.cpp)
  target_link_libraries(demo_visualize PRIVATE ${PARENT_PROJECT_NAME}::${TARGET_NAME} ${Open3D_LIBRARIES} "stdc++fs")
  target_include_directories(demo_visualize PUBLIC ${Open3D_INCLUDE_DIRS})
  set_target_properties(demo_visualize PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples")

  add_executable(demo_sequential ${CMAKE_CURRENT_SOURCE_DIR}/patchworkpp/examples/demo_sequential.cpp)
  target_link_libraries(demo_sequential PRIVATE ${PARENT_PROJECT_NAME}::${TARGET_NAME} ${Open3D_LIBRARIES} "stdc++fs")
  target_include_directories(demo_sequential PUBLIC ${Open3D_INCLUDE_DIRS})
  set_target_properties(demo_sequential PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples")
endif()
