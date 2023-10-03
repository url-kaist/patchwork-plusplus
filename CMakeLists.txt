cmake_minimum_required(VERSION 3.19)
project(PATCHWORK VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)
set(PYTHON_EXECUTABLE python3)
set(CMAKE_BUILD_TYPE Release)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(FetchContent)

list(APPEND Open3D_LIBRARIES dl)

add_subdirectory(patchworkpp)

if (INCLUDE_PYTHON_WRAPPER)

  message("Building Python wrapper")
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
  include(pybind11)
  add_subdirectory(python_wrapper)

endif()

if (INCLUDE_EXAMPLES)

  message(STATUS "Building examples for c++")
  find_package(Open3D REQUIRED HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)


  list(APPEND Open3D_LIBRARIES dl)
  link_directories(${Open3D_LIBRARY_DIRS})
  message(STATUS "Found Open3D ${Open3D_VERSION}")

  add_executable(demo_visualize examples/cpp/demo_visualize.cpp)
  target_link_libraries(demo_visualize PRIVATE PATCHWORK::patchworkpp ${Open3D_LIBRARIES} "stdc++fs")
  target_include_directories(demo_visualize PUBLIC ${Open3D_INCLUDE_DIRS})
  set_target_properties(demo_visualize PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/examples/cpp")

  add_executable(demo_sequential examples/cpp/demo_sequential.cpp)
  target_link_libraries(demo_sequential PRIVATE PATCHWORK::patchworkpp ${Open3D_LIBRARIES} "stdc++fs")
  target_include_directories(demo_sequential PUBLIC ${Open3D_INCLUDE_DIRS})
  set_target_properties(demo_sequential PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/examples/cpp")

endif()
