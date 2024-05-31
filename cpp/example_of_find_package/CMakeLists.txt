cmake_minimum_required(VERSION 3.11)
project(example_of_find_package VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(PYTHON_EXECUTABLE python3)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(patchworkpp REQUIRED)

if(CMAKE_VERSION VERSION_LESS "3.15")
# Just automatically update cmake version
execute_process(COMMAND bash ../scripts/install_latest.cmake.bash
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

include_directories(${patchworkpp_INCLUDE_DIRS})

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

add_executable(demo_visualize ${CMAKE_CURRENT_SOURCE_DIR}/demo_visualize_copied.cpp)
# Note that `patchworkpp::ground_seg_cores` is aliased as `ground_seg_cores` when installing `patchworkpp` package
target_link_libraries(demo_visualize patchworkpp::ground_seg_cores ${Open3D_LIBRARIES} "stdc++fs")
target_include_directories(demo_visualize PUBLIC ${Open3D_INCLUDE_DIRS})

