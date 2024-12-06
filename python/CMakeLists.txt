cmake_minimum_required(VERSION 3.16...3.26)
project(patchworkpp_python_wrapper LANGUAGES CXX)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Parameters used in `patchworkpp` subdirectory.
# Thus, link should be `patchworkpp::ground_seg_cores`
# See https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp/CMakeLists.txt#L21
set(PARENT_PROJECT_NAME patchworkpp)
set(TARGET_NAME ground_seg_cores)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp ${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_cpp)
else()
  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Performing out-of-tree build, fetching Patchwork++ v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  FetchContent_Declare(
    ext_ground_seg_cores PREFIX ${PARENT_PROJECT_NAME}
    URL https://github.com/url-kaist/patchwork-plusplus/archive/refs/tags/v1.0.tar.gz SOURCE_SUBDIR
        cpp/patchworkpp)
  FetchContent_MakeAvailable(ext_ground_seg_cores)
endif()

pybind11_add_module(pypatchworkpp patchworkpp/pybinding.cpp)

target_link_libraries(pypatchworkpp PUBLIC ${PARENT_PROJECT_NAME}::${TARGET_NAME})

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(pypatchworkpp PUBLIC -fsized-deallocation)
endif()

install(TARGETS pypatchworkpp DESTINATION .)
