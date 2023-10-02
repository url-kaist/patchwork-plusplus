cmake_minimum_required(VERSION 3.16)


project(pypatchworkpp LANGUAGES CXX)

#find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

#list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
#include(pybind11)

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../patchworkpp patchworkpp)


pybind11_add_module(pypatchworkpp pybinding.cpp)

target_link_libraries(pypatchworkpp PRIVATE patchworkpp)

install(TARGETS pypatchworkpp DESTINATION .)
