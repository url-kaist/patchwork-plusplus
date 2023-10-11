project(patchworkpp_python_wrapper LANGUAGES CXX)

pybind11_add_module(pypatchworkpp pybinding.cpp)

target_link_libraries(pypatchworkpp PUBLIC PATCHWORK::patchworkpp)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(pypatchworkpp PUBLIC -fsized-deallocation)
endif()

install(TARGETS pypatchworkpp DESTINATION .)
