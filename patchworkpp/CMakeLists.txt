project(patchworkpp_src)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED QUIET)

add_library(patchworkpp STATIC src/patchworkpp.cpp)
set_target_properties(patchworkpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(patchworkpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(patchworkpp Eigen3::Eigen)
add_library(PATCHWORK::patchworkpp ALIAS patchworkpp)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS patchworkpp
  EXPORT PATCHWORKConfig
  LIBRARY DESTINATION lib
)

export(TARGETS patchworkpp
  NAMESPACE PATCHWORK::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/PATCHWORKConfig.cmake"
)
install(EXPORT PATCHWORKConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/PATCHWORK/cmake"
  NAMESPACE PATCHWORK::
)