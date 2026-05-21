project(patchwork_classic_src)

include(GNUInstallDirs)

set(CLASSIC_TARGET ground_seg_classic)

add_library(${CLASSIC_TARGET} STATIC src/patchwork.cpp)
set_target_properties(${CLASSIC_TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${CLASSIC_TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${CLASSIC_TARGET} Eigen3::Eigen ground_seg_common ground_seg_cores)
if (TBB_FOUND)
  target_link_libraries(${CLASSIC_TARGET} TBB::tbb)
  target_compile_definitions(${CLASSIC_TARGET} PUBLIC PATCHWORK_HAS_TBB)
endif()
add_library(${PARENT_PROJECT_NAME}::${CLASSIC_TARGET} ALIAS ${CLASSIC_TARGET})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS ${CLASSIC_TARGET}
  EXPORT ${PARENT_PROJECT_NAME}Config
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

option(BUILD_PATCHWORK_TESTS "Build C++ smoke test for PatchWork" OFF)
if (BUILD_PATCHWORK_TESTS)
  add_executable(patchwork_smoke tests/smoke_test.cpp)
  target_link_libraries(patchwork_smoke PRIVATE ${CLASSIC_TARGET})
endif()
