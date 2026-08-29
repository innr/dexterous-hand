cmake_minimum_required(VERSION 3.16)

project(dexterous_hand_retargeting VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(dexterous_hand_retargeting
  src/retargeter.cpp
  src/mapping.cpp
  src/virtual_backend.cpp
)
add_library(dexterous_hand::retargeting ALIAS dexterous_hand_retargeting)

target_include_directories(dexterous_hand_retargeting
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_options(dexterous_hand_retargeting PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS dexterous_hand_retargeting
  EXPORT dexterous_hand_retargetingTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT dexterous_hand_retargetingTargets
  FILE dexterous_hand_retargetingTargets.cmake
  NAMESPACE dexterous_hand::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dexterous_hand_retargeting
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dexterous_hand_retargetingConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dexterous_hand_retargetingConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dexterous_hand_retargeting
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/dexterous_hand_retargetingConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dexterous_hand_retargetingConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/dexterous_hand_retargetingConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dexterous_hand_retargeting
)

option(BUILD_TESTING "Build C++ unit tests" ON)
option(BUILD_EXAMPLES "Build the offline hardware pipeline demo" ON)

if(BUILD_TESTING)
  include(CTest)
  enable_testing()
  add_executable(test_retargeting tests/test_retargeting.cpp)
  target_link_libraries(test_retargeting PRIVATE dexterous_hand_retargeting)
  target_compile_options(test_retargeting PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_retargeting COMMAND test_retargeting)
endif()

if(BUILD_EXAMPLES)
  add_executable(retarget_demo examples/retarget_demo.cpp)
  target_link_libraries(retarget_demo PRIVATE dexterous_hand_retargeting)
  target_compile_options(retarget_demo PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS retarget_demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
