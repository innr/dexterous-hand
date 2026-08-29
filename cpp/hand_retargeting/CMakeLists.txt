cmake_minimum_required(VERSION 3.16)

project(dexterous_hand_retargeting LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(dexterous_hand_retargeting
  src/retargeter.cpp
  src/mapping.cpp
  src/virtual_backend.cpp
)

target_include_directories(dexterous_hand_retargeting
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(dexterous_hand_retargeting
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
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
endif()
