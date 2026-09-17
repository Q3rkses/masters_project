cmake_minimum_required(VERSION 3.8)
project(kalman_filtering_and_smoothing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)

add_library(kalman_lib
  src/models.cpp
  src/kalman_filter.cpp
  src/rauch_tung_striebel_smoother.cpp
  src/simulation.cpp
  src/results.cpp
)

include_directories(lib)
target_include_directories(kalman_lib PUBLIC lib)
target_link_libraries(kalman_lib PUBLIC Eigen3::Eigen)

add_executable(kalman_filtering_and_smoothing
  src/main.cpp
)
target_link_libraries(kalman_filtering_and_smoothing PRIVATE kalman_lib)
target_compile_definitions(kalman_filtering_and_smoothing PRIVATE
  PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)
