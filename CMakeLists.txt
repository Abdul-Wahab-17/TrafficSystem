cmake_minimum_required(VERSION 3.16)
project(TrafficSystem CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Use an installed raylib if there is one, otherwise download and build 5.5.
find_package(raylib 5.0 QUIET)
if(NOT raylib_FOUND)
  include(FetchContent)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
    URL_HASH SHA256=aea98ecf5bc5c5e0b789a76de0083a21a70457050ea4cc2aec7566935f5e258e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(raylib)
endif()

# Simulation logic (no window); linked by both the game and the tests
add_library(traffic_core src/graph.cpp src/vehicle.cpp)
target_include_directories(traffic_core PUBLIC include)
target_link_libraries(traffic_core PUBLIC raylib)

add_executable(traffic_system src/main.cpp)
target_link_libraries(traffic_system PRIVATE traffic_core)

enable_testing()
add_executable(test_core tests/test_core.cpp)
target_link_libraries(test_core PRIVATE traffic_core)
add_test(NAME core COMMAND test_core)

foreach(target traffic_core traffic_system test_core)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra)
  endif()
endforeach()
