cmake_minimum_required(VERSION 3.9)

project(NavGen CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(NavGen SHARED
  src/dllmain.cpp
  src/bvhbuilder.cpp
 "src/dllmain (2).cpp")

add_subdirectory(lib/bvh)
add_subdirectory(lib/recastnavigation/Recast)
add_subdirectory(lib/recastnavigation/Detour)

target_link_libraries(NavGen PUBLIC bvh)
target_link_libraries(NavGen PUBLIC Recast)
target_link_libraries(NavGen PUBLIC Detour)

