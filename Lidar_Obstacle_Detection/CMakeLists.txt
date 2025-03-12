cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(playback)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LUIBRARY_DIRS})
add_definitions(${PCL_DEFNITIONS})

add_executable(environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries(environment ${PCL_LIBRARIES} stdc++fs)




