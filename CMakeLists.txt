cmake_minimum_required(VERSION 3.14)
project(ground_removal)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ground_removal/include)
set(GROUND_REMOVAL_SRC ground_removal/src/ground_removal.cpp)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(visualize)
