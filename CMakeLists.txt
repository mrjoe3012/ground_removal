cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 11)

project(ground_removal_test)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(vis visualize.cpp)
target_link_libraries(vis ${PCL_LIBRARIES})

enable_testing()
add_executable(unit_tests unit_tests.cpp)
target_link_libraries(unit_tests gtest_main ${PCL_LIBRARIES})
include(GoogleTest)
gtest_discover_tests(unit_tests)
