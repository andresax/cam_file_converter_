
cmake_minimum_required(VERSION 3.5)

project(converter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(
    eigen/
)

set(ref_MBC_SOURCE 
                  dtu_to_colmap.cpp
                  )

add_executable(converter ${ref_MBC_SOURCE})
