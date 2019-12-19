
cmake_minimum_required(VERSION 3.7)

project(converter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)

set(ref_MBC_SOURCE 
                  dtu_to_colmap.cpp
                  )

add_executable(converter ${ref_MBC_SOURCE})
