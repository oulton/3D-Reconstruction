cmake_minimum_required(VERSION 2.8)
project(project)

SET( CMAKE_CXX_FLAGS "-std=c++14 -O3")
find_package(Eigen3  REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

INCLUDE(${VTK_USE_FILE})
include_directories(  ${OpenCV_INCLUDE_DIRS}   ${EIGEN3_INCLUDE_DIR}  ${PCL_INCLUDE_DIRS})

add_executable(main src/main.cpp  )
target_link_libraries( main  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} )