cmake_minimum_required(VERSION 2.8)
#set (CMAKE_CXX_STANDARD 11)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

project( project )

# find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
find_package(Eigen3  REQUIRED)
find_package(PCL  REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS}   ${EIGEN3_INCLUDE_DIR}  ${PCL_INCLUDE_DIRS}    ${CMAKE_SOURCE_DIR} )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

add_executable( main main.cpp camera.h mvs-make_data.h)
target_link_libraries( main  ${OpenCV_LIBRARIES}   ${PCL_LIBRARIES} )

# add_executable( refine refine.cpp make_depth.h other.h camera.h image.h)
# target_link_libraries( refine  ${Open3D_LIBRARIES}  ${OpenCV_LIBRARIES}   ${PCL_LIBRARIES} )
