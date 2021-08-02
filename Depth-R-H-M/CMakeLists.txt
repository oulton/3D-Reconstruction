cmake_minimum_required(VERSION 2.8) 
project( Depth_R_H_M )

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV  REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(PCL  REQUIRED)
 # 添加头文件 
include_directories(  ${OpenCV_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}  )
include_directories(  ./include )

# 语法：add_executable( 程序名 源代码文件 ） 
add_executable( Depth_R_H_M src/Depth_R_H_M.cpp )
target_link_libraries( Depth_R_H_M ${OpenCV_LIBS} ${PCL_LIBRARIES})
