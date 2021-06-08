cmake_minimum_required(VERSION 2.8) 
# project( displayimage )
project( pcd2mesh )

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV  REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(PCL  REQUIRED)
 # 添加头文件 
include_directories(${OpenCV_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}  )


# 语法：add_executable( 程序名 源代码文件 ） 
add_executable( displayimage displayimage.cpp )
target_link_libraries( displayimage ${OpenCV_LIBS} ${PCL_LIBRARIES})

