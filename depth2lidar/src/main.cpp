
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#define THRESHOLD 0.02
#define FILTER_SCALE 3


Eigen::Vector3f cam2point( int m, int n, Eigen::Matrix3f  K_x_ )
{
    return Eigen::Vector3f(
            // ( n - cx ) / fx ,
            // ( m - cy ) / fy ,
            //         1
            ( n - K_x_(0,2) ) / K_x_(0,0) ,
            ( m - K_x_(1,2) ) / K_x_(1,1) ,
                    1
            );
}

void PCLcloudVisualizer( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud ){

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("test"));//创建可视化窗口，名字叫做`test
    //viewer->setCameraPosition(1,1,1,1,1,1); //设置坐标原点
    viewer->initCameraParameters();   //初始化相机参数
    viewer->setBackgroundColor (255, 255, 255);    //设置视口的背景颜色
    viewer->addText("Radius: source", 10, 10, "v1 text");  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color( cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> ( cloud, single_color, "sample cloud ");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"sample cloud ");
    viewer->addCoordinateSystem ( 0.1);//显示坐标系统方向

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);//显示
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


int main( )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcds/pcd-lidar.pcd", *cloud_lidar);
    std::vector<int> indices_lidar;
    pcl::removeNaNFromPointCloud( *cloud_lidar, *cloud_lidar, indices_lidar);
    Eigen::Matrix4f T_lidar_color;  
    T_lidar_color  <<   4.9167826375697443e-01, -2.8196337036073216e-02, -8.7032031547394939e-01, 1.4270700514316559e-02,
                        8.7067594755675448e-01, 6.9699067031483986e-04, 4.9185659348040989e-01, -2.0799441263079643e-02,
                        -1.3261949143117568e-02, -9.9960216125304246e-01, 2.4892567629147849e-02, -1.3210496306419373e-01, 
                        0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4f T_color_depth; 
    T_color_depth  <<   0.9999839067459106, -0.005434882361441851, 0.0016148093855008483, 0.014913409948348999,
                        0.005436081439256668, 0.9999849796295166, -0.0007390331011265516, -7.743491005385295e-05,
                        -0.0016107684932649136, 0.0007477994658984244, 0.9999984502792358, -0.00016766598855610937,
                        0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3f CameraDepth;
    CameraDepth <<  384.6130676269531, 0.0, 319.5439758300781, 
                    0.0, 384.6130676269531, 240.34002685546875, 
                    0.0, 0.0, 1.0;
    
    Eigen::Matrix4f T_lidar_depth = T_lidar_color * T_color_depth;

    cv::Mat depth = cv::imread("../pcds/depth.png", -1);
    for(int m=0;m<depth.rows;m++)
    {
        for(int n=0;n<depth.cols;n++)
        {
            double d=depth.ptr<ushort>(m)[n];
            if(d<=0 || d>3000)
                continue;
            Eigen::Vector3f depthFramPoint = cam2point( m, n, CameraDepth );
            depthFramPoint = depthFramPoint * ( d / 1000.0 );

            Eigen::Vector3f lidarFramPoint = T_lidar_depth.block(0,0,3,3) * depthFramPoint + T_lidar_depth.block(0,3,3,1);

            pcl::PointXYZ temp;
            temp.x = lidarFramPoint[0];
            temp.y = lidarFramPoint[1];
            temp.z = lidarFramPoint[2];

            cloud_lidar->push_back(temp);
        }
    }

    pcl::io::savePCDFileBinary("../pcds/1_save.pcd", *cloud_lidar);

    return 0;
}