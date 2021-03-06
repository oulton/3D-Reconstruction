
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile("../mesh/mesh.ply", *cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud( *cloud, *cloud, indices);
    // PCLcloudVisualizer( cloud );

    float max_distance = 0;
    Eigen::Vector3f point_min( 0, 0, -10 ); //获取距离原点最近的point

    for (int i = 0; i < cloud->points.size(); i++)
    {
        Eigen::Vector3f point_temp( cloud->points[i].x,  cloud->points[i].y , cloud->points[i].z);
        float distance_ = sqrt(pow(point_temp[0], 2) + pow(point_temp[1], 2) + pow(point_temp[2], 2));
        max_distance = max_distance>distance_ ? max_distance:distance_;

        point_min[1] = point_min[1]>point_temp[1] ? point_temp[1]:point_min[1];
        point_min[2] = point_min[2]<point_temp[2] ? point_temp[2]:point_min[2];
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_8level ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud_8level ( new pcl::PointCloud<pcl::PointXYZRGB> );

    pcl::PointCloud<pcl::PointXYZRGB> levels[8];
    pcl::PointCloud<pcl::PointXYZRGB> filtered_levels[8];

    for (int i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZRGB P = cloud->points[i];
        float distance_ = sqrt(pow(P.x, 2) + pow(P.y - point_min[1], 2) + pow(P.z - point_min[2], 2));
   
        for (int j = 0; j < 8; j++)
        {
            if(abs(distance_ - max_distance*j/8) < THRESHOLD)
            {
                P.b = 0;
                P.g = 0;
                P.r = 255;
            }
        }
        cloud_8level->push_back(P);

        if(distance_ <= max_distance/8)
            levels[0].push_back(P);
        if(distance_ > max_distance/8 && distance_ <= max_distance*2/8)
            levels[1].push_back(P);
        if(distance_ > max_distance*2/8 && distance_ <= max_distance*3/8)
            levels[2].push_back(P);
        if(distance_ > max_distance*3/8 && distance_ <= max_distance*4/8)
            levels[3].push_back(P);
        if(distance_ > max_distance*4/8 && distance_ <= max_distance*5/8)
            levels[4].push_back(P);
        if(distance_ > max_distance*5/8 && distance_ <= max_distance*6/8)
            levels[5].push_back(P);
        if(distance_ > max_distance*6/8 && distance_ <= max_distance*7/8)
            levels[6].push_back(P);
        if(distance_ > max_distance*7/8 && distance_ <= max_distance)
            levels[7].push_back(P);
    }

    //将8级，分别进行不同程度的下采样
    for (int i = 0; i < 8; i++)
    {
        if(i==0)
            filtered_levels[i] = levels[i];
        else
        {
            // 创建滤波对象
            pcl::VoxelGrid<pcl::PointXYZRGB> filter;
            filter.setInputCloud(levels[i].makeShared());
            // 设置体素栅格的大小为 1x1x1cm * i * FILTER_SCALE
            if (i<=4)
                filter.setLeafSize(i * FILTER_SCALE * 0.01f, i * FILTER_SCALE * 0.01f, i * FILTER_SCALE * 0.01f);
            else if(i==5)
                filter.setLeafSize(i * FILTER_SCALE * 0.02f, i * FILTER_SCALE * 0.02f, i * FILTER_SCALE * 0.02f);
            else
                filter.setLeafSize(i * FILTER_SCALE * 0.03f, i * FILTER_SCALE * 0.03f, i * FILTER_SCALE * 0.03f);
            filter.filter(filtered_levels[i]);
        }

        for (int j = 0; j < filtered_levels[i].points.size(); j++)
            filtered_cloud_8level->push_back(filtered_levels[i].points[j]);
        
    }

    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*cloud_8level, mesh.cloud  );
    pcl::io::savePLYFile( "../mesh/dis_8level.ply", mesh );
    std::cout << "saved  cloud_8level" << std::endl;

    pcl::PolygonMesh filtered_mesh;
    pcl::toPCLPointCloud2(*filtered_cloud_8level, filtered_mesh.cloud  );
    pcl::io::savePLYFile( "../mesh/dis_filtered_8level.ply", filtered_mesh );
    std::cout << "saved  filtered_cloud_8level" << std::endl;

    return 0;
}