//
// Created by wy on 2021/8/1.
//

#ifndef PROJECT_COLORMAP_H
#define PROJECT_COLORMAP_H

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include "vtk_expand.h"


void PCLcloudVisualizer( pcl::PolygonMesh& mesh  ){

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("test"));
    viewer->setBackgroundColor(255,255,255);//设置窗口颜色
    viewer->initCameraParameters();
    viewer->setShowFPS(false);
    viewer->addPolygonMesh(mesh,"mesh");//设置所要显示的网格对象
    viewer->setRepresentationToSurfaceForAllActors();//网格模型以片面形式显示
    //viewer->setRepresentationToPointsForAllActors();//网格模型以点形式显示
    //viewer->setRepresentationToWireframeForAllActors();//网格模型以线框图形式显示
    //viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);//显示
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}



void colormap(std::string filename, std::string save_path,
              cv::Mat color,
              Eigen::Matrix4d extrinsic,
              Eigen::Matrix3d intrinsic) {

    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile ( filename, mesh );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2( mesh.cloud, *cloud );

    Eigen::Matrix4d extrinsic_=extrinsic;
    Eigen::Matrix3d intrinsic_=intrinsic;
    int w=color.cols, h=color.rows;

    for(int i=0;i<cloud->points.size();i++ )
    {
        Eigen::Vector4d xyz( cloud->points[i].x, cloud->points[i].y , cloud->points[i].z,1 );
        Eigen::Vector4d p=extrinsic_.inverse()*xyz;
        Eigen::Vector3d uv= intrinsic_*p.head(3);
        int u=uv[0]/uv[2];
        int v=uv[1]/uv[2];
        if( u<0||u>=w||v<0||v>=h  )
            continue;

        if(color.ptr<cv::Vec3b>(v)[u][0] == 0 && color.ptr<cv::Vec3b>(v)[u][1] == 0 && color.ptr<cv::Vec3b>(v)[u][2] == 0)
            continue;
        else
        {
            cloud->points[i].r=color.ptr<cv::Vec3b>(v)[u][2];
            cloud->points[i].g=color.ptr<cv::Vec3b>(v)[u][1];
            cloud->points[i].b=color.ptr<cv::Vec3b>(v)[u][0];
        }
    }

    pcl::toPCLPointCloud2(*cloud, mesh.cloud  );

//    PCLcloudVisualizer( mesh );
    pcl::io::savePLYFile( save_path, mesh );

    std::cout << "ply saved" <<std::endl;

}


#endif //PROJECT_COLORMAP_H
