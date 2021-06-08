
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
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/filter.h>

using namespace std;

float fx= 508.1586608886719 ;
float fy= 508.1586608886719;
float cx= 322.898193359375 ;
float cy= 250.3104248046875;



void PCLcloudVisualizer( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ){

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("test"));//创建可视化窗口，名字叫做`test
    //viewer->setCameraPosition(1,1,1,1,1,1); //设置坐标原点
    viewer->initCameraParameters();   //初始化相机参数
    viewer->setBackgroundColor (255, 255, 255);    //设置视口的背景颜色
    viewer->addText("Radius: source", 10, 10, "v1 text");  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color( cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> ( cloud, single_color, "sample cloud ");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"sample cloud ");
    viewer->addCoordinateSystem ( 0.1);//显示坐标系统方向

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);//显示
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

cv::Mat CreatDepthFromCloud( int width, int height,  Eigen::Matrix3f intrinsic, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){

    cv::Mat depth=cv::Mat::zeros( height, width, CV_16UC1 );
    for(int i=0;i<cloud->points.size(); i++ ){

        Eigen::Vector3f v( cloud->points[i].x,  cloud->points[i].y , cloud->points[i].z);
        std::cout<<v<<std::endl;
        if(v[0]==0&&v[1]==0&&v[2]==0)
            continue;;
        Eigen::Vector3f uv=intrinsic*v;
        int u_=round( uv[0]/uv[2] );
        int v_=round( uv[1]/uv[2] );
        int d= round( uv[2]*1000 );
        if(u_>=width||u_<0||v_>=height||v_<0)
            continue;
        depth.ptr<ushort>(v_)[u_]=d;
    }
    return depth;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CreatCloudFromDepth(  Eigen::Matrix3f intrinsic, cv::Mat depth ){

    float fx = intrinsic(0, 0);
    float fy = intrinsic(1, 1);
    float cx = intrinsic(0, 2);
    float cy = intrinsic(1, 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<depth.rows;i++){

        for(int j=0;j<depth.cols;j++){

            float d = depth.ptr<ushort>(i)[j];
            if (d == 0)
                continue;
            float z = d / 1000;
            float x = (i- cx) * z / fx;
            float y = (j - cy) * z / fy;
            pcl::PointXYZ point;
            point.x=x; point.y=y; point.z=z;
            cloud->points.push_back(point);
        }
    }
    cloud->width=1;
    cloud->height = cloud->points.size() ;
    return cloud;
}


int main( )
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcd/1621934856.899079000.pcd",  *cloud );
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud( *cloud, *cloud, indices);
    //PCLcloudVisualizer( cloud );

    Eigen::Matrix3f intrinsic;
    intrinsic<<  fx,0,cx,
                  0,fy,cy,
                  0,0,1;

    cv::Mat depth = CreatDepthFromCloud( 640 , 480, intrinsic, cloud );

    //cv::imshow( "depth" , depth);
    // cv::Mat depth_color;
    // depth.convertTo( depth_color,  CV_8UC1 );
    cv::imwrite("save.png",depth);
    //cv::imshow( "depth_COLOR" , depth_color);
    //cv::waitKey(0);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    // cloud_=CreatCloudFromDepth(  intrinsic, depth );
    // PCLcloudVisualizer( cloud_ );

    return 0;
}