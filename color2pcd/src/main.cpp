
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

#if 0
void FuseRGB2PCD( cv::Mat color_img,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix3f intrinsic_color, Eigen::Matrix4f T_depth_color )
{
    // pcl::PointCloud<pcl::PointXYZRGB> P_new;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr P_new ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr red ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr green ( new pcl::PointCloud<pcl::PointXYZRGB> );
    for(int i=0;i<cloud->points.size(); i++ )
    {
        pcl::PointXYZRGB temp;
        Eigen::Vector3f p( cloud->points[i].x,  cloud->points[i].y , cloud->points[i].z);
        temp.x = cloud->points[i].x;
        temp.y = cloud->points[i].y;
        temp.z = cloud->points[i].z;

        Eigen::Vector3f p1=T_depth_color.block(0,0,3,3)*p+T_depth_color.block(0,3,3,1);

        Eigen::Vector3f p2=intrinsic_color*p1;
        int u=p2[0]/p2[2];
        int v=p2[1]/p2[2];
        if( u<0||u>=color_img.cols || v<0||v>=color_img.rows )
            continue;

        if(color_img.ptr<cv::Vec3b>(v)[u][0] == 0 && color_img.ptr<cv::Vec3b>(v)[u][1] == 0 && color_img.ptr<cv::Vec3b>(v)[u][2] == 0)
        {
           temp.r=255;
           temp.g=255;
           temp.b=255;
        }
        else
        {
            temp.r=color_img.ptr<cv::Vec3b>(v)[u][2];
            temp.g=color_img.ptr<cv::Vec3b>(v)[u][1];
            temp.b=color_img.ptr<cv::Vec3b>(v)[u][0];
            if(temp.b == 0 && temp.g == 0 && temp.r == 128)
                red->push_back(temp);
            if(temp.b == 0 && temp.g == 128 && temp.r == 0)
                green->push_back(temp);
        }
        P_new->push_back(temp);
    }

    pcl::PointXYZ red_ave;
    pcl::PointXYZ green_ave;
    red_ave.x=0;red_ave.y=0;red_ave.z=0;
    green_ave.x=0;green_ave.y=0;green_ave.z=0;
    for (int i = 0; i < red->points.size(); ++i) {
        red_ave.x += red->points[i].x;
        red_ave.y += red->points[i].y;
        red_ave.z += red->points[i].z;
    }
    for (int i = 0; i < green->points.size(); ++i) {
        green_ave.x += green->points[i].x;
        green_ave.y += green->points[i].y;
        green_ave.z += green->points[i].z;
    }
    red_ave.x/=red->points.size();red_ave.y/=red->points.size();red_ave.z/=red->points.size();
    green_ave.x/=green->points.size();green_ave.y/=green->points.size();green_ave.z/=green->points.size();
    float distance = sqrt(pow(red_ave.x-green_ave.x, 2) + pow(red_ave.y-green_ave.y, 2) + pow(red_ave.z-green_ave.z, 2));
    cout << red_ave << endl;
    cout << green_ave << endl;
    cout << distance << endl;
    pcl::io::savePCDFile("./pointcloud.pcd", *P_new);
	//清除数据并保存
	cloud->points.clear();
	cout<< "Point cloud saved." << endl;
}
#else
void FuseRGB2PCD( cv::Mat color,  cv::Mat depth, Eigen::Matrix3f intrinsic_color, Eigen::Matrix3f intrinsic_depth, Eigen::Matrix4f T_depth_color )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr red ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr green ( new pcl::PointCloud<pcl::PointXYZRGB> );
    for(int m=0;m<depth.rows;m++)
    {
        for(int n=0;n<depth.cols;n++)
        {
            double d=depth.ptr<ushort>(m)[n];
            if(d<=0 || d>5000)
                continue;
            Eigen::Vector3f depthFramPoint = cam2point( m, n, intrinsic_depth );
            depthFramPoint = depthFramPoint * ( d / 1000.0 );

            pcl::PointXYZRGB P;
            P.x=depthFramPoint[0];
            P.y=depthFramPoint[1];
            P.z=depthFramPoint[2];

            int u, v;
            //color对齐到depth  
            Eigen::Vector3f colorPixPoint = intrinsic_color * (T_depth_color.block(0,0,3,3) * depthFramPoint + T_depth_color.block(0,3,3,1));
            u=colorPixPoint[0]/colorPixPoint[2];
            v=colorPixPoint[1]/colorPixPoint[2];
            if( u<0||u>=color.cols || v<0||v>=color.rows )
                continue;
            if(color.ptr<cv::Vec3b>(v)[u][0] == 0 && color.ptr<cv::Vec3b>(v)[u][1] == 0 && color.ptr<cv::Vec3b>(v)[u][2] == 0)
            {
                P.r=255;
                P.g=255;
                P.b=255;
            }
            else
            {
                P.r=color.ptr<cv::Vec3b>(v)[u][2];
                P.g=color.ptr<cv::Vec3b>(v)[u][1];
                P.b=color.ptr<cv::Vec3b>(v)[u][0];
                if(P.b == 0 && P.g == 128 && P.r == 0)
                    red->push_back(P);
                if(P.b == 0 && P.g == 128 && P.r == 128)
                    green->push_back(P);
            }
            cloud->push_back(P);
		}
	}

    pcl::PointXYZ red_ave;
    pcl::PointXYZ green_ave;
    red_ave.x=0;red_ave.y=0;red_ave.z=0;
    green_ave.x=0;green_ave.y=0;green_ave.z=0;
    for (int i = 0; i < red->points.size(); ++i) {
        red_ave.x += red->points[i].x;
        red_ave.y += red->points[i].y;
        red_ave.z += red->points[i].z;
    }
    for (int i = 0; i < green->points.size(); ++i) {
        green_ave.x += green->points[i].x;
        green_ave.y += green->points[i].y;
        green_ave.z += green->points[i].z;
    }
    red_ave.x/=red->points.size();red_ave.y/=red->points.size();red_ave.z/=red->points.size();
    green_ave.x/=green->points.size();green_ave.y/=green->points.size();green_ave.z/=green->points.size();
    float distance = sqrt(pow(red_ave.x-green_ave.x, 2) + pow(red_ave.y-green_ave.y, 2) + pow(red_ave.z-green_ave.z, 2));
    cout << red_ave << endl;
    cout << green_ave << endl;
    cout << distance << endl;

    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
	//清除数据并保存
	cloud->points.clear();
	// cout<< "Point cloud saved." << endl;
}
#endif

cv::Mat CreatDepthFromCloud( int width, int height,  Eigen::Matrix3f intrinsic, pcl::PointCloud<pcl::PointXYZ> cloud ){

	std::vector<int> indices;
    pcl::removeNaNFromPointCloud( cloud, cloud, indices);  //remove无效点

    cv::Mat depth=cv::Mat::zeros( height, width, CV_16UC1 );
    for(int i=0;i<cloud.points.size(); i++ ){

        Eigen::Vector3f v( cloud.points[i].x,  cloud.points[i].y , cloud.points[i].z);

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

int main( )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcd/1623298062.606874.pcd",  *cloud );

    cv::Mat color_img = cv::imread("../pcd/1623298062.606874.png", -1);
    cv::Mat depth_img = cv::imread("../pcd/depth.png", -1);

    //color   depth  内外参
	Eigen::Matrix3f intrinsic_depth;
    intrinsic_depth  <<  507.8936724646329,0, 365.0001498679966,
                         0,490.4605167415518, 230.91658138264464,
                         0,0,1;
    // intrinsic_depth  <<  509.1124267578125,0,318.6107177734375,
    //                      0,509.1124267578125,236.57981872558594,
    //                      0,0,1;
	Eigen::Matrix3f intrinsic_color;
    // intrinsic_color  <<  2187.838429, 0, 1050.0,
	// 					 0, 2187.838429, 1005.266588,
	// 					 0, 0, 1;
    intrinsic_color  <<  2136.4240830606855, 0, 1081.5132654639765,
						 0, 2140.234862042462, 1024.5919569660448,
						 0, 0, 1;
	Eigen::Matrix4f T_color_depth;  //此处是color2depth     下面代码传参地方需要depth2color   所以要转置一下
    T_color_depth  <<    0.9980969724486808, -0.0019868338822574493, -0.061631859292932197, -0.37329782506326975,
						 0.0024830583728700562, 0.9999651092489452, 0.00797588276393677,  0.007630703788648211,
						 0.061613862156956196, -0.00811373994354376, 0.9980670815201924, -0.026399685129054495,
						 0., 0., 0., 1.0;

    // cv::Mat depth = CreatDepthFromCloud( 640 , 480, intrinsic_depth, *cloud );
    // cv::imwrite("../pcd/depth_save.png", depth);
    // FuseRGB2PCD( color_img, cloud, intrinsic_color, T_color_depth.inverse() );
    FuseRGB2PCD(color_img, depth_img, intrinsic_color, intrinsic_depth, T_color_depth.inverse());

    return 0;
}