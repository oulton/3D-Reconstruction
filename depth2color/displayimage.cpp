/*
将深度图映射到彩色图上，生成和深度图匹配的对齐彩色图
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

#define RGB_WIDTH 640
#define RGB_HIGTH 480
#define DEP_WIDTH 640
#define DEP_HIGTH 480

struct KinectParm
{
    float fx_rgb;
    float fy_rgb;
    float cx_rgb;
    float cy_rgb;

    float fx_ir;
    float fy_ir;
    float cx_ir;
    float cy_ir;

    Eigen::Matrix3f R_ir2rgb;
    Eigen::Vector3f T_ir2rgb;
};

bool loadParm(KinectParm* kinectParm)
{
    // 加载参数
    ifstream parm("./registration.txt");
    string stringLine;
    if (parm.is_open())
    {
        // rgb相机参数：fx,fy,cx,cy
        getline(parm, stringLine);
        stringstream lin(stringLine);
        string s1, s2, s3, s4, s5, s6, s7, s8, s9;
        lin >> s1 >> s2 >> s3 >> s4;
        kinectParm->fx_rgb = atof(s1.c_str());
        kinectParm->fy_rgb = atof(s2.c_str());
        kinectParm->cx_rgb = atof(s3.c_str());
        kinectParm->cy_rgb = atof(s4.c_str());
        stringLine.clear();
        // ir相机参数：fx,fy,cx,cy
        getline(parm, stringLine);
        stringstream lin2(stringLine);
        lin2 << stringLine;
        lin2 >> s1 >> s2 >> s3 >> s4;
        kinectParm->fx_ir = atof(s1.c_str());
        kinectParm->fy_ir = atof(s2.c_str());
        kinectParm->cx_ir = atof(s3.c_str());
        kinectParm->cy_ir = atof(s4.c_str());
        stringLine.clear();

        // R_ir2rgb
        getline(parm, stringLine);
        stringstream lin3(stringLine);
        lin3 << stringLine;
        lin3 >> s1 >> s2 >> s3 >> s4 >> s5 >> s6 >> s7 >> s8 >> s9;
        kinectParm->R_ir2rgb <<
            atof(s1.c_str()), atof(s2.c_str()), atof(s3.c_str()),
            atof(s4.c_str()), atof(s5.c_str()), atof(s6.c_str()),
            atof(s7.c_str()), atof(s8.c_str()), atof(s9.c_str());
        stringLine.clear();

        // T_ir2rgb
        getline(parm, stringLine);
        stringstream lin4(stringLine);
        lin4 << stringLine;
        lin4 >> s1 >> s2 >> s3;
        kinectParm->T_ir2rgb << atof(s1.c_str()), atof(s2.c_str()), atof(s3.c_str());
    }
    else
    {
        cout << "parm.txt not right!!!";
        return false;
    }
    cout << "******************************************" << endl;

    cout << "fx_rgb:    " << kinectParm->fx_rgb << endl;
    cout << "fy_rgb:    " << kinectParm->fy_rgb << endl;
    cout << "cx_rgb:    " << kinectParm->cx_rgb << endl;
    cout << "cy_rgb:    " << kinectParm->cy_rgb << endl;
    cout << "******************************************" << endl;
    cout << "fx_ir:     " << kinectParm->fx_ir << endl;
    cout << "fy_ir:     " << kinectParm->fy_ir << endl;
    cout << "cx_ir:     " << kinectParm->cx_ir << endl;
    cout << "cy_ir:     " << kinectParm->cy_ir << endl;
    cout << "******************************************" << endl;
    cout << "R_ir2rgb:" << endl << kinectParm->R_ir2rgb << endl;
    cout << "******************************************" << endl;
    cout << "T_ir2rgb:" << endl << kinectParm->T_ir2rgb << endl;
    cout << "******************************************" << endl;
    return true;
}

Eigen::Vector3f cam2point( int m, int n, KinectParm* kinectParm  ){

    return Eigen::Vector3f(
            ( n - kinectParm->cx_ir ) / kinectParm->fx_ir ,
            ( m - kinectParm->cy_ir ) / kinectParm->fy_ir ,
                    1
            );
}


int main()
{
    // 1. 读取参数
    KinectParm *parm = new KinectParm();
    if(!loadParm(parm))
        return 0;
    cout << parm->fx_ir <<
    " "  << parm->fy_ir <<
    " "  << parm->cx_ir <<
    " "  << parm->cy_ir <<endl;
    // 2. 载入rgb图片和深度图并显示
    Mat bgr(RGB_HIGTH, RGB_WIDTH, CV_8UC4);
    //bgr = imread("./rgbd/image_rgb/1617340415.827282.png",-1);
    bgr = imread("../2.png",-1);
    Mat depth(DEP_HIGTH, DEP_WIDTH, CV_16UC1);
    //depth = imread("./rgbd/image_depth/1617340415.827282.png", -1);   // 图片读入后的格式不一定和定义时候的一样，比如这里读入后的格式就是8UC3
    depth = imread("../22.png", -1);

    Eigen::Matrix3f  K;
    K<<610.90966796875, 0.0, 325.97857666015625, 
       0.0, 610.1874389648438, 228.50498962402344,
       0,0,1;
    // K<<916.364501953125, 0.0, 648.9678955078125, 
    //    0.0, 915.2811279296875, 342.75750732421875, 
    //    0.0, 0.0, 1.0;

    Eigen::Matrix4f  T;
    T.block(0,0,3,3)=parm->R_ir2rgb;
    T.block(0,3,3,1)=parm->T_ir2rgb;
    T(3,3)=0;
    Eigen::Matrix4f  T_=T.inverse();


    Mat color_ir=Mat::zeros( depth.rows, depth.cols, CV_8UC3 );
    Mat depth_ir=Mat::zeros( depth.rows, depth.cols, CV_16UC1 );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    for(int m=0;m<depth.rows;m++)
        for(int n=0;n<depth.cols;n++){
            double d=depth.ptr<ushort>(m)[n];
            if(d==0 || d>20000)
                continue;
            Eigen::Vector3f p=cam2point( m, n, parm );
            p=p*float(d/1000);

            pcl::PointXYZRGB P;
            P.x=p[0];
            P.y=p[1];
            P.z=p[2];
            // P.r=255;
            // P.g=0;
            // P.b=0;
            // cloud->push_back(P);

            Eigen::Vector3f p1=T.block(0,0,3,3)*p+T.block(0,3,3,1);


            Eigen::Vector3f p2=K*p1;
            int u=p2[0]/p2[2];
            int v=p2[1]/p2[2];
            if( u<0||u>=RGB_WIDTH || v<0||v>=RGB_HIGTH )
                continue;

            color_ir.ptr<Vec3b>(m)[n]=bgr.ptr<Vec3b>(v)[u];
            depth_ir.ptr<ushort>(v)[u] = depth.ptr<ushort>(m)[n];

            P.b = bgr.ptr<uchar>(v)[u*3];
            P.g = bgr.ptr<uchar>(v)[u*3+1];
            P.r = bgr.ptr<uchar>(v)[u*3+2];
            cloud->push_back(P);
        }


    // imshow("1",color_ir);
    // waitKey(0);
    imwrite("./color_ir.png",color_ir);
    imwrite("./depth_ir.png",depth_ir);
    

    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
	//清除数据并保存
	cloud->points.clear();
	cout<< "Point cloud saved." << endl;

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0.5, 0.5, 0.5);
    // viewer->addPointCloud<pcl::PointXYZRGB>( cloud, "sample cloud");
    // //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // viewer->addCoordinateSystem(1.0);

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }

    return 0;
}
