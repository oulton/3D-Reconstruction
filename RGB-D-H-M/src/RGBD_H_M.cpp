#include "camparaload.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

Eigen::Vector3f cam2point(int , int , Eigen::Matrix3f);

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

int main()
{
    CamParasLoad CamParas_("../config/CarParams.yaml");
    
    Mat color = imread("../data/color.png", -1);
    Mat depth = imread("../data/depth.png", -1);
    Mat heat = imread("../data/heat.jpg", -1);
    Mat inf1 = imread("../data/inf1.png", -1);
    Mat inf2 = imread("../data/inf2.png", -1);
    Mat multi1 = imread("../data/multi.tif", -1);

#if ISCOUT
    cout << color.channels() << "  " << color.type() << endl;
    cout << depth.channels() <<  "  " << depth.type() << endl;
    cout << heat.channels() <<  "  " << heat.type() << endl;
    cout << inf1.channels() <<  "  " << inf1.type() << endl;
    cout << inf2.channels() <<  "  " << inf2.type() << endl;
    cout << multi1.channels() << "  " << multi1.type() << endl;
#endif

    Eigen::Matrix3f  CameraColor, CameraDepth, CameraMulti1, CameraHeat, CameraInf1, CameraInf2;   //相机内参
    cv2eigen(CamParas_.get_Color_para(), CameraColor);
    cv2eigen(CamParas_.get_Depth_para(), CameraDepth);
    cv2eigen(CamParas_.get_Heat_para(), CameraHeat);
    cv2eigen(CamParas_.get_Multi1_para(), CameraMulti1);
    cv2eigen(CamParas_.get_Inf1_para(), CameraInf1);
    cv2eigen(CamParas_.get_Inf2_para(), CameraInf2);

    Eigen::Matrix4f  T_color_depth, T_color_multi1, T_color_infra1, T_color_infra2, T_lidar_heat, T_lidar_color;   //相机外参
    cv2eigen(CamParas_.get_T_color_depth(), T_color_depth);
    cv2eigen(CamParas_.get_T_color_multi1(), T_color_multi1);

    cv2eigen(CamParas_.get_T_color_infra1(), T_color_infra1);
    cv2eigen(CamParas_.get_T_color_infra2(), T_color_infra2);

    cv2eigen(CamParas_.get_T_lidar_heat(), T_lidar_heat);
    cv2eigen(CamParas_.get_T_lidar_color(), T_lidar_color);

    // Eigen::Matrix4f  T_multi_depth;
    // T_multi_depth = T_color_multi * T_color_depth;

#if ISCOUT
    std::cout << "CameraColor instrinc matrix :" << std::endl << CameraColor << std::endl;
	std::cout << "CameraDepth instrinc matrix :" << std::endl << CameraDepth << std::endl;
	std::cout << "CameraInf1 instrinc matrix :" << std::endl << CameraInf1 << std::endl;
	std::cout << "CameraInf2 instrinc matrix :" << std::endl << CameraInf2 << std::endl;
	std::cout << "CameraMulti1 instrinc matrix :" << std::endl << CameraMulti1 << std::endl;
	std::cout << "CameraHeat instrinc matrix :" << std::endl << CameraHeat << std::endl;

	std::cout << "T_color_depth matrix :" << std::endl << T_color_depth << std::endl;
	std::cout << "T_color_infra1 matrix :" << std::endl << T_color_infra1 << std::endl;
	std::cout << "T_color_infra2 matrix :" << std::endl << T_color_infra2 << std::endl;
	std::cout << "T_color_multi1 matrix :" << std::endl << T_color_multi1 << std::endl;
	std::cout << "T_lidar_heat matrix :" << std::endl << T_lidar_heat << std::endl;
	std::cout << "T_lidar_color matrix :" << std::endl << T_lidar_color << std::endl;
#endif

#if 0
    //Mat color_out = Mat::zeros( color.rows, color.cols, CV_8UC3 );
    Mat depth_out = Mat::zeros( color.rows, color.cols, CV_16UC1 );
    Mat multi1_out = Mat::zeros( color.rows, color.cols, CV_16UC1 );
    for(int m=0;m<depth.rows;m++)
    {
        for(int n=0;n<depth.cols;n++)
        {
            double d=depth.ptr<ushort>(m)[n];
            if(d<=0 || d>5000)
                continue;
            Eigen::Vector3f depthFramPoint = cam2point( m, n, CameraDepth );
            depthFramPoint = depthFramPoint * ( d / 1000.0 );

            //depth对齐到color
            
            Eigen::Vector3f colorPixPoint = CameraColor * (T_color_depth.block(0,0,3,3) * depthFramPoint + T_color_depth.block(0,3,3,1));
            int u_=colorPixPoint[0]/colorPixPoint[2];
            int v_=colorPixPoint[1]/colorPixPoint[2];
            if( u_<0||u_>=color.cols || v_<0||v_>=color.rows )
                continue;
            depth_out.ptr<ushort>(v_)[u_] = colorPixPoint[2] * 1000.0;
            

            //multi1对齐到color
            
            Eigen::Vector3f multi1PixPoint = CameraMat1 * (T_multi_depth.block(0,0,3,3) * depthFramPoint + T_multi_depth.block(0,3,3,1));
            int u=multi1PixPoint[0]/multi1PixPoint[2];
            int v=multi1PixPoint[1]/multi1PixPoint[2];
            if( u<0||u>=multi1.cols || v<0||v>=multi1.rows )
                continue;
            multi1_out.ptr<ushort>(v_)[u_] = multi1.ptr<ushort>(v)[u];

        }
    }
        
    imwrite("../result/color.png", color);
    imwrite("../result/depth_out.png", depth_out);
    imwrite("../result/multi1_out.tif", multi1_out);
    cout << "save end" << endl;
#endif

}
