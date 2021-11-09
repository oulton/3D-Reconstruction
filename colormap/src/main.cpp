//
// Created by wy on 2021/8/1.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <math.h>

//#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Sparse>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


//#include "vtk_visualization.h"
#include "camera.h"
#include "vtk_expand.h"
#include "colormap.h"

using namespace std;

int main() {

    Camera camera;
    camera.LoadCameraExtrisicParam("../data/ex_param.txt");

    Eigen::Matrix3d color_intrinsic;  //color-param
    color_intrinsic << 2187.838429, 0, 1050.0,
            0, 2187.838429, 1005.266588,
            0, 0, 1;

    string filename = "../data/mesh.ply";
    Eigen::Matrix4d extrinsic;
    Eigen::Matrix3d intrinsic;

    string path = "../data/" + camera.Name[0] + ".png";
    cv::Mat color = cv::imread(path, -1);
    if(color.empty())
        cerr<<"error\n";

    colormap( filename, color, camera.extrinsic[0], color_intrinsic );

    return 0;
}