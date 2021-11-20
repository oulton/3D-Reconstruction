//
// Created by wy on 20-8-29.
//

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <camera.h>
#include <mvs-make_data.h>

using namespace std;

#define NUM_CAMS 58

int main( ) {

    string project_path="../data/";

    Camera_param camera;
    camera.LoadCameraParam(project_path+"camera_para/ex_param.txt");

    vector<Mat> color_(NUM_CAMS);
    for( int i=0;i<NUM_CAMS;i++){
        string pic_path = project_path+"pic/frame"+str(boost::format("%04d")%(i*10))+".jpg";
        color_[i] = imread( pic_path, -1 ); //png
    }

    make_data(  project_path , camera, color_ );

    return 0;
}
