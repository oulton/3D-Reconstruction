//
// Created by wy on 2020/9/15.
//

#ifndef PROJECT_MVS_MAKE_DATA_H
#define PROJECT_MVS_MAKE_DATA_H

#include <sys/stat.h>
#include <sys/types.h>
#include <boost/format.hpp>
#include "camera.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

void write_ini( string filename, int index,Eigen::Matrix3d intrinsic, Eigen::Matrix4d extrinsic ){

    ofstream outfile ( filename);
    if(!outfile.is_open())
    {
        cout<<" the file open fail"<<endl;
        exit(1);
    }

    double focal_length = 2087.8384/2048 ;//intrinsic(0,0)/1280.0;
    int pixel_aspect = 1;
    float principal_point = 0.5;
    Eigen::Matrix4d extrinsic_= extrinsic.inverse();
    outfile<<"# MVE view meta data is stored in INI-file syntax.\n"
           <<"# This file is generated, formatting will get lost.\n"
           <<"\n";

    outfile<<"[camera]\n";
    outfile<<"focal_length = "<<fixed<<showpoint<<setprecision(10)<<focal_length<<"\n";
    outfile<<"pixel_aspect = "<<1<<"\n";
    outfile<<"principal_point = "<<fixed<<showpoint<<setprecision(1)<<0.5<<" "<<0.5<<"\n";
    outfile<<"rotation = ";
    outfile<<fixed<<showpoint<<setprecision(10);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){
            outfile<<extrinsic_(i,j)<<" ";
        }
    outfile<<"\n";
    outfile<<"translation = "<<extrinsic_(0,3)<<" "<<extrinsic_(1,3)<<" "<< extrinsic_(2,3)<<"\n";

    outfile<<"\n";
    outfile<<"[view]\n";
    outfile<<"id = "<<index<<"\n";
    outfile<<"name = "<<"frame"+str(boost::format("%04d")%(index*10));
    outfile.close();
}


void make_data(  string path , Camera_param camera, vector<Mat> color )
{
    std::string dir = path + "scene/";
    mkdir( dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    dir = dir + "views/";
    mkdir( dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);

    int n=color.size();
    for( int i=0;i<n;i++){
        std::string subdir = dir + "view_"+str( boost::format("%04d") %i )+".mve/";
        mkdir(subdir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        imwrite(subdir +"undistorted.png" ,  color[i] );
        write_ini( subdir+"meta.ini", i, camera.intrinsic[i], camera.extrinsic[i]);
    }

    std::cout << "convert data end" << std::endl;
}

#endif //PROJECT_MVS_MAKE_DATA_H
