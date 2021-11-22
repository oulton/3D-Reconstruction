//
// Created by xue420 on 20-1-14.
//

#ifndef PROJECT_CAMERA_H
#define PROJECT_CAMERA_H


#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;


class Camera_param {

public:
    std::vector< Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > extrinsic ;
    std::vector< Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > intrinsic ;
    std::vector<string> Name;
    std::map<string, int> Map;

    Camera_param(){

    }

    void LoadCameraParam(std::string filePath, std::string ops) 
    {
        if(ops == "photoscan")
        {
            Eigen::Matrix4d trans;
            trans << 1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, -1, 0,
                    0, 0, 0, 1;

            ifstream infile;
            infile.open(filePath);
            if (!infile.is_open())
            {
                cerr << "The file does not exist " << endl;
                exit(-1);
            }
            string str;
            string name;
            double x, y, z, Omega, Phi, Kappa,r11, r12, r13, r21, r22, r23, r31, r32, r33;

            while(getline(infile,str)){
                if(str[0]=='#')
                    continue;
                istringstream in(str);
                in>>name>>x>>y>>z>>Omega>>Phi>>Kappa>>r11>>r12>>r13>>r21>>r22>>r23>>r31>>r32>>r33;

                Eigen::Matrix4d extrinsic_;
                extrinsic_ <<  r11,r21,r31,x,
                        r12,r22,r32,y,
                        r13,r23,r33,z,
                        0,0,0,1;
                extrinsic_=extrinsic_ * trans;
                extrinsic.push_back(extrinsic_);

                Map[name]=Name.size();
                Name.push_back(name);

                Eigen::Matrix3d intr;
                intr << 2187.8384, 0, 1050.0,
                        0, 2187.8384, 1005.266588,
                        0, 0, 1;
                intrinsic.push_back(intr);
            }

            infile.close();
            cout<< "load camera extrinsics" <<endl;
        }
        else
        {
            ifstream infile;
            infile.open(filePath);
            if (!infile.is_open())
            {
                cerr << "The file does not exist " << endl;
                exit(-1);
            }
            string str;
            string stamp;
            double q0,  q1,  q2,  q3,  x,  y,  z;

            while(getline(infile,str)){
                if(str[0]=='#')
                    continue;
                istringstream in(str);
                in>>stamp>>q0>>q1>>q2>>q3>>x>>y>>z;

                Eigen::Matrix4d extrinsic_;
                extrinsic_ <<   1-2*q2*q2-2*q3*q3, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2, x,
                                2*q1*q2+2*q0*q3, 1-2*q1*q1-2*q3*q3, 2*q2*q3-2*q0*q1, y,
                                2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 1-2*q1*q1-2*q2*q2, z,
                                0, 0, 0, 1;
                extrinsic.push_back(extrinsic_);

                Map[stamp]=Name.size();
                Name.push_back(stamp);

                Eigen::Matrix3d intr;
                intr << 1.0660120360637927e+03, 0, 9.4558752751085558e+02, 
                        0, 1.0688708399911650e+03, 5.2006994012356529e+02, 
                        0,  0, 1;
                intrinsic.push_back(intr);
            }

            infile.close();
            cout<< "load camera extrinsics" <<endl;
        }
        
    }


};


#endif //PROJECT_CAMERA_H
