#ifndef _CAMPARALOAD_H_
#define _CAMPARALOAD_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define ISCOUT 0

class CamParasLoad
{
private:
	cv::Mat CameraColor, CameraDepth, CameraHeat, CameraInf1, CameraInf2, CameraMulti1;
	cv::Mat T_color_depth, T_color_infra1, T_color_infra2, T_color_multi1;
	cv::Mat T_lidar_heat, T_lidar_color;
public:
	CamParasLoad(std::string path);

	cv::Mat get_Color_para();
	cv::Mat get_Depth_para();
	cv::Mat get_Inf1_para();
	cv::Mat get_Inf2_para();
	cv::Mat get_Multi1_para();
	cv::Mat get_Heat_para();

	cv::Mat get_T_color_depth();
	cv::Mat get_T_color_infra1();
	cv::Mat get_T_color_infra2();
	cv::Mat get_T_color_multi1();
	cv::Mat get_T_lidar_heat();
	cv::Mat get_T_lidar_color();
};

CamParasLoad::CamParasLoad(std::string path)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);

	fs["CameraColor"] >> CameraColor;
	fs["CameraDepth"] >> CameraDepth;
	fs["CameraInf1"] >> CameraInf1;
	fs["CameraInf2"] >> CameraInf2;
	fs["CameraMulti1"] >> CameraMulti1;
	fs["CameraHeat"] >> CameraHeat;

	fs["T_color_depth"] >> T_color_depth;
	fs["T_color_infra1"] >> T_color_infra1;
	fs["T_color_infra2"] >> T_color_infra2;
	fs["T_color_multi1"] >> T_color_multi1;
	fs["T_lidar_heat"] >> T_lidar_heat;
	fs["T_lidar_color"] >> T_lidar_color;

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

	fs.release();    	//close the file opened
}

cv::Mat CamParasLoad::get_Color_para(){ return CameraColor; }
cv::Mat CamParasLoad::get_Depth_para(){ return CameraDepth; }
cv::Mat CamParasLoad::get_Inf1_para(){ return CameraInf1; }
cv::Mat CamParasLoad::get_Inf2_para(){ return CameraInf2; }
cv::Mat CamParasLoad::get_Multi1_para(){ return CameraMulti1; }
cv::Mat CamParasLoad::get_Heat_para(){ return CameraHeat; }

cv::Mat CamParasLoad::get_T_color_depth(){ return T_color_depth; }
cv::Mat CamParasLoad::get_T_color_infra1(){ return T_color_infra1; }
cv::Mat CamParasLoad::get_T_color_infra2(){ return T_color_infra2; }
cv::Mat CamParasLoad::get_T_color_multi1(){ return T_color_multi1; }
cv::Mat CamParasLoad::get_T_lidar_heat(){ return T_lidar_heat; }
cv::Mat CamParasLoad::get_T_lidar_color(){ return T_lidar_color; }

#endif
