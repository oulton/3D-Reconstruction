//参考：https://www.cnblogs.com/gary-guo/p/6542141.html#commentform
//2019-05-05
//实现了将深度图像和RGB彩色图像转换成RGB点云
//*********注意数据集深度图与rgb图像的对应关系***********
// c++标准库 
#include <iostream>
#include <string>
// opencv库
#include <opencv2/opencv.hpp>
// PCL库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
 
using namespace std;
 
// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
 
// 相机内参
 const double camera_factor=1000;
 const double camera_cx=320;
 const double camera_cy=240;
 const double camera_fx=583.0;
 const double camera_fy=583.0;

//const double camera_factor=1000;
//const double camera_cx=319.5439758300781;
//const double camera_cy=240.34002685546875;
//const double camera_fx=384.6130676269531;
//const double camera_fy=384.6130676269531;
 
// 主函数
int main(int argc,char** argv)
{
	//读取rgb图像和depth图像，并转化为点云
	//图像矩阵
	cv::Mat rgb, depth;
	//使用cv::imread()来读取图像
	//rgb图像是8UC3的彩色图像
	rgb = cv::imread("../frame-003498.color.jpg");
	//depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改
	depth = cv::imread("../frame-003498.depth.png", -1);
 
	//点云变量
	//使用智能指针，创建一个空点云。这种指针用完会自动释放
	PointCloud::Ptr cloud(new PointCloud);
	
	//遍历深度图
	for(int m = 0; m<depth.rows; m++) {
		for(int n = 0; n<depth.cols; n++) {
		//获取深度图中(m,n)处的值
		ushort d = depth.ptr<ushort>(m)[n];
		//d可能没有值，若如此，跳过此点
		if(d == 0)
			continue;
		//d存在值，则向点云增加一个点
		PointT p;
		//计算这个点的空间坐标
		p.z = double(d)/camera_factor;
		p.x = (n-camera_cx)*p.z/camera_fx;
		p.y = (m-camera_cy)*p.z/camera_fy;
		//从rgb图像中获取它的颜色
		//rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
		p.b = rgb.ptr<uchar>(m)[n*3];
		p.g = rgb.ptr<uchar>(m)[n*3+1];
		p.r = rgb.ptr<uchar>(m)[n*3+2];
		//把p加入到点云中
		cloud->points.push_back(p);
		}
	}
	//设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout<< "point cloud size=" << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile("./pointcloud1.pcd", *cloud);
	//清除数据并保存
	cloud->points.clear();
	cout<< "Point cloud saved." << endl;
	return 0;
}
