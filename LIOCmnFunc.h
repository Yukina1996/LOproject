/*
LIO程序公用的常用函数
*/

#ifndef LIODLL_LIOCMNFUNC_H
#define LIODLL_LIOCMNFUNC_H

#include "LIOBase.h"

using namespace std;

bool Rawdata2PCL(vector<float> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout);
bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<float> pointCloudout);

bool Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout);
bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<vector<float>> pointCloudout);

double rad2deg(double radians);
double deg2rad(double degrees);

void M13xM33(const double M1[3], const double M2[9], double M3[3]);


#endif // !LIODLL_LIOCMNFUNC_H

