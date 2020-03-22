# LOproject
LiDAR SLAM, especially LiDAR Odometry and local optimization

说明：程序在visual studio 2017环境下进行开发，依赖的第三方库有PCL(1.8.1)和openCV(3.4.1)

测试数据：KITTI数据集

1 × Velodyne HDL-64E rotating 3D laser scanner, 10 Hz,64 beams, 0.09 degree angular resolution, 2 cm distance accuracy, collecting ∼ 1.3 million points/second, field of view: 360◦ horizontal, 26.8◦ vertical, range: 120 m
Note that the Velodyne laser scanner rotates continuously around its vertical axis (counter-clockwise逆时针), which can be taken into account using the timestamp files.
  
XXXXXX.bin: 存储每帧点云的二进制文件，格式为N*4 float，N个扫描点，x(m),y(m),z(m), reflectance

二、程序类及成员

1、CScanRegistration
从源点云中分割出待提取特征的点云基础，根据曲率大小提取出线特征点和面特征点。输出分割点云文件。
cv::mat rangeMat 标记对应行/列的距离值
cv::mat labelMat 标记对应行/列是否分割点云
cv::mat groundMat 标记对应行/列是否地面点云

2、CLidarOdometry
提取特征点，相邻帧间进行特征匹配，恢复相邻帧间的位姿。后端进行局部优化，优化位姿估计。

3、CLIOApplication
最关键的类，最重要的主函数runLIOApp(string optFile)，调用其他类里的数据解码、数据处理、前端跟踪、后端优化的功能，实现LIO函数的功能。

4、CLIOFFStream
从规定格式的点云文件中读取数据流

5、CLIOOption
class CLIOOPTION// 读取配置文件，存放配置参数等等
配置文件内容包括：
数据文件路径
LIO处理方式:
LIO参数配置:
激光雷达和惯导之间的外参标定：杆臂值和旋转矩阵

6、LIOCmnFunc
一些公用的函数，比如原始点云转pcl点云，pcl点云转自定义点云格式

7、LIOSDC
程序相关的结构体,定义和常量
Struct cloudInfo;//存放分割点云信息的结构体
Struct PointCloudType;//自定义的点云类型

