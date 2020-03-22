/*
程序公用的一些结构体、常量的定义
*/
#ifndef LIODLL_LIOSDC_H
#define LIODLL_LIOSDC_H

//#define M_PI 3.14159265

#define MAXSIZE             (1024)

#include "LIOBase.h"

typedef pcl::PointXYZI  PointType;

using namespace std;

extern long gs_SystemCount;									///< 每处理一帧点云计数加一
static const float SegmentTheta = 1.0472;					///< 点云分割时的角度跨度上限（π/3）
static const int SegmentValidPointNum = 5;					///< 检查上下左右连续5个点做为分割的特征依据
static const int SegmentValidLineNum = 3;

static const int EdgeFeatureNum = 2;						///< 每个子区域最多提供2个边特征点
static const int SurfFeatureNum = 4;						///< 每个子区域最多提供4个面特征点
static const int SectionsTotal = 6;							///< 360度 划分6个子区域
static const float EdgeThreshold = 0.1;						///< 边or面特征点的曲率阈值
static const float SurfThreshold = 0.1;
static const float NearestFeatureSearchSqDist = 20;

static const int SlidingWindow = 4;

static const double SurroundingKeyframeSearchRadius = 50;
//static const float KeyFramesDistance = 3;					///< 关键帧彼此距离4m以上
//static const float KeyFramesAttitude = 15;					///< 关键帧彼此转过30度以上


struct PointCloudType
{
	float x, y, z;
	float intensity;
	int rowID;
	int colID;
	int label;					///< 标记点云的特征类型

	PointCloudType()
	{
		x = -1; y = -1; z = -1;
		intensity = -999;
		rowID = -1; colID = -1; label = -999;
	}
};

struct SegCloudInfo
{
	vector<int> startRingIndex;
	vector<int> endRingIndex;

	float startOrientation;
	float endOrientation;
	float orientationDiff;

	vector<bool> segmentedCloudGroundFlag;
	vector<uint32_t> segmentedCloudColInd;
	vector<float> segmentedCloudRange;
};

struct smoothness_t {
	float value;
	int ind;
};

struct by_value {
	bool operator()(smoothness_t const &left, smoothness_t const &right) {
		return left.value < right.value;
	}
};

struct Pose3D {
	double yaw, pitch, roll;
	double posX, posY, posZ;
	Pose3D() {
		yaw = pitch = roll = 0;
		posX = posY = posZ = 0;
	}
};

///< PCL自定义点类型
struct PointXYZIRPYT
{
	PCL_ADD_POINT4D		///// 该点类型有4个元素
		PCL_ADD_INTENSITY;
	float roll;
	float pitch;
	float yaw;
	double time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;// 强制SSE对齐

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,// 注册点类型宏
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif // !LIODLL_LIOSDC_H

