/*
 实现点云的分割和特征点的提取
 */
#ifndef LIODLL_CFEATURE_EXTRACTION_H
#define LIODLL_CFEATURE_EXTRACTION_H

#include "LIOBase.h"
#include "LIOCmnFunc.h"
#include "LIOSDC.h"
#include "CLIOOption.h"
#include "CLIOFFStream.h"

class CFeatureExtraction
{
public:
	CFeatureExtraction();
	~CFeatureExtraction();

	CLIOOPTION* m_LIOOpt;
	CPointCloudFFSTREAM* m_PCFFStream;

	bool RunCloudSegmentation(const int SystemCount);///< 执行点云分割

	bool InitValues();					///< 初始化
	bool FindStartEndAngle();
	bool ProjectPointCloud();
	bool GroundRemoval();
	bool CloudSegmentation();
	void labelComponents(int row, int col);
	void ResetParameters();

	bool AdjustDistortion();			///< 坐标系转换、IMU初始值辅助点云畸变校正
	bool CalculateSmoothness();
	bool MarkOccludedPoints();
	bool ExtractFeatures();

	bool SaveSegPointCloud();

public:
	struct SegCloudInfo m_SegCloudInfo;									///< 分割点云的性质
	vector<struct PointCloudType> m_PointCloudMapCur;					///< 输出标记了特征点的点云集

	pcl::PointCloud<pcl::PointXYZI>::Ptr m_fullPoints;						///< 处理后的原始点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_segmentedPoints;					///< 分割点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_cornerPointsSharp;				///< 边特征点云,曲率大
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_cornerPointsLessSharp;			///< 边特征点云，曲率较大
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsFlat;					///< 面特征点云，曲率小
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlat;				///< 面特征点云，曲率较小

	//pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScan;
	//pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScanDS;


	string m_segmentedPCFile;									///< 输出特征点云集到文件中

private:
	int m_SystemCount;					///< 提取第某帧的点云特征

	pcl::PointCloud<pcl::PointXYZI>::Ptr m_rawPointCloudCur;		///< 输入当前原始点云
	cv::Mat m_rangeMat;
	cv::Mat m_labelMat;
	cv::Mat m_groundMat;
	int m_labelCount;
	float m_startOrientation;
	float m_endOrientation;

	std::vector<std::pair<uint8_t, uint8_t>> m_vNeighborIterator;

	uint16_t *m_allPushedIndX;
	uint16_t *m_allPushedIndY;

	uint16_t *m_queueIndX;
	uint16_t *m_queueIndY;

	vector<smoothness_t> m_vCloudSmoothness;
	vector<float> m_vCloudCurvature;
	vector<int> m_vCloudNeighborPicked;
	vector<int> m_vCloudLabel;	   	 
};





#endif
