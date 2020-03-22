/*
 ʵ�ֵ��Ƶķָ�����������ȡ
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

	bool RunCloudSegmentation(const int SystemCount);///< ִ�е��Ʒָ�

	bool InitValues();					///< ��ʼ��
	bool FindStartEndAngle();
	bool ProjectPointCloud();
	bool GroundRemoval();
	bool CloudSegmentation();
	void labelComponents(int row, int col);
	void ResetParameters();

	bool AdjustDistortion();			///< ����ϵת����IMU��ʼֵ�������ƻ���У��
	bool CalculateSmoothness();
	bool MarkOccludedPoints();
	bool ExtractFeatures();

	bool SaveSegPointCloud();

public:
	struct SegCloudInfo m_SegCloudInfo;									///< �ָ���Ƶ�����
	vector<struct PointCloudType> m_PointCloudMapCur;					///< ��������������ĵ��Ƽ�

	pcl::PointCloud<pcl::PointXYZI>::Ptr m_fullPoints;						///< ������ԭʼ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_segmentedPoints;					///< �ָ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_cornerPointsSharp;				///< ����������,���ʴ�
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_cornerPointsLessSharp;			///< ���������ƣ����ʽϴ�
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsFlat;					///< ���������ƣ�����С
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlat;				///< ���������ƣ����ʽ�С

	//pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScan;
	//pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScanDS;


	string m_segmentedPCFile;									///< ����������Ƽ����ļ���

private:
	int m_SystemCount;					///< ��ȡ��ĳ֡�ĵ�������

	pcl::PointCloud<pcl::PointXYZI>::Ptr m_rawPointCloudCur;		///< ���뵱ǰԭʼ����
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
