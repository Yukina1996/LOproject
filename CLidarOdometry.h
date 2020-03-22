/*
ʵ�ּ����״���̼ƵĹ��ܣ���������֡���λ�˹��ƺͶ�֡���Ƽ�ľֲ��Ż�
*/

#ifndef LIODLL_CLIDAR_ODOMETRY_H
#define LIODLL_CLIDAR_ODOMETRY_H

#include "LIOBase.h"
#include "LIOCmnFunc.h"
#include "LIOSDC.h"
#include "CLIOOption.h"
#include "CLIOFFStream.h"
#include "CFeatureExtraction.h"
#include "BaseMatrix.h"
#include "CPose3D.h"

using namespace basetk;

class CLidarOdometry
{
public:
	CLidarOdometry();
	virtual ~CLidarOdometry();

	CLIOOPTION* m_LIOOpt;
	CPointCloudFFSTREAM* m_PCFFStream;
	CFeatureExtraction* m_FeatureExtraction;

public:

	bool RunLidarOdometry(const int frameIndex);

	bool Init();														///< ����ǰ��ĵ����ļ�����ʼ��vector������

	bool ResetParameters();

	bool InitPreviousPointsCloud(const int frameIndex);

	void TransformToStart(pcl::PointXYZI * const pi, pcl::PointXYZI * const po);

	void TransformToEnd(pcl::PointXYZI * const pi, pcl::PointXYZI * const po);

	int UpdateTransformation();

	void FindCorrespondingSurfFeatures(int iterCount);

	void FindCorrespondingCornerFeatures(int iterCount);

	bool CalculateTransformation(int iterCount);
	bool NewCalculateTransformation(int iterCount);

	bool calculateTransformationSurf(int iterCount);

	bool calculateTransformationCorner(int iterCount);

	void integrateTransformation();

	void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
		float &ox, float &oy, float &oz);
	
	void GetTransformationBetween();

	void PoseCorrection(double matX[]);


					
	int CornerFeatureNum, SurfFeatureNum;

	float transformCur[6] = { 0 };			///< ��ǰ֡�����һ֡��״̬ת������in the local frame
	float transformSum[6] = { 0 };			///< ��ǰ֡����ڵ�һ֡��״̬ת������in the global frame
	Pose3D m_TransformCur;					///< ��Ե�һ֡�� P(X,Y,Z) angle(yaw,pitch,roll)
	vector<Pose3D> m_TransformSum;

	double m_RotaMatrixBetween[9];					///< �����һ֡����ת����3x3
	double m_RotaMatrixBetween_T[9];				///< �����һ֡����ת����3x3
	double m_RotaAngleBetween[3] = { 0 };					///< �����һ֡��ŷ����3x1
	double m_TranslationBetween[3] = { 0 };					///< �����һ֡��ƽ����3x1
	double CurRotAng[3], CurPos[3], PreRotAng[3], PrePos[3], CurRot[9], CurRot_T[9], PreRot[9];
	double m_ErrorState[6];							///< deltaP phi



	//��ǰʱ��
	//struct SegCloudInfo m_SegInfo;									///< �ָ���Ƶ�����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SegmentedPointsCur;			///< �ָ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_CornerPointsSharpCur;				///< ����������,���ʴ�
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_CornerPointsLessSharpCur;			///< ���������ƣ����ʽϴ�
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SurfPointsFlatCur;					///< ���������ƣ�����С
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SurfPointsLessFlatCur;				///< ���������ƣ����ʽ�С

	//��һʱ�� ���ָ�����ļ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_CornerPointsLast;					
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SurfPointsLast;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScan;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_surfPointsLessFlatScanDS;

	pcl::VoxelGrid<pcl::PointXYZI> m_DownSizeFilter;
	int m_LaserCloudCornerLastNum;
	int m_LaserCloudSurfLastNum;

	pcl::PointCloud<pcl::PointXYZI>::Ptr m_laserCloudOri;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_coeffSel;

	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kdtreeCornerLast;	
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kdtreeSurfLast;

	vector<int> m_vPointSearchIndex;
	vector<float> m_vPointSearchSqDis;

	vector<float> m_vpointSearchCornerInd1;
	vector<float> m_vpointSearchCornerInd2;

	vector<float> m_vpointSearchSurfInd1;
	vector<float> m_vpointSearchSurfInd2;
	vector<float> m_vpointSearchSurfInd3;

	pcl::PointXYZI m_pointOri, m_pointSel, m_pointClose, m_coeff, pointSel, tripod1, tripod2, tripod3;

	int m_SystemCount;

	bool isDegenerate;
	cv::Mat matP;
};


#endif // !CLIDAR_ODOMETRY_H
