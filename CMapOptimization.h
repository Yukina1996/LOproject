/*
�ڼ����״���̼ƵĻ�����ʵ��λ�˾���
*/
#ifndef LIODLL_CMAP_OPTIMIZATION_H
#define LIODLL_CMAP_OPTIMIZATION_H
#include "LIOBase.h"
#include "LIOCmnFunc.h"
#include "LIOSDC.h"
#include "CLIOOption.h"
#include "CLIOFFStream.h"
#include "CFeatureExtraction.h"
#include "CLidarOdometry.h"

class CMapOptimization 
{
public:
	CMapOptimization();
	virtual ~CMapOptimization();

	CLIOOPTION* m_LIOOpt;
	CPointCloudFFSTREAM* m_PCFFStream;
	CFeatureExtraction* m_FeatureExtraction;
	CLidarOdometry* m_LidarOdometry;

	bool Init();

	bool RunMapOptimization(const int frameIndex);

	void AllocateMemory();

	int CheckKeyFrames();

	pcl::PointCloud<PointType>::Ptr TransformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn);	///< ת����عؼ�֡�ĵ��Ƶ���������ϵ��

	void TransformToEnd(pcl::PointXYZI * const pi, pcl::PointXYZI * const po);

	bool ExtractSurroundingKeyFrames();		///< ��ȡ��Χ�ؼ�֡

	bool DownsampleCurrentScan();			///< ������ƥ��

	bool scan2MapOptimization();			///< �Ż�����

	bool ndtMapOptimization();				///< ��ndt�Ż�

	void PointAssociateToMap(PointType const * const pi, PointType * const po);

	void CornerOptimization(int iterCount);

	void SurfOptimization(int iterCount);

	int LMOptimization(int iterCount);

	void UpdatePointAssociateToMapSinCos();

	void UpdateTransformPointCloudSinCos(PointTypePose *tIn);

	bool Update();

	int SaveKeyFrames();

	bool PublishPointCloudMap();
	  
	int m_SystemCount;

	double m_TransformTobeMapped[6];	///< ���Ż���ȫ��λ�ˣ�����̼��Ǳ߳�ʼ��

	vector<pcl::PointCloud<PointType>::Ptr> m_CornerCloudKeyFrames;	///< ��Źؼ�֡������������
	vector<pcl::PointCloud<PointType>::Ptr> m_SurfCloudKeyFrames;	///< ��Źؼ�֡������������
	
	vector<pcl::PointCloud<PointType>::Ptr> m_PointCloudMap;	///< ������йؼ�֡�����е���
	pcl::PointCloud<PointType>::Ptr m_PointCloudCornerMapDS;
	pcl::PointCloud<PointType>::Ptr m_PointCloudSurfMapDS;

	vector<int> m_vSurroundingExistingKeyPosesID;
	deque<pcl::PointCloud<PointType>::Ptr> m_SurroundingCornerCloudKeyFrames;
	deque<pcl::PointCloud<PointType>::Ptr> m_SurroundingSurfCloudKeyFrames;

	PointType m_PreviousRobotPosPoint;
	PointType m_CurrentRobotPosPoint;
	float m_PreviousRobotYaw;
	float m_CurrentRobotYaw;

	pcl::PointCloud<PointType>::Ptr m_CloudKeyPoses3D;
	pcl::PointCloud<PointTypePose>::Ptr m_CloudKeyPoses6D;			///< ��Źؼ�֡��λ����Ϣ

	pcl::PointCloud<PointType>::Ptr m_SurroundingKeyPoses;
	pcl::PointCloud<PointType>::Ptr m_SurroundingKeyPosesDS;

	pcl::PointCloud<PointType>::Ptr m_LaserCloudCornerLast;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudSurfLast;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudCornerLastDS;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudSurfLastDS;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudSurfLessFlatLast;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudCornerLessSharpLast;

	pcl::PointCloud<PointType>::Ptr m_laserCloudOri;
	pcl::PointCloud<PointType>::Ptr m_coeffSel;

	pcl::PointCloud<PointType>::Ptr m_LaserCloudCornerFromMap;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudSurfFromMap;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudCornerFromMapDS;
	pcl::PointCloud<PointType>::Ptr m_LaserCloudSurfFromMapDS;

	pcl::KdTreeFLANN<PointType>::Ptr m_KdtreeCornerFromMap;
	pcl::KdTreeFLANN<PointType>::Ptr m_KdtreeSurfFromMap;

	pcl::KdTreeFLANN<PointType>::Ptr m_KdtreeSurroundingKeyPoses;

	std::vector<int> m_vPointSearchInd;
	std::vector<float> m_vPointSearchSqDis;

	//����VoxelGrid�˲���������դ���˲�����
	pcl::VoxelGrid<PointType> m_DownSizeFilterCorner;
	pcl::VoxelGrid<PointType> m_DownSizeFilterSurf;
	pcl::VoxelGrid<PointType> m_DownSizeFilterSurroundingKeyPoses;
	pcl::VoxelGrid<PointType> m_DownSizeFilterPointCloudMap;

	PointType pointOri, pointSel, pointProj, coeff;

	cv::Mat matA0;
	cv::Mat matB0;
	cv::Mat matX0;

	cv::Mat matA1;
	cv::Mat matD1;
	cv::Mat matV1;

	bool isDegenerate;
	cv::Mat matP;

	int m_LaserCloudCornerFromMapDSNum;
	int m_LaserCloudSurfFromMapDSNum;
	int m_LaserCloudCornerLastDSNum;
	int m_LaserCloudSurfLastDSNum;

	float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
	float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

	// ����NDT����ı���
	pcl::PointCloud<PointType>::Ptr m_target_cloud;		///< �ӵ�ͼ������ΪĿ�����
	pcl::PointCloud<PointType>::Ptr m_input_cloud;		///< ��ǰ֡������ΪԴ����

};

#endif // !LIODLL_CMAP_OPTIMIZATION_H

