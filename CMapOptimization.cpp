#include "CMapOptimization.h"

CMapOptimization::CMapOptimization()
{

	m_DownSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);//20cm*20cm*20cm
	m_DownSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
	m_DownSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0);
	m_DownSizeFilterPointCloudMap.setLeafSize(0.5, 0.5, 0.5);

	AllocateMemory();
}

CMapOptimization::~CMapOptimization()
{
	//�ͷ��ڴ�
	vector<pcl::PointCloud<PointType>::Ptr>().swap(m_CornerCloudKeyFrames);
	vector<pcl::PointCloud<PointType>::Ptr>().swap(m_SurfCloudKeyFrames);
	vector<pcl::PointCloud<PointType>::Ptr>().swap(m_PointCloudMap);

	vector<int>().swap(m_vSurroundingExistingKeyPosesID);

	m_SurroundingCornerCloudKeyFrames.~deque<pcl::PointCloud<PointType>::Ptr>();
	m_SurroundingSurfCloudKeyFrames.~deque<pcl::PointCloud<PointType>::Ptr>();
}

bool CMapOptimization::Init()
{
	//ֱ�Ӵ�m_FeatureExtraction���л�õ�ǰ���Ƶ�������
	m_LaserCloudCornerLast->clear();
	m_LaserCloudSurfLast->clear();

	/**m_LaserCloudCornerLast = *m_FeatureExtraction->m_cornerPointsSharp;
	*m_LaserCloudCornerLast += *m_FeatureExtraction->m_cornerPointsLessSharp;*/
	*m_LaserCloudCornerLast = *m_FeatureExtraction->m_cornerPointsLessSharp;

	/**m_LaserCloudSurfLast = *m_FeatureExtraction->m_surfPointsFlat;
	*m_LaserCloudSurfLast += *m_FeatureExtraction->m_surfPointsLessFlat;*/
	*m_LaserCloudSurfLast = *m_FeatureExtraction->m_surfPointsLessFlat;

	int cornerPointsLessSharpNum = m_LaserCloudCornerLast->points.size();
	for (int i = 0; i < cornerPointsLessSharpNum; i++) {
		TransformToEnd(&m_LaserCloudCornerLast->points[i], &m_LaserCloudCornerLast->points[i]);
	}


	int surfPointsLessFlatNum = m_LaserCloudSurfLast->points.size();
	for (int i = 0; i < surfPointsLessFlatNum; i++) {
		TransformToEnd(&m_LaserCloudSurfLast->points[i], &m_LaserCloudSurfLast->points[i]);
	}

	// ��ʼ���ֲ����Ƶ�ͼ
	m_LaserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

	return true;
}

bool CMapOptimization::RunMapOptimization(const int frameIndex)
{
	m_SystemCount = frameIndex;

	if (CheckKeyFrames() == 1) {// ���ǹؼ�֡��ֱ���˳��Ż�
		return true;
	}

	Init();	

	DownsampleCurrentScan();

	if (ExtractSurroundingKeyFrames())
	{
		scan2MapOptimization();

		Update();

	}
	SaveKeyFrames();

	return true;
}

void CMapOptimization::AllocateMemory()
{
	m_CloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
	m_CloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

	m_KdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
	
	m_SurroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
	m_SurroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

	m_LaserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudSurfLessFlatLast.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudCornerLessSharpLast.reset(new pcl::PointCloud<PointType>());

	m_laserCloudOri.reset(new pcl::PointCloud<PointType>());
	m_coeffSel.reset(new pcl::PointCloud<PointType>());

	m_LaserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
	m_LaserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

	m_KdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
	m_KdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

	m_PointCloudCornerMapDS.reset(new pcl::PointCloud<PointType>());
	m_PointCloudSurfMapDS.reset(new pcl::PointCloud<PointType>());

	matA0 = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
	matB0 = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));//matB0=[-1;-1;-1;-1;-1]
	matX0 = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));
	matA1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
	matD1 = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
	matV1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
	isDegenerate = false;
	matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

	m_LaserCloudCornerFromMapDSNum = 0;
	m_LaserCloudSurfFromMapDSNum = 0;
	m_LaserCloudCornerLastDSNum = 0;
	m_LaserCloudSurfLastDSNum = 0;

	memset(m_TransformTobeMapped, 0, sizeof(m_TransformTobeMapped));

}

int CMapOptimization::CheckKeyFrames()
{
	///< ����̼��Ǳ߶�������ǰ֡�ĳ�ʼλ��
	m_TransformTobeMapped[0] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].pitch;	//rx
	m_TransformTobeMapped[1] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].yaw;		//ry		
	m_TransformTobeMapped[2] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].roll;		//rz
	m_TransformTobeMapped[3] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posX;		//tx
	m_TransformTobeMapped[4] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posY;		//ty
	m_TransformTobeMapped[5] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posZ;		//tz

	m_CurrentRobotPosPoint.x = m_TransformTobeMapped[3];
	m_CurrentRobotPosPoint.y = m_TransformTobeMapped[4];
	m_CurrentRobotPosPoint.z = m_TransformTobeMapped[5];

	m_CurrentRobotYaw = m_TransformTobeMapped[1];

	bool saveThisKeyFrame = true;

	if (m_CloudKeyPoses3D->points.empty())///< ����ؼ�֡�б�û���κ�һ֡
	{

	}
	else
	{
		// �ؼ�֡�˴�֮�����KeyFramesDistance���ϣ�����ˮƽ��ת��KeyFramesAttitude��s����
		if (sqrt((m_PreviousRobotPosPoint.x - m_CurrentRobotPosPoint.x)*
			(m_PreviousRobotPosPoint.x - m_CurrentRobotPosPoint.x)
			+ (m_PreviousRobotPosPoint.y - m_CurrentRobotPosPoint.y)*
			(m_PreviousRobotPosPoint.y - m_CurrentRobotPosPoint.y)
			+ (m_PreviousRobotPosPoint.z - m_CurrentRobotPosPoint.z)*
			(m_PreviousRobotPosPoint.z - m_CurrentRobotPosPoint.z)) < m_LIOOpt->m_KeyFramesDistance
			&& fabs(m_CurrentRobotYaw - m_PreviousRobotYaw) < deg2rad(m_LIOOpt->m_KeyFramesAttitude))
		{
			saveThisKeyFrame = false;
		}
	}

	if (saveThisKeyFrame == false && !m_CloudKeyPoses3D->points.empty())//��ǰ֡���ǹؼ�֡���˳�λ���Ż�
	{
		return 1;
	}


	m_PreviousRobotPosPoint = m_CurrentRobotPosPoint;

	m_PreviousRobotYaw = m_CurrentRobotYaw;

	return 0;
}

pcl::PointCloud<PointType>::Ptr CMapOptimization::TransformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn)
{
	pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

	PointType *pointFrom;
	PointType pointTo;

	int cloudSize = cloudIn->points.size();
	cloudOut->resize(cloudSize);

	for (int i = 0; i < cloudSize; ++i) 
	{
		//20200305 ע���LeGO-LOAM��ͬ
		pointFrom = &cloudIn->points[i];
		float x1 = ctRoll * pointFrom->x - stRoll * pointFrom->y;
		float y1 = stRoll * pointFrom->x + ctRoll * pointFrom->y;
		float z1 = pointFrom->z;

		float x2 = x1;
		float y2 = ctPitch * y1 - stPitch * z1;
		float z2 = stPitch * y1 + ctPitch * z1;

		pointTo.x = ctYaw * x2 + stYaw * z2 + tInX;
		pointTo.y = y2 + tInY;
		pointTo.z = -stYaw * x2 + ctYaw * z2 + tInZ;
		pointTo.intensity = pointFrom->intensity;

		cloudOut->points[i] = pointTo;
	}
	return cloudOut;
}

void CMapOptimization::TransformToEnd(pcl::PointXYZI * const pi, pcl::PointXYZI * const po)
{
	//s��ʾĳ����һ��ɨ�������е����ʱ�䣬��λ���ڲ�
	float s = 10 * (pi->intensity - int(pi->intensity));
	
	float rx = s * m_LidarOdometry->transformCur[0];
	float ry = s * m_LidarOdometry->transformCur[1];
	float rz = s * m_LidarOdometry->transformCur[2];
	float tx = s * m_LidarOdometry->transformCur[3];
	float ty = s * m_LidarOdometry->transformCur[4];
	float tz = s * m_LidarOdometry->transformCur[5];

	float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
	float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
	float z1 = (pi->z - tz);

	float x2 = x1;
	float y2 = cos(rx) * y1 + sin(rx) * z1;
	float z2 = -sin(rx) * y1 + cos(rx) * z1;

	float x3 = cos(ry) * x2 - sin(ry) * z2;
	float y3 = y2;
	float z3 = sin(ry) * x2 + cos(ry) * z2;

	rx = m_LidarOdometry->transformCur[0];
	ry = m_LidarOdometry->transformCur[1];
	rz = m_LidarOdometry->transformCur[2];
	tx = m_LidarOdometry->transformCur[3];
	ty = m_LidarOdometry->transformCur[4];
	tz = m_LidarOdometry->transformCur[5];

	float x4 = cos(ry) * x3 + sin(ry) * z3;
	float y4 = y3;
	float z4 = -sin(ry) * x3 + cos(ry) * z3;

	float x5 = x4;
	float y5 = cos(rx) * y4 - sin(rx) * z4;
	float z5 = sin(rx) * y4 + cos(rx) * z4;

	po->x = cos(rz) * x5 - sin(rz) * y5 + tx;
	po->y = sin(rz) * x5 + cos(rz) * y5 + ty;
	po->z = z5 + tz;
	po->intensity = int(pi->intensity);
}

bool CMapOptimization::ExtractSurroundingKeyFrames()
{
	//if (m_CloudKeyPoses3D->points.empty() == true)
	if (m_CloudKeyPoses3D->points.size() < 1)
		return false;

	// Ѱ�Ҹ�����֮����н�ά����Ŀ�����õ�ͼ���Ʋ�Ҫ̫���ܼ�
	m_SurroundingKeyPoses->clear();
	m_SurroundingKeyPosesDS->clear();

	//surroundingKeyframeSearchRadius��50�ף�Ҳ����˵���ڵ�ǰλ�ý��а뾶���ң��õ������Ĺ켣��
	m_KdtreeSurroundingKeyPoses->setInputCloud(m_CloudKeyPoses3D);
	m_KdtreeSurroundingKeyPoses->radiusSearch(m_CurrentRobotPosPoint, (double)SurroundingKeyframeSearchRadius, m_vPointSearchInd, m_vPointSearchSqDis, 0);
	for (int i = 0; i < m_vPointSearchInd.size(); ++i)
		m_SurroundingKeyPoses->points.push_back(m_CloudKeyPoses3D->points[m_vPointSearchInd[i]]);

	// �Ը����켣��ĵ��ƽ��н��������켣����һ�����
	m_DownSizeFilterSurroundingKeyPoses.setInputCloud(m_SurroundingKeyPoses);
	m_DownSizeFilterSurroundingKeyPoses.filter(*m_SurroundingKeyPosesDS);

	
	int numSurroundingPosesDS = m_SurroundingKeyPosesDS->points.size();
	for (int i = 0; i < m_vSurroundingExistingKeyPosesID.size(); ++i) {
		bool existingFlag = false;
		for (int j = 0; j < numSurroundingPosesDS; ++j) {
			// �����ʽ���������жϸ���ĳһ���ؼ�֡���ڽ�ά����Ƶĵ�j���ؼ�֡
			if (m_vSurroundingExistingKeyPosesID[i] == (int)m_SurroundingKeyPosesDS->points[j].intensity) {
				existingFlag = true;
				break;
			}
		}
		if (existingFlag == false) {
			m_vSurroundingExistingKeyPosesID.erase(m_vSurroundingExistingKeyPosesID.begin() + i);
			m_SurroundingCornerCloudKeyFrames.erase(m_SurroundingCornerCloudKeyFrames.begin() + i);
			m_SurroundingSurfCloudKeyFrames.erase(m_SurroundingSurfCloudKeyFrames.begin() + i);
			--i;
		}
	}



	for (int i = 0; i < numSurroundingPosesDS; ++i) {
		bool existingFlag = false;
		for (auto iter = m_vSurroundingExistingKeyPosesID.begin(); iter != m_vSurroundingExistingKeyPosesID.end(); ++iter) {
			if ((*iter) == (int)m_SurroundingKeyPosesDS->points[i].intensity) {
				existingFlag = true;
				break;
			}
		}
		if (existingFlag == true) {
			continue;
		}
		else {
			int thisKeyInd = (int)m_SurroundingKeyPosesDS->points[i].intensity;
			PointTypePose thisTransformation = m_CloudKeyPoses6D->points[thisKeyInd];
			UpdateTransformPointCloudSinCos(&thisTransformation);
			m_vSurroundingExistingKeyPosesID.push_back(thisKeyInd);
			m_SurroundingCornerCloudKeyFrames.push_back(TransformPointCloud(m_CornerCloudKeyFrames[thisKeyInd]));
			m_SurroundingSurfCloudKeyFrames.push_back(TransformPointCloud(m_SurfCloudKeyFrames[thisKeyInd]));
		}
	}


	for (int i = 0; i < m_vSurroundingExistingKeyPosesID.size(); ++i) {
		*m_LaserCloudCornerFromMap += *m_SurroundingCornerCloudKeyFrames[i];
		*m_LaserCloudSurfFromMap += *m_SurroundingSurfCloudKeyFrames[i];
	}

	m_DownSizeFilterCorner.setInputCloud(m_LaserCloudCornerFromMap);
	m_DownSizeFilterCorner.filter(*m_LaserCloudCornerFromMapDS);
	m_LaserCloudCornerFromMapDSNum = m_LaserCloudCornerFromMapDS->points.size();

	m_DownSizeFilterSurf.setInputCloud(m_LaserCloudSurfFromMap);
	m_DownSizeFilterSurf.filter(*m_LaserCloudSurfFromMapDS);
	m_LaserCloudSurfFromMapDSNum = m_LaserCloudSurfFromMapDS->points.size();

	//����������������laserCloudCornerFromMapDS��laserCloudSurfFromMapDS����ɴ�ƥ��ļ�����Ƶ�ͼ

	return true;
}

bool CMapOptimization::DownsampleCurrentScan()
{
	m_LaserCloudCornerLastDS->clear();
	m_DownSizeFilterCorner.setInputCloud(m_LaserCloudCornerLast);
	m_DownSizeFilterCorner.filter(*m_LaserCloudCornerLastDS);
	m_LaserCloudCornerLastDSNum = m_LaserCloudCornerLastDS->points.size();

	m_LaserCloudSurfLastDS->clear();
	m_DownSizeFilterSurf.setInputCloud(m_LaserCloudSurfLast);
	m_DownSizeFilterSurf.filter(*m_LaserCloudSurfLastDS);
	m_LaserCloudSurfLastDSNum = m_LaserCloudSurfLastDS->points.size();
	return true;
}

bool CMapOptimization::scan2MapOptimization()
{

	// ��ͼ��ı�Ե����������10 ƽ�����������100
	if (m_LaserCloudCornerFromMapDSNum > 10 && m_LaserCloudSurfFromMapDSNum > 100) {

		//����kd������
		m_KdtreeCornerFromMap->setInputCloud(m_LaserCloudCornerFromMapDS);
		m_KdtreeSurfFromMap->setInputCloud(m_LaserCloudSurfFromMapDS);

		for (int iterCount = 0; iterCount < 10; iterCount++) {

			m_laserCloudOri->clear();
			m_coeffSel->clear();

			//�Ż��Ĺ�������̼Ƶļ������ƣ���ͨ������㵽ֱ�߻�ƽ��ľ��룬�����Ż���ʽ����LM�����
			CornerOptimization(iterCount);
			SurfOptimization(iterCount);

			if (LMOptimization(iterCount) == 2)
				break;
		}

	}

	return true;
}

bool CMapOptimization::ndtMapOptimization()
{
	m_target_cloud.reset(new pcl::PointCloud<PointType>());
	m_input_cloud.reset(new pcl::PointCloud<PointType>());

	// m_target_cloud�����ӵ�ͼ��������������ϵ�µ�λ��
	*m_target_cloud = *m_LaserCloudCornerFromMapDS;
	*m_target_cloud += *m_LaserCloudSurfFromMapDS;

	// m_input_cloud���뵱ǰ֡��������������ϵ�µ�λ��
	*m_input_cloud = *m_LaserCloudCornerLast;
	*m_input_cloud += *m_LaserCloudSurfLast;

	UpdatePointAssociateToMapSinCos();
	for (int i = 0; i < m_input_cloud->points.size(); i++)
	{
		pointOri = m_input_cloud->points[i];

		PointAssociateToMap(&pointOri, &pointSel);

		m_input_cloud->points[i] = pointSel;
	}

	pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(m_input_cloud);
	approximate_voxel_filter.filter(*filtered_cloud);

	pcl::NormalDistributionsTransform<PointType, PointType> ndt;

	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);

	ndt.setMaximumIterations(35);

	ndt.setInputSource(filtered_cloud);
	ndt.setInputTarget(m_target_cloud);

	// ��ʼ���趨
	float rx, ry, rz, tx, ty, tz;
	rx = m_TransformTobeMapped[0];
	ry = m_TransformTobeMapped[1];
	rz = m_TransformTobeMapped[2];
	tx = m_TransformTobeMapped[3];
	ty = m_TransformTobeMapped[4];
	tz = m_TransformTobeMapped[5];

	Eigen::Vector3f init_euler_angles(rx, ry, rz);

	Eigen::Matrix3f init_rotation_matrix;
	init_rotation_matrix = Eigen::AngleAxisf(init_euler_angles[1], Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(init_euler_angles[0], Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(init_euler_angles[2], Eigen::Vector3f::UnitZ());

	Eigen::Vector3f init_translation(tx, ty, tz);

	Eigen::Matrix4f init_guess;
	init_guess <<
		init_rotation_matrix(0, 0), init_rotation_matrix(0, 1), init_rotation_matrix(0, 2), init_translation(0),
		init_rotation_matrix(1, 0), init_rotation_matrix(1, 1), init_rotation_matrix(1, 2), init_translation(1),
		init_rotation_matrix(2, 0), init_rotation_matrix(2, 1), init_rotation_matrix(2, 2), init_translation(2),
		0, 0, 0, 1;

	 //init_rotation_matrix.eulerAngles(1, 2, 0);		// ������ת˳����ZXY


	// Calculating required rigid transform to align the input cloud to the target cloud.
	pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
	ndt.align(*output_cloud, init_guess);

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
		<< " score: " << ndt.getFitnessScore() << std::endl;

	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud(*m_input_cloud, *output_cloud, ndt.getFinalTransformation());

	return true;
}

void CMapOptimization::PointAssociateToMap(PointType const * const pi, PointType * const po)
{
	//20200305 ע���LeGO-LOAM��ͬ
	// ��Z��ת
	float x1 = cRoll * pi->x - sRoll * pi->y;
	float y1 = sRoll * pi->x + cRoll * pi->y;
	float z1 = pi->z;

	// ��X��ת
	float x2 = x1;
	float y2 = cPitch * y1 - sPitch * z1;
	float z2 = sPitch * y1 + cPitch * z1;

	// ��Y��ת
	po->x = cYaw * x2 + sYaw * z2 + tX;
	po->y = y2 + tY;
	po->z = -sYaw * x2 + cYaw * z2 + tZ;
	po->intensity = pi->intensity;

}

void CMapOptimization::CornerOptimization(int iterCount)
{
	UpdatePointAssociateToMapSinCos();
	for (int i = 0; i < m_LaserCloudCornerLastDSNum; i++) {//m_LaserCloudCornerLastDSNum��ǰ�ؼ�֡������������

		//Դ���ƣ��Ե�ǰ֡�����״��Ե�㽵������
		pointOri = m_LaserCloudCornerLastDS->points[i];

		//����ǰ���������transformTobeMapped,�ǵ�ǰ֡�����״�����������ϵ�µ����꣬�ѵ�ǰ֡����ת����������ϵ��
		PointAssociateToMap(&pointOri, &pointSel);

		//if (iterCount % 5 == 0) {

			//kd��������5��������
			m_KdtreeCornerFromMap->nearestKSearch(pointSel, 5, m_vPointSearchInd, m_vPointSearchSqDis);

			//ֻ������ĵ㶼��һ����ֵ�ڣ�1�ף��Ž��м���
			if (m_vPointSearchSqDis[4] < 1.0) {
				float cx = 0, cy = 0, cz = 0;
				for (int j = 0; j < 5; j++) {
					cx += m_LaserCloudCornerFromMapDS->points[m_vPointSearchInd[j]].x;
					cy += m_LaserCloudCornerFromMapDS->points[m_vPointSearchInd[j]].y;
					cz += m_LaserCloudCornerFromMapDS->points[m_vPointSearchInd[j]].z;
				}
				//����5�㣨x,y,z��������ƽ��ֵ
				cx /= 5; cy /= 5;  cz /= 5;

				float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
				//��������Э�������
				for (int j = 0; j < 5; j++) {
					float ax = m_LaserCloudCornerFromMapDS->points[m_vPointSearchInd[j]].x - cx;
					float ay = m_LaserCloudCornerFromMapDS->points[m_vPointSearchInd[j]].y - cy;
					float az = m_LaserCloudCornerFromMapDS->points[m_vPointSearchInd[j]].z - cz;

					a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
					a22 += ay * ay; a23 += ay * az;
					a33 += az * az;
				}
				a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

				matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
				matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
				matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;
				//��Э������������ֵ����������
				//��ȡ������ֵ�ǰ��ս������е�,�������һ����Ե������������һ������ֵԶ�������������� 
				//�������һ��ƽ����������ô����һ������ֵԶС��������������ֵ�� 
				cv::eigen(matA1, matD1, matV1);
				//�ж���������:
				if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

					float x0 = pointSel.x;
					float y0 = pointSel.y;
					float z0 = pointSel.z;
					float x1 = cx + 0.1 * matV1.at<float>(0, 0);//x1=��ֵ+0.1*��������(0,0)	�������ֵ��Ӧ������������ʾֱ�ߵķ���
					float y1 = cy + 0.1 * matV1.at<float>(0, 1);
					float z1 = cz + 0.1 * matV1.at<float>(0, 2);
					float x2 = cx - 0.1 * matV1.at<float>(0, 0);
					float y2 = cy - 0.1 * matV1.at<float>(0, 1);
					float z2 = cz - 0.1 * matV1.at<float>(0, 2);

					float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
						* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
						+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
						* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

					float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

					float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

					float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
						- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

					float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
						+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

					float ld2 = a012 / l12;

					float s = 1 - 0.9 * fabs(ld2);//��Ȩ

					coeff.x = s * la;
					coeff.y = s * lb;
					coeff.z = s * lc;
					coeff.intensity = s * ld2;

					if (s > 0.1) {
						m_laserCloudOri->push_back(pointOri);
						m_coeffSel->push_back(coeff);
					}
				}
			}
		//}
	}
}

void CMapOptimization::SurfOptimization(int iterCount)
{
	UpdatePointAssociateToMapSinCos();
	for (int i = 0; i < m_LaserCloudSurfLastDSNum; i++) {
		pointOri = m_LaserCloudSurfLastDS->points[i];

		PointAssociateToMap(&pointOri, &pointSel);

		//if (iterCount % 5 == 0) {
			//kd��������5��������
			m_KdtreeSurfFromMap->nearestKSearch(pointSel, 5, m_vPointSearchInd, m_vPointSearchSqDis);

			if (m_vPointSearchSqDis[4] < 1.0) {
				for (int j = 0; j < 5; j++) {
					matA0.at<float>(j, 0) = m_LaserCloudSurfFromMapDS->points[m_vPointSearchInd[j]].x;
					matA0.at<float>(j, 1) = m_LaserCloudSurfFromMapDS->points[m_vPointSearchInd[j]].y;
					matA0.at<float>(j, 2) = m_LaserCloudSurfFromMapDS->points[m_vPointSearchInd[j]].z;
				}
				//matA0 5*3 matB0 5*1 cv::solve���matA0*matX0=matB0 ������matX0
				//�ռ��е�ƽ�淽�̿���дΪ��AX+BY+CZ+D=0 ��D=1 ����ٽ�����루X,Y,Z�������A,B,C
				//matX0:��matA0�еĵ㹹�ɵ�ƽ��ķ�����=[A,B,C]
				cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

				//��[pa,pb,pc,pd]���е�λ��
				float pa = matX0.at<float>(0, 0);
				float pb = matX0.at<float>(1, 0);
				float pc = matX0.at<float>(2, 0);
				float pd = 1;


				float ps = sqrt(pa * pa + pb * pb + pc * pc);
				pa /= ps; pb /= ps; pc /= ps; pd /= ps;

				bool planeValid = true;
				///���ķ���������ÿ������ˣ�������ǲ�������Χ�ڡ�������̫��Ͳ��ѵ�ǰ��pointSel�ŵ�������ȥ��
				for (int j = 0; j < 5; j++) {
					if (fabs(pa * m_LaserCloudSurfFromMapDS->points[m_vPointSearchInd[j]].x +
						pb * m_LaserCloudSurfFromMapDS->points[m_vPointSearchInd[j]].y +
						pc * m_LaserCloudSurfFromMapDS->points[m_vPointSearchInd[j]].z + pd) > 0.2) {
						planeValid = false;
						break;
					}
				}

				if (planeValid) {
					//�㵽ƽ��ľ��룺d=|A��X1+B��Y1+C��Z1+D|�¡�(A��A+B��B+C��C) 
					float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

					float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
						+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

					coeff.x = s * pa;
					coeff.y = s * pb;
					coeff.z = s * pc;
					coeff.intensity = s * pd2;

					if (s > 0.1) {
						m_laserCloudOri->push_back(pointOri);
						m_coeffSel->push_back(coeff);
					}
				}
			}
		//}
	}
}

int CMapOptimization::LMOptimization(int iterCount)
{
	float srx = sin(m_TransformTobeMapped[0]);
	float crx = cos(m_TransformTobeMapped[0]);
	float sry = sin(m_TransformTobeMapped[1]);
	float cry = cos(m_TransformTobeMapped[1]);
	float srz = sin(m_TransformTobeMapped[2]);
	float crz = cos(m_TransformTobeMapped[2]);

	int laserCloudSelNum = m_laserCloudOri->points.size();
	//�����ǰֻ֡��С��50����������Ż���ֱ���˳�
	if (laserCloudSelNum < 50) {
		return false;
	}

	cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
	for (int i = 0; i < laserCloudSelNum; i++) {
		pointOri = m_laserCloudOri->points[i];
		coeff = m_coeffSel->points[i];

		float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
			+ (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
			+ (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

		float ary = ((cry*srx*srz - crz * sry)*pointOri.x
			+ (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
			+ ((-cry * crz - srx * sry*srz)*pointOri.x
				+ (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

		float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
			+ (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
			+ ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

		matA.at<float>(i, 0) = arx;
		matA.at<float>(i, 1) = ary;
		matA.at<float>(i, 2) = arz;
		matA.at<float>(i, 3) = coeff.x;
		matA.at<float>(i, 4) = coeff.y;
		matA.at<float>(i, 5) = coeff.z;
		matB.at<float>(i, 0) = -coeff.intensity;
	}
	cv::transpose(matA, matAt);
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);//���matX������matAtA*matX=matAtB

	if (iterCount == 0) {
		cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

		cv::eigen(matAtA, matE, matV);
		matV.copyTo(matV2);

		isDegenerate = false;
		float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
		for (int i = 5; i >= 0; i--) {
			if (matE.at<float>(0, i) < eignThre[i]) {//����ֵ��������Ϊ100��С�����ֵ��Ϊ���˻���
				for (int j = 0; j < 6; j++) {
					matV2.at<float>(i, j) = 0;
				}
				isDegenerate = true;
			}
			else {
				break;
			}
		}
		matP = matV.inv() * matV2;
	}

	if (isDegenerate) {
		cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
		matX.copyTo(matX2);
		matX = matP * matX2;
	}

	//����transformTobeMapped������lidar��ǰ֡��ͼ�ڵ��λ��
	m_TransformTobeMapped[0] += matX.at<float>(0, 0);
	m_TransformTobeMapped[1] += matX.at<float>(1, 0);
	m_TransformTobeMapped[2] += matX.at<float>(2, 0);
	m_TransformTobeMapped[3] += matX.at<float>(3, 0);
	m_TransformTobeMapped[4] += matX.at<float>(4, 0);
	m_TransformTobeMapped[5] += matX.at<float>(5, 0);

	float deltaR = sqrt(
		pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
		pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
		pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
	float deltaT = sqrt(
		pow(matX.at<float>(3, 0) * 100, 2) +
		pow(matX.at<float>(4, 0) * 100, 2) +
		pow(matX.at<float>(5, 0) * 100, 2));

	if (deltaR < 0.05 && deltaT < 0.05) {
		//��ת���ֵ�ģ��С��0.05��ƽ�Ʋ��ֵ�ģ��ҲС��0.05��L-M��������
		return 2;
	}
	return 1;
}

void CMapOptimization::UpdatePointAssociateToMapSinCos()
{
	
	cPitch = cos(m_TransformTobeMapped[0]);
	sPitch = sin(m_TransformTobeMapped[0]);
	cYaw = cos(m_TransformTobeMapped[1]);
	sYaw = sin(m_TransformTobeMapped[1]);
	cRoll = cos(m_TransformTobeMapped[2]);
	sRoll = sin(m_TransformTobeMapped[2]);
	tX = m_TransformTobeMapped[3];
	tY = m_TransformTobeMapped[4];
	tZ = m_TransformTobeMapped[5];
}

void CMapOptimization::UpdateTransformPointCloudSinCos(PointTypePose * tIn)
{
	ctRoll = cos(tIn->roll);
	stRoll = sin(tIn->roll);

	ctPitch = cos(tIn->pitch);
	stPitch = sin(tIn->pitch);

	ctYaw = cos(tIn->yaw);
	stYaw = sin(tIn->yaw);

	tInX = tIn->x;
	tInY = tIn->y;
	tInZ = tIn->z;
}

bool CMapOptimization::Update()
{
	//��tobemapped����transformSum
	m_LidarOdometry->m_TransformSum[m_SystemCount - 1].pitch = m_TransformTobeMapped[0];
	m_LidarOdometry->m_TransformSum[m_SystemCount - 1].yaw = m_TransformTobeMapped[1];
	m_LidarOdometry->m_TransformSum[m_SystemCount - 1].roll = m_TransformTobeMapped[2];
	m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posX = m_TransformTobeMapped[3];
	m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posY = m_TransformTobeMapped[4];
	m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posZ = m_TransformTobeMapped[5];

	//����TransformSum
	m_LidarOdometry->transformSum[0] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].pitch;
	m_LidarOdometry->transformSum[1] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].yaw;
	m_LidarOdometry->transformSum[2] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].roll;
	m_LidarOdometry->transformSum[3] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posX;
	m_LidarOdometry->transformSum[4] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posY;
	m_LidarOdometry->transformSum[5] = m_LidarOdometry->m_TransformSum[m_SystemCount - 1].posZ;

	return true;
}

int CMapOptimization::SaveKeyFrames()
{

	// ����ؼ�֡��λ��
	PointType thisPose3D;
	PointTypePose thisPose6D;

	thisPose3D.x = m_TransformTobeMapped[3];
	thisPose3D.y = m_TransformTobeMapped[4];
	thisPose3D.z = m_TransformTobeMapped[5];
	thisPose3D.intensity = m_CloudKeyPoses3D->points.size();//��ǰ֡�ǹؼ�֡������ؼ�֡�б�
	m_CloudKeyPoses3D->push_back(thisPose3D);

	m_PreviousRobotPosPoint = m_CurrentRobotPosPoint;

	thisPose6D.x = m_TransformTobeMapped[3];
	thisPose6D.y = m_TransformTobeMapped[4];
	thisPose6D.z = m_TransformTobeMapped[5];
	thisPose6D.pitch = m_TransformTobeMapped[0];
	thisPose6D.yaw = m_TransformTobeMapped[1];
	thisPose6D.roll  =	m_TransformTobeMapped[2];
	thisPose6D.intensity = m_CloudKeyPoses6D->points.size();
	m_CloudKeyPoses6D->push_back(thisPose6D);
	
	// ����ǰ�ؼ�֡���������ƣ��洢��cornerCloudKeyFrames��surfCloudKeyFrames

	pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
	pcl::copyPointCloud(*m_LaserCloudCornerLastDS, *thisCornerKeyFrame);

	m_CornerCloudKeyFrames.push_back(thisCornerKeyFrame);


	pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
	pcl::copyPointCloud(*m_LaserCloudSurfLastDS, *thisSurfKeyFrame);
	
	m_SurfCloudKeyFrames.push_back(thisSurfKeyFrame);


	return 0;
}

bool CMapOptimization::PublishPointCloudMap()
{
	for (int i = 0; i < m_CloudKeyPoses6D->size(); i++)
	{
		// �ҵ��ؼ�֡����,��ת������������ϵ��
		PointTypePose thisTransformation = m_CloudKeyPoses6D->points[i];
		UpdateTransformPointCloudSinCos(&thisTransformation);

		m_DownSizeFilterPointCloudMap.setInputCloud(m_CornerCloudKeyFrames[i]);
		m_DownSizeFilterPointCloudMap.filter(*m_PointCloudCornerMapDS);
		m_PointCloudMap.push_back(TransformPointCloud(m_PointCloudCornerMapDS));

		m_DownSizeFilterPointCloudMap.setInputCloud(m_SurfCloudKeyFrames[i]);
		m_DownSizeFilterPointCloudMap.filter(*m_PointCloudSurfMapDS);
		m_PointCloudMap.push_back(TransformPointCloud(m_PointCloudSurfMapDS));
	}

	return true;
}


