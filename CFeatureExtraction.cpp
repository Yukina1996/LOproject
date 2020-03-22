#include "CFeatureExtraction.h"

CFeatureExtraction::CFeatureExtraction()
{
	m_fullPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_rawPointCloudCur.reset(new pcl::PointCloud < pcl::PointXYZI>());
	m_segmentedPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_cornerPointsSharp.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_cornerPointsLessSharp.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_surfPointsFlat.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_surfPointsLessFlat.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

CFeatureExtraction::~CFeatureExtraction()
{
	vector<smoothness_t>().swap(m_vCloudSmoothness);
	vector<float>().swap(m_vCloudCurvature);
	vector<int>().swap(m_vCloudNeighborPicked);
	vector<int>().swap(m_vCloudLabel);
}

bool CFeatureExtraction::RunCloudSegmentation(const int SystemCount)
{
	m_SystemCount = SystemCount;

	InitValues();

	FindStartEndAngle();

	ProjectPointCloud();

	GroundRemoval(); 

	CloudSegmentation();

	AdjustDistortion();

	CalculateSmoothness();

	MarkOccludedPoints();

	ExtractFeatures();

	SaveSegPointCloud();

	ResetParameters();

	return true;
}

bool CFeatureExtraction::InitValues()
{
	m_rawPointCloudCur = m_PCFFStream->m_PCL_PointCloud;		///< 从FFSTREAM中读入原始点云
	m_segmentedPoints->clear();
	m_cornerPointsSharp->clear();
	m_cornerPointsLessSharp->clear();
	m_surfPointsFlat->clear();
	m_surfPointsLessFlat->clear();

	vector<PointCloudType>().swap(m_PointCloudMapCur);			///< 清空vector

	m_segmentedPCFile = m_LIOOpt->m_SegmentedPCPath + "\\" + m_PCFFStream->m_vPointCloudFileNames[m_SystemCount - 1];
	m_segmentedPCFile = m_segmentedPCFile.substr(0, m_segmentedPCFile.rfind("\n"));

	m_rangeMat = cv::Mat(m_LIOOpt->m_N_SCAN, m_LIOOpt->m_Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
	m_groundMat = cv::Mat(m_LIOOpt->m_N_SCAN, m_LIOOpt->m_Horizon_SCAN, CV_8S, cv::Scalar::all(0));
	m_labelMat = cv::Mat(m_LIOOpt->m_N_SCAN, m_LIOOpt->m_Horizon_SCAN, CV_32S, cv::Scalar::all(0));
	m_labelCount = 1;

	pcl::PointXYZI nanPoint;
	nanPoint.x = std::numeric_limits<float>::quiet_NaN();
	nanPoint.y = std::numeric_limits<float>::quiet_NaN();
	nanPoint.z = std::numeric_limits<float>::quiet_NaN();
	nanPoint.intensity = -1;
	long MaxPtsNum;
	MaxPtsNum = m_LIOOpt->m_N_SCAN*m_LIOOpt->m_Horizon_SCAN;

	m_fullPoints->points.resize(MaxPtsNum);		//eg N_SCAN=16 Horizon_SCAN=1800 代表一圈完整的点云
	std::fill(m_fullPoints->points.begin(), m_fullPoints->points.end(), nanPoint);
		
	std::pair<int8_t, int8_t> neighbor;
	neighbor.first = -1; neighbor.second = 0; m_vNeighborIterator.push_back(neighbor);
	neighbor.first = 0; neighbor.second = 1; m_vNeighborIterator.push_back(neighbor);
	neighbor.first = 0; neighbor.second = -1; m_vNeighborIterator.push_back(neighbor);
	neighbor.first = 1; neighbor.second = 0; m_vNeighborIterator.push_back(neighbor);

	m_allPushedIndX = new uint16_t[MaxPtsNum];
	m_allPushedIndY = new uint16_t[MaxPtsNum];
	m_queueIndX = new uint16_t[MaxPtsNum];
	m_queueIndY = new uint16_t[MaxPtsNum];

	m_vCloudSmoothness.resize(MaxPtsNum);
	m_vCloudCurvature.resize(MaxPtsNum);
	m_vCloudNeighborPicked.resize(MaxPtsNum);
	m_vCloudLabel.resize(MaxPtsNum);

	m_SegCloudInfo.startRingIndex.resize(m_LIOOpt->m_N_SCAN);
	m_SegCloudInfo.endRingIndex.resize(m_LIOOpt->m_N_SCAN);
	m_SegCloudInfo.segmentedCloudGroundFlag.resize(MaxPtsNum);
	m_SegCloudInfo.segmentedCloudColInd.resize(MaxPtsNum);
	m_SegCloudInfo.segmentedCloudRange.resize(MaxPtsNum);

	return true;
}

bool CFeatureExtraction::FindStartEndAngle()
{
	m_SegCloudInfo.startOrientation= 
		-atan2(m_rawPointCloudCur->points[0].y, m_rawPointCloudCur->points[0].x);
	m_SegCloudInfo.endOrientation= 
		-atan2(m_rawPointCloudCur->points[m_rawPointCloudCur->points.size() - 1].y,
		m_rawPointCloudCur->points[m_rawPointCloudCur->points.size() - 1].x) + 2 * M_PI;
	if (m_SegCloudInfo.endOrientation - m_SegCloudInfo.startOrientation > 3 * M_PI) 
	{
		m_SegCloudInfo.endOrientation -= 2 * M_PI;
	}
	else if (m_SegCloudInfo.endOrientation - m_SegCloudInfo.startOrientation < M_PI)
		m_SegCloudInfo.endOrientation += 2 * M_PI;

	m_SegCloudInfo.orientationDiff = m_SegCloudInfo.endOrientation - m_SegCloudInfo.startOrientation;

	return true;
}

bool CFeatureExtraction::ProjectPointCloud()
{
	float verticalAngle, horizonAngle, range;
	size_t rowIdn, columnIdn, index, cloudSize;
	pcl::PointXYZI thisPoint;

	cloudSize = m_rawPointCloudCur->points.size();

	double ang_bottom = m_LIOOpt->m_Ang_Bottom;
	int Horizon_SCAN = m_LIOOpt->m_Horizon_SCAN;
	int N_SCAN = m_LIOOpt->m_N_SCAN;
	double ang_res_x = m_LIOOpt->m_Ang_Res_X;
	double ang_res_y = m_LIOOpt->m_Ang_Res_Y;

	for (size_t i = 0; i < cloudSize; i++)
	{
		thisPoint.x = m_rawPointCloudCur->points[i].x;
		thisPoint.y = m_rawPointCloudCur->points[i].y;
		thisPoint.z = m_rawPointCloudCur->points[i].z;		

		verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
		rowIdn = (verticalAngle + ang_bottom) / ang_res_y;		// -15.1°的扫描线rowIdn=0
		if (rowIdn < 0 || rowIdn >= N_SCAN)
			continue;

		horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;	// 水平角：相对于y轴的方位角

		columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;	// columnIdn
		if (columnIdn >= Horizon_SCAN)
			columnIdn -= Horizon_SCAN;

		if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
			continue;

		range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
		m_rangeMat.at<float>(rowIdn, columnIdn) = range;

		thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

		index = columnIdn + rowIdn * Horizon_SCAN;
		m_fullPoints->points[index] = thisPoint;
	}

	return true;
}

bool CFeatureExtraction::GroundRemoval()
{
	size_t lowerInd, upperInd;
	float diffX, diffY, diffZ, angle;
	int Horizon_SCAN = m_LIOOpt->m_Horizon_SCAN;
	int groundScanInd = m_LIOOpt->m_GroundScanInd;
	

	for (size_t j = 0; j < Horizon_SCAN; ++j) {	//  Horizon_SCAN = 1800
		for (size_t i = 0; i < groundScanInd; ++i) {	// groundScanInd = 7

			lowerInd = j + (i)*Horizon_SCAN;
			upperInd = j + (i + 1)*Horizon_SCAN;

			if (m_fullPoints->points[lowerInd].intensity == -1 ||
				m_fullPoints->points[upperInd].intensity == -1) {
				m_groundMat.at<int8_t>(i, j) = -1;
				continue;
			}

			diffX = m_fullPoints->points[upperInd].x - m_fullPoints->points[lowerInd].x;
			diffY = m_fullPoints->points[upperInd].y - m_fullPoints->points[lowerInd].y;
			diffZ = m_fullPoints->points[upperInd].z - m_fullPoints->points[lowerInd].z;

			angle = atan2(diffZ, sqrt(diffX*diffX + diffY * diffY)) * 180 / M_PI;

			/*Since sloped terrain is common in many environments,
			we do not assume the ground is flat
			允许的地面斜度是10°
			*/
			if (abs(angle - m_LIOOpt->m_SensorMountAngle) <= 10) {	// sensorMountAngle = 0.0
				m_groundMat.at<int8_t>(i, j) = 1;
				m_groundMat.at<int8_t>(i + 1, j) = 1;
			}
		}
	}

	for (size_t i = 0; i < m_LIOOpt->m_N_SCAN; ++i) {
		for (size_t j = 0; j < Horizon_SCAN; ++j) {
			if (m_groundMat.at<int8_t>(i, j) == 1 || m_rangeMat.at<float>(i, j) == FLT_MAX) {
				//是地面点或者没有距离值，标记该点不是分割点云
				m_labelMat.at<int>(i, j) = -1;
			}
		}
	}

	return true;
}

bool CFeatureExtraction::CloudSegmentation()
{
	struct PointCloudType PtsCloudType;

	// 这是在排除地面点与异常点之后，逐一检测邻点特征并生成局部特征
	for (size_t i = 0; i < m_LIOOpt->m_N_SCAN; ++i)
		for (size_t j = 0; j < m_LIOOpt->m_Horizon_SCAN; ++j)
			if (m_labelMat.at<int>(i, j) == 0)
			{
				labelComponents(i, j);
			}

	int sizeOfSegCloud = 0;
	for (size_t i = 0; i < m_LIOOpt->m_N_SCAN; ++i) {

		m_SegCloudInfo.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

		for (size_t j = 0; j < m_LIOOpt->m_Horizon_SCAN; ++j) {
			// 如果是被认可的特征点或者是地面点，就可以纳入被分割点云
			if (m_labelMat.at<int>(i, j) > 0 || m_groundMat.at<int8_t>(i, j) == 1) {

				if (m_groundMat.at<int8_t>(i, j) == 1) {
					// 地面点云每隔5个点纳入被分割点云
					if (j % 5 != 0 && j > 5 && j < m_LIOOpt->m_Horizon_SCAN - 5)
						continue;
				}
				// segMsg是自定义rosmsg
				// 是否是地面点
				m_SegCloudInfo.segmentedCloudGroundFlag[sizeOfSegCloud] = (m_groundMat.at<int8_t>(i, j) == 1);
				// 当前水平方向上的列数
				m_SegCloudInfo.segmentedCloudColInd[sizeOfSegCloud] = j;
				// 深度
				m_SegCloudInfo.segmentedCloudRange[sizeOfSegCloud] = m_rangeMat.at<float>(i, j);
				// 把当前点纳入分割点云中
				m_segmentedPoints->push_back(m_fullPoints->points[j + i * m_LIOOpt->m_Horizon_SCAN]);
				++sizeOfSegCloud;

				PtsCloudType.x = m_fullPoints->points[j + i * m_LIOOpt->m_Horizon_SCAN].x;
				PtsCloudType.y = m_fullPoints->points[j + i * m_LIOOpt->m_Horizon_SCAN].y;
				PtsCloudType.z = m_fullPoints->points[j + i * m_LIOOpt->m_Horizon_SCAN].z;
				PtsCloudType.rowID = i; PtsCloudType.colID = j;				
				m_PointCloudMapCur.push_back(PtsCloudType);				
			}
		}

		m_SegCloudInfo.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
	}

	return true;
}

void CFeatureExtraction::labelComponents(int row, int col)
{
	float d1, d2, alpha, angle;
	int fromIndX, fromIndY, thisIndX, thisIndY;
	int N_SCAN = m_LIOOpt->m_N_SCAN;

	bool lineCountFlag[64] = { false };				///< 最大处理64线激光雷达

	m_queueIndX[0] = row;	// uint16_t *queueIndX
	m_queueIndY[0] = col; // uint16_t *queueIndY
	int queueSize = 1;
	int queueStartInd = 0;
	int queueEndInd = 1;

	m_allPushedIndX[0] = row;	// uint16_t *allPushedIndX
	m_allPushedIndY[0] = col;	// uint16_t *allPushedIndY
	int allPushedIndSize = 1;

	// queueSize指的是在特征处理时还未处理好的点的数量
	// 该while循环是在尝试检测该特定点的周围的点的几何特征
	while (queueSize > 0) {
		fromIndX = m_queueIndX[queueStartInd];
		fromIndY = m_queueIndY[queueStartInd];
		--queueSize;	// queueSize=0
		++queueStartInd;// queueStartInd=1
		m_labelMat.at<int>(fromIndX, fromIndY) = m_labelCount;

		// 检查上下左右四个邻点
		for (auto iter = m_vNeighborIterator.begin(); iter != m_vNeighborIterator.end(); ++iter) {

			thisIndX = fromIndX + (*iter).first;
			thisIndY = fromIndY + (*iter).second;

			if (thisIndX < 0 || thisIndX >= N_SCAN)
				continue;

			if (thisIndY < 0)
				thisIndY = m_LIOOpt->m_Horizon_SCAN - 1;
			if (thisIndY >= m_LIOOpt->m_Horizon_SCAN)
				thisIndY = 0;

			if (m_labelMat.at<int>(thisIndX, thisIndY) != 0)
				continue;

			// d1与d2分别是该特定点与某邻点的深度
			d1 = std::max(m_rangeMat.at<float>(fromIndX, fromIndY),
				m_rangeMat.at<float>(thisIndX, thisIndY));
			d2 = std::min(m_rangeMat.at<float>(fromIndX, fromIndY),
				m_rangeMat.at<float>(thisIndX, thisIndY));

			// 该迭代器的first是0则是水平方向上的邻点，否则是竖直方向上的
			if ((*iter).first == 0)
				alpha = m_LIOOpt->m_Ang_Res_X / 180 * M_PI;	// 水平分辨率(rad) 

			else
				alpha = m_LIOOpt->m_Ang_Res_Y / 180 * M_PI;	// 垂直分辨率(rad) 

			// 这个angle其实是该特定点与某邻点的连线与XOZ平面的夹角，这个夹角代表了局部特征的敏感性
			angle = atan2(d2*sin(alpha), (d1 - d2 * cos(alpha)));

			// 如果夹角大于60°，则将这个邻点纳入到局部特征中，该邻点可以用来配准使用
			if (angle > SegmentTheta) {

				m_queueIndX[queueEndInd] = thisIndX;// queueIndx[1]=thisIndX
				m_queueIndY[queueEndInd] = thisIndY;
				++queueSize;	// queueSize=1
				++queueEndInd;	// queueEndInd=2

				m_labelMat.at<int>(thisIndX, thisIndY) = m_labelCount;
				lineCountFlag[thisIndX] = true;

				m_allPushedIndX[allPushedIndSize] = thisIndX;
				m_allPushedIndY[allPushedIndSize] = thisIndY;
				++allPushedIndSize;
			}
		}
	}


	bool feasibleSegment = false;
	if (allPushedIndSize >= 30)			// 当邻点数目达到30后，则该帧雷达点云的几何特征配置成功
		feasibleSegment = true;
	else if (allPushedIndSize >= SegmentValidPointNum) {// segmentValidPointNum = 5
		int lineCount = 0;
		for (size_t i = 0; i < N_SCAN; ++i)
			if (lineCountFlag[i] == true)
				++lineCount;
		if (lineCount >= SegmentValidLineNum)
			feasibleSegment = true;
	}

	if (feasibleSegment == true) {
		++m_labelCount;
	}
	else {
		for (size_t i = 0; i < allPushedIndSize; ++i) {
			m_labelMat.at<int>(m_allPushedIndX[i], m_allPushedIndY[i]) = 999999;
		}
	}

}

void CFeatureExtraction::ResetParameters()
{
	// 清空变量
	delete[] m_allPushedIndX;
	delete[] m_allPushedIndY;
	delete[] m_queueIndX;
	delete[] m_queueIndY;

	vector<smoothness_t>().swap(m_vCloudSmoothness);
	vector<float>().swap(m_vCloudCurvature);
	vector<int>().swap(m_vCloudNeighborPicked);
	vector<int>().swap(m_vCloudLabel);

	vector<pair<uint8_t, uint8_t >>().swap(m_vNeighborIterator);
}

bool CFeatureExtraction::SaveSegPointCloud()
{
	FILE *fout_t, *fout;
	PointCloudType PtsCloudType;
	fopen_s(&fout, m_segmentedPCFile.c_str(), "wb");	///< 分割点云输出到bin文件

	//string segmentedPCFile_t = m_segmentedPCFile.substr(0, m_segmentedPCFile.rfind(".bin"));
	//segmentedPCFile_t = segmentedPCFile_t + ".txt";
	//fopen_s(&fout_t, segmentedPCFile_t.c_str(), "wt");///< 分割点云输出到txt文件

	for (size_t i = 0; i < m_PointCloudMapCur.size(); i++)
	{
		PtsCloudType = m_PointCloudMapCur[i];
		fwrite(&PtsCloudType, sizeof(PtsCloudType), 1, fout);

		//fprintf(fout_t, "%f\t%f\t%f\t%d\t%d\t%d\n", 
		//	m_PointCloudMapCur[i].x, m_PointCloudMapCur[i].y,m_PointCloudMapCur[i].z, 
		//	m_PointCloudMapCur[i].rowID, m_PointCloudMapCur[i].colID, m_PointCloudMapCur[i].label);
	}
	fclose(fout);
	//fclose(fout_t);
	return true;
}


bool CFeatureExtraction::AdjustDistortion()
{
	bool halfPassed = false;
	int cloudSize = m_segmentedPoints->points.size();

	pcl::PointXYZI point;

	for (int i = 0; i < cloudSize; i++)
	{
		//point.x = m_segmentedPoints->points[i].y;///< !!!坐标转换!!!!转换为LOAM定义坐标系
		//point.y = m_segmentedPoints->points[i].z;///< !!!坐标转换!!!!
		//point.z = m_segmentedPoints->points[i].x;///< !!!坐标转换!!!!
		point.x = -m_segmentedPoints->points[i].y;	///< !!!坐标转换!!!!转换为实验室常用坐标系
		point.y = m_segmentedPoints->points[i].x;	///< !!!坐标转换!!!!
		point.z = m_segmentedPoints->points[i].z;	///< !!!坐标转换!!!!


		float ori = -atan2(point.x, point.z);
		if (!halfPassed) {
			if (ori < m_SegCloudInfo.startOrientation - M_PI / 2)
				ori += 2 * M_PI;
			else if (ori > m_SegCloudInfo.startOrientation + M_PI * 3 / 2)
				ori -= 2 * M_PI;

			if (ori - m_SegCloudInfo.startOrientation > M_PI)
				halfPassed = true;
		}
		else {
			ori += 2 * M_PI;

			if (ori < m_SegCloudInfo.endOrientation - M_PI * 3 / 2)
				ori += 2 * M_PI;
			else if (ori > m_SegCloudInfo.endOrientation + M_PI / 2)
				ori -= 2 * M_PI;
		}

		// relative time 某点在一个扫描周期中的相对时间
		float relTime = (ori - m_SegCloudInfo.startOrientation) / m_SegCloudInfo.orientationDiff;
		
		// 整数部分：rowIdn，小数部分：每个点扫描的时间（在startOri->endOri按照均匀划分）
		point.intensity = int(m_segmentedPoints->points[i].intensity) + m_LIOOpt->m_SampleTime * relTime;
		//point.intensity = int(m_segmentedPoints->points[i].intensity);

		m_segmentedPoints->points[i] = point;
		m_PointCloudMapCur[i].x = point.x;
		m_PointCloudMapCur[i].y = point.y;
		m_PointCloudMapCur[i].z = point.z;
		m_PointCloudMapCur[i].intensity = point.intensity;
	}

	return true;
}

bool CFeatureExtraction::CalculateSmoothness()
{
	int cloudSize = m_segmentedPoints->points.size();
	for (int i = 5; i < cloudSize - 5; i++) {

		float diffRange = m_SegCloudInfo.segmentedCloudRange[i - 5] + m_SegCloudInfo.segmentedCloudRange[i - 4]
			+ m_SegCloudInfo.segmentedCloudRange[i - 3] + m_SegCloudInfo.segmentedCloudRange[i - 2]
			+ m_SegCloudInfo.segmentedCloudRange[i - 1] - m_SegCloudInfo.segmentedCloudRange[i] * 10
			+ m_SegCloudInfo.segmentedCloudRange[i + 1] + m_SegCloudInfo.segmentedCloudRange[i + 2]
			+ m_SegCloudInfo.segmentedCloudRange[i + 3] + m_SegCloudInfo.segmentedCloudRange[i + 4]
			+ m_SegCloudInfo.segmentedCloudRange[i + 5];

		m_vCloudCurvature[i] = diffRange * diffRange;

		m_vCloudNeighborPicked[i] = 0;
		m_vCloudLabel[i] = 0;

		m_vCloudSmoothness[i].value = m_vCloudCurvature[i];
		m_vCloudSmoothness[i].ind = i;
	}

	return true;
}

bool CFeatureExtraction::MarkOccludedPoints()
{	
	int cloudSize = m_segmentedPoints->points.size();

	for (int i = 5; i < cloudSize - 6; ++i) {

		// depth1和depth2分别是两点的深度
		float depth1 = m_SegCloudInfo.segmentedCloudRange[i];
		float depth2 = m_SegCloudInfo.segmentedCloudRange[i + 1];

		// 将可能存在遮挡的点去除，将远侧的点视为瑕点
		// 比较点的列序号，去掉其中一侧的6个点
		int columnDiff = std::abs(int(m_SegCloudInfo.segmentedCloudColInd[i + 1] - m_SegCloudInfo.segmentedCloudColInd[i]));

		if (columnDiff < 10) {

			if (depth1 - depth2 > 0.3) {
				m_vCloudNeighborPicked[i - 5] = 1;
				m_vCloudNeighborPicked[i - 4] = 1;
				m_vCloudNeighborPicked[i - 3] = 1;
				m_vCloudNeighborPicked[i - 2] = 1;
				m_vCloudNeighborPicked[i - 1] = 1;
				m_vCloudNeighborPicked[i] = 1;
			}
			else if (depth2 - depth1 > 0.3) {
				m_vCloudNeighborPicked[i + 1] = 1;
				m_vCloudNeighborPicked[i + 2] = 1;
				m_vCloudNeighborPicked[i + 3] = 1;
				m_vCloudNeighborPicked[i + 4] = 1;
				m_vCloudNeighborPicked[i + 5] = 1;
				m_vCloudNeighborPicked[i + 6] = 1;
			}
		}
		// diff1和diff2是当前点距离前后两个点的距离
		float diff1 = std::abs(m_SegCloudInfo.segmentedCloudRange[i - 1] - m_SegCloudInfo.segmentedCloudRange[i]);
		float diff2 = std::abs(m_SegCloudInfo.segmentedCloudRange[i + 1] - m_SegCloudInfo.segmentedCloudRange[i]);
		// 如果当前点距离左右邻点都过远，则视其为瑕点，因为入射角可能太小导致误差较大
		if (diff1 > 0.02 * m_SegCloudInfo.segmentedCloudRange[i] && diff2 > 0.02 * m_SegCloudInfo.segmentedCloudRange[i])
			m_vCloudNeighborPicked[i] = 1;
	}

	return true;
}

bool CFeatureExtraction::ExtractFeatures()
{
	m_cornerPointsSharp->clear();
	m_cornerPointsLessSharp->clear();
	m_surfPointsFlat->clear();
	m_surfPointsLessFlat->clear();

	for (int i = 0; i < m_LIOOpt->m_N_SCAN; i++) {


		for (int j = 0; j < 6; j++) {

			int sp = (m_SegCloudInfo.startRingIndex[i] * (6 - j) + m_SegCloudInfo.endRingIndex[i] * j) / 6;
			int ep = (m_SegCloudInfo.startRingIndex[i] * (5 - j) + m_SegCloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

			if (sp >= ep)
				continue;

			std::sort(m_vCloudSmoothness.begin() + sp, m_vCloudSmoothness.begin() + ep, by_value());
			int largestPickedNum = 0;
			for (int k = ep; k >= sp; k--) {
				int ind = m_vCloudSmoothness[k].ind;
				// 非瑕点，曲率大于edgeThreshold（0.1），而且不能是地面点，则视为角点
				if (m_vCloudNeighborPicked[ind] == 0 &&
					m_vCloudCurvature[ind] > EdgeThreshold &&
					m_SegCloudInfo.segmentedCloudGroundFlag[ind] == false) {

					largestPickedNum++;
					if (largestPickedNum <= 2) {
						m_vCloudLabel[ind] = 2;
						m_cornerPointsSharp->push_back(m_segmentedPoints->points[ind]);
						m_cornerPointsLessSharp->push_back(m_segmentedPoints->points[ind]);
						m_PointCloudMapCur[ind].label = 2;
					}
					else if (largestPickedNum <= 20) {
						m_vCloudLabel[ind] = 1;
						m_cornerPointsLessSharp->push_back(m_segmentedPoints->points[ind]);
						m_PointCloudMapCur[ind].label = 1;
					}
					else {
						break;
					}

					// 对ind点周围的点是否能做为特征点进行判断
					m_vCloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++) {
						int columnDiff = std::abs(int(m_SegCloudInfo.segmentedCloudColInd[ind + l] - m_SegCloudInfo.segmentedCloudColInd[ind + l - 1]));
						if (columnDiff > 10)
							break;
						m_vCloudNeighborPicked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--) {
						int columnDiff = std::abs(int(m_SegCloudInfo.segmentedCloudColInd[ind + l] - m_SegCloudInfo.segmentedCloudColInd[ind + l + 1]));
						if (columnDiff > 10)
							break;
						m_vCloudNeighborPicked[ind + l] = 1;
					}
				}
			}

			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++) {
				int ind = m_vCloudSmoothness[k].ind;
				if (m_vCloudNeighborPicked[ind] == 0 &&
					m_vCloudCurvature[ind] < SurfThreshold &&
					m_SegCloudInfo.segmentedCloudGroundFlag[ind] == true) {

					m_vCloudLabel[ind] = -1;
					m_surfPointsFlat->push_back(m_segmentedPoints->points[ind]);
					m_PointCloudMapCur[ind].label = -1;		///< 面特征点曲率很小

					smallestPickedNum++;
					if (smallestPickedNum >= 4) {
						break;
					}

					// 对ind点周围的点是否能做为特征点进行判断
					m_vCloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++) {

						int columnDiff = std::abs(int(m_SegCloudInfo.segmentedCloudColInd[ind + l] - m_SegCloudInfo.segmentedCloudColInd[ind + l - 1]));
						if (columnDiff > 10)
							break;

						m_vCloudNeighborPicked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--) {

						int columnDiff = std::abs(int(m_SegCloudInfo.segmentedCloudColInd[ind + l] - m_SegCloudInfo.segmentedCloudColInd[ind + l + 1]));
						if (columnDiff > 10)
							break;

						m_vCloudNeighborPicked[ind + l] = 1;
					}
				}
			}

			for (int k = sp; k <= ep; k++) {
				if (m_vCloudLabel[k] <= 0) {
					//m_surfPointsLessFlatScan->push_back(m_segmentedPoints->points[k]);
					m_surfPointsLessFlat->push_back(m_segmentedPoints->points[k]);
					if (m_PointCloudMapCur[k].label == -1)
					{
						break;
					}
					m_PointCloudMapCur[k].label = 0;			///< 面特征点曲率比较小
				}
			}
		}
	}
	return true;
}
