#include "CLIOApplication.h"
#include <time.h>　　

CLIOAPP::CLIOAPP()
{
	m_PCFFStream.m_LIOOpt = &(this->m_LIOOpt);

	m_FeatureExtract.m_LIOOpt = &(this->m_LIOOpt);
	m_FeatureExtract.m_PCFFStream = &(this->m_PCFFStream);

	m_LidarOdometry.m_LIOOpt = &(this->m_LIOOpt);
	m_LidarOdometry.m_PCFFStream = &(this->m_PCFFStream);
	m_LidarOdometry.m_FeatureExtraction = &(this->m_FeatureExtract);

	m_MapOptimization.m_LIOOpt = &(this->m_LIOOpt);
	m_MapOptimization.m_PCFFStream = &(this->m_PCFFStream);
	m_MapOptimization.m_FeatureExtraction = &(this->m_FeatureExtract);
	m_MapOptimization.m_LidarOdometry = &(this->m_LidarOdometry);
}

CLIOAPP::~CLIOAPP()
{
}

int CLIOAPP::RunLIOApp(string optFile)
{

	// 1、读取配置文件
	m_LIOOpt.ReadOptFile(optFile);
	
	m_PCFFStream.Init();

	// 2、处理点云文件
	FILE *flog;
	string LogFile,OutPutPath;
	OutPutPath = m_LIOOpt.m_PrjDataPath + "\\output";
	CreateDirectory(OutPutPath.c_str(), NULL);

	LogFile = OutPutPath + "\\log.txt";
	fopen_s(&flog, LogFile.c_str(), "wt");

	clock_t start1, start2, start3, end1, end2, end3;

	while (true)
	{
		gs_SystemCount++;

		//if (gs_SystemCount > m_PCFFStream.m_vPointCloudFileNames.size())
		if (gs_SystemCount > 100)
		{
			break;
		}

		cout << "Processing " << gs_SystemCount << "th point cloud" << endl;
		m_PCFFStream.ReadOnePCFile(gs_SystemCount);
		// 点云分割
		start1 = clock();
		m_FeatureExtract.RunCloudSegmentation(gs_SystemCount);
		end1 = (clock() - start1);
		cout << "RunCloudSegmentation time comsumption is " << end1 << endl;
		if (gs_SystemCount > 1) 
			fprintf(flog, "%8ld", end1);		

		if (gs_SystemCount > 1)
		{
			// 前端里程计
			start2 = clock();
			m_LidarOdometry.RunLidarOdometry(gs_SystemCount);
			end2 = (clock() - start2);
			cout << "RunLidarOdometry time comsumption is " << end2 << endl;
			fprintf(flog, "%8ld", end2);

			// 后端优化
			/*start3 = clock();
			m_MapOptimization.RunMapOptimization(gs_SystemCount);
			end3 = (clock() - start3);
			cout << "RunMapOptimization time comsumption is " << end3 << endl;
			fprintf(flog, "%8ld\n", end3);*/
		}

		// 处理完最后一帧点云后 发布地图
		//if (gs_SystemCount == m_PCFFStream.m_vPointCloudFileNames.size())
		if (gs_SystemCount == 100)
			m_MapOptimization.PublishPointCloudMap();

		cout << "******************************************************" << endl;
	}
	fclose(flog);
	
	// 3、输出位姿结果
	FILE * fout;
	Pose3D Pose;
	string resultfilename;
	resultfilename = OutPutPath + "\\result.txt";
	fopen_s(&fout, resultfilename.c_str(), "wt");

	for (int i = 0; i < m_LidarOdometry.m_TransformSum.size(); i++)
	{
		Pose = m_LidarOdometry.m_TransformSum[i];
		// 基于实验室坐标系
		fprintf(fout, "%f\t%f\t%f\t%f\t%f\t%f\n", Pose.posX, Pose.posZ, Pose.posY, Pose.yaw, Pose.pitch, Pose.roll);
		// 基于LOAM坐标系
		//fprintf(fout, "%f\t%f\t%f\t%f\t%f\t%f\n", -Pose.posX, Pose.posY, Pose.posZ, Pose.yaw, Pose.pitch, Pose.roll);
	}
	fclose(fout);

	// 4、输出建图结果
	FILE *fmap;
	string mapfilename;
	mapfilename = OutPutPath + "\\map.txt";
	fopen_s(&fmap, mapfilename.c_str(), "wt");

	PointType mapPoint;
	float intense = 1;
	for (int i = 0; i < m_MapOptimization.m_PointCloudMap.size(); i++)
	{
		for (int j = 0; j < m_MapOptimization.m_PointCloudMap[i]->size(); j++)
		{			
			mapPoint = m_MapOptimization.m_PointCloudMap[i]->points[j];
			
			fprintf(fmap, "%f\t\t%f\t\t%f\t\t%f\n", -mapPoint.x, mapPoint.y, mapPoint.z, intense);
		}		
	}
	fclose(fmap);

	return 0;
}

int CLIOAPP::ClearLIOApp()
{
	return 0;
}
