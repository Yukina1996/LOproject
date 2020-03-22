/*
读取点云原始文件
*/
#ifndef LIODLL_CLIOFFSTREAM_H
#define LIODLL_CLIOFFSTREAM_H

#include "LIOCmnFunc.h"
#include "LIOBase.h"
#include "CLIOOption.h"
#include <io.h>

class CPointCloudFFSTREAM 
{
public:
	CPointCloudFFSTREAM();
	virtual ~CPointCloudFFSTREAM();

	CLIOOPTION* m_LIOOpt;

	bool Init();									///< 从点云文件夹中初始化当前所有点云的文件名
	int ReadOnePCFile(string pointcloud_filename);
	int ReadOnePCFile(const int SystemCount);
	int ReadOneSegmentedPCFile();


public:
	vector<vector<float>> m_PointCloud;								///< 点云原始数据
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_PCL_PointCloud;			///< 输出PCL格式的点云集
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SegmentedPointCloud;		///< 输出分割后的点云集
	vector<string> m_vPointCloudFileNames;

private:
	//int m_SystemCount;
	string m_CurrentPCFile;
};


#endif // !LIODLL_CLIOFFSTREAM_H

