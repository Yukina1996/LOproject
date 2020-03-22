/*
��ȡ����ԭʼ�ļ�
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

	bool Init();									///< �ӵ����ļ����г�ʼ����ǰ���е��Ƶ��ļ���
	int ReadOnePCFile(string pointcloud_filename);
	int ReadOnePCFile(const int SystemCount);
	int ReadOneSegmentedPCFile();


public:
	vector<vector<float>> m_PointCloud;								///< ����ԭʼ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_PCL_PointCloud;			///< ���PCL��ʽ�ĵ��Ƽ�
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_SegmentedPointCloud;		///< ����ָ��ĵ��Ƽ�
	vector<string> m_vPointCloudFileNames;

private:
	//int m_SystemCount;
	string m_CurrentPCFile;
};


#endif // !LIODLL_CLIOFFSTREAM_H

