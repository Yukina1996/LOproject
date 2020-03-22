#include "CLIOFFStream.h"

CPointCloudFFSTREAM::CPointCloudFFSTREAM()
{
	m_PCL_PointCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
	m_SegmentedPointCloud.reset(new pcl::PointCloud < pcl::PointXYZI>());
}

CPointCloudFFSTREAM::~CPointCloudFFSTREAM()
{
}

bool CPointCloudFFSTREAM::Init()
{
	//�ļ����  
	intptr_t   hFile = 0;
	//�ļ���Ϣ  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(m_LIOOpt->m_PCPath.c_str()).append("\\*.bin").c_str(), &fileinfo)) != -1)
	{
		do
		{
			m_vPointCloudFileNames.push_back(fileinfo.name);
		} while (_findnext(hFile, &fileinfo) == 0);
	}
	_findclose(hFile);

	return true;
}

int CPointCloudFFSTREAM::ReadOnePCFile(string pointcloud_filename)
{
	int num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;
	float tmpx, tmpy, tmpz, tmpr;

	float Point[4];
	vector<float> vPoint;
	vector<vector<float>> pointCloud;

	FILE *instream;
	pointcloud_filename = pointcloud_filename.substr(0, pointcloud_filename.rfind("\n"));//ȥ��\n����
	fopen_s(&instream, pointcloud_filename.c_str(), "rb");
	num = fread(data, sizeof(float), num, instream) / 4;
	
	for (int i = 0; i < num; i++)
	{
		vPoint.push_back(*px); vPoint.push_back(*py); vPoint.push_back(*pz); vPoint.push_back(*pr);
		//if (*px >= -10)								///< X<-10�ĵ��ƶ���
		//{
			m_PointCloud.push_back(vPoint);
		//}
		
		px += 4; py += 4; pz += 4; pr += 4;		
		vector<float>().swap(vPoint);				///< memset�д���swap�ķ�ʽ���vector
	}

	///< ԭʼ����ת��PCL��ʽ����
	Rawdata2PCL(m_PointCloud, m_PCL_PointCloud);

	fclose(instream);

	return 0;
}

int CPointCloudFFSTREAM::ReadOnePCFile(const int SystemCount)
{
	m_CurrentPCFile = m_LIOOpt->m_PCPath + "\\" + m_vPointCloudFileNames[SystemCount - 1];///< ָ��ǰ������ĵ����ļ�	
	
	float *data = (float*)malloc(4 * sizeof(float));

	// pointers
	float *px;
	float *py;
	float *pz;
	float *pr;
	float tmpx, tmpy, tmpz, tmpr;

	float Point[4];
	vector<float> vPoint;
	vPoint.resize(4);

	FILE *instream;
	m_CurrentPCFile = m_CurrentPCFile.substr(0, m_CurrentPCFile.rfind("\n"));//ȥ��\n����
	fopen_s(&instream, m_CurrentPCFile.c_str(), "rb");

	while (!feof(instream))
	{
		if (fread(data, sizeof(float), 4, instream) == 4)
		{
			px = data + 0; py = data + 1; pz = data + 2; pr = data + 3;
			vPoint[0] = *px; vPoint[1] = *py; vPoint[2] = *pz; vPoint[3] = *pr;
			//if (*px >= -10)								///< X<-10�ĵ��ƶ�����KITTI����Ҫ��һ��
			{
				m_PointCloud.push_back(vPoint);
			}
		}
	}

	///< ԭʼ����ת��PCL��ʽ����
	m_PCL_PointCloud->clear();
	Rawdata2PCL(m_PointCloud, m_PCL_PointCloud);
	
	vector<vector<float>>().swap(m_PointCloud);///< �ⶼ��������������һ֡�ĵ�����

	fclose(instream);
	free(data);

	return 0;
}


int CPointCloudFFSTREAM::ReadOneSegmentedPCFile()
{
	return 0;
}
