/*
读取配置文件的信息
*/
#ifndef LIODLL_CLIOOPTION_H
#define LIODLL_CLIOOPTION_H

#include "LIOSDC.h"


using namespace std;
class CLIOOPTION
{
public:
	CLIOOPTION();
	~CLIOOPTION();

public:
	void Init();
	int ReadOptFile(string filename);
	int DecodeEachOpt(const char*);
	bool CheckOptFileOneline(const char* oneline, string& head, string& text);
	void CheckFilePath(char* filepath);// 检查文件路径,将"F://onsa.txt" 改为"F:\\onsa.txt"
	void CheckFilePath(string& filepath);
	void xstrmid(const char *szSrc, const int nPos, const int nCount, char *szDest);

	string m_PrjDataPath;				///< 点云文件路径
	string m_PCPath;					///< 点云文件路径
	string m_PCTimesFile;				///< 点云时间戳文件
	string m_SegmentedPCPath;			///< 分割点云文件路径
	double m_SampleTime;				///< 点云采样率
	double m_SystemDelay;				///< 弃用前n帧数据

	float m_KeyFramesDistance;
	float m_KeyFramesAttitude;

	int m_N_SCAN;						///< 激光雷达线束
	int m_Horizon_SCAN;					///< 水平一圈扫描点
	double m_Ang_Res_X;					///< 水平角度分辨率
	double m_Ang_Res_Y;					///< 竖直角度分辨率
	double m_Ang_Bottom;				///< 垂直视场最低角度
	int m_GroundScanInd;				///< 可以扫到地面的圈数
	double m_SensorMountAngle;			///< 激光雷达安装角度

	double Lidar2IMULeverArm[3];		///< 激光雷达和惯导间的杆臂值
	double Lidar2IMURotation[3];		///< 激光雷达和惯导间的旋转矩阵	

private:

	//vector<string> m_vPCListsFile;		///< 存放点云文件名

};


#endif // !LIODLL_CLIOOPTION_H

