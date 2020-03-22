/*
��ȡ�����ļ�����Ϣ
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
	void CheckFilePath(char* filepath);// ����ļ�·��,��"F://onsa.txt" ��Ϊ"F:\\onsa.txt"
	void CheckFilePath(string& filepath);
	void xstrmid(const char *szSrc, const int nPos, const int nCount, char *szDest);

	string m_PrjDataPath;				///< �����ļ�·��
	string m_PCPath;					///< �����ļ�·��
	string m_PCTimesFile;				///< ����ʱ����ļ�
	string m_SegmentedPCPath;			///< �ָ�����ļ�·��
	double m_SampleTime;				///< ���Ʋ�����
	double m_SystemDelay;				///< ����ǰn֡����

	float m_KeyFramesDistance;
	float m_KeyFramesAttitude;

	int m_N_SCAN;						///< �����״�����
	int m_Horizon_SCAN;					///< ˮƽһȦɨ���
	double m_Ang_Res_X;					///< ˮƽ�Ƕȷֱ���
	double m_Ang_Res_Y;					///< ��ֱ�Ƕȷֱ���
	double m_Ang_Bottom;				///< ��ֱ�ӳ���ͽǶ�
	int m_GroundScanInd;				///< ����ɨ�������Ȧ��
	double m_SensorMountAngle;			///< �����״ﰲװ�Ƕ�

	double Lidar2IMULeverArm[3];		///< �����״�͹ߵ���ĸ˱�ֵ
	double Lidar2IMURotation[3];		///< �����״�͹ߵ������ת����	

private:

	//vector<string> m_vPCListsFile;		///< ��ŵ����ļ���

};


#endif // !LIODLL_CLIOOPTION_H

