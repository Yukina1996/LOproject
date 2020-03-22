#include "CLIOOption.h"

long gs_SystemCount = 0;

CLIOOPTION::CLIOOPTION()
{
	this->Init();
}

CLIOOPTION::~CLIOOPTION()
{
}

void CLIOOPTION::Init()
{
	m_PCPath = "";
	//m_PCListsFile = "";
	m_PCTimesFile = "";
	m_SegmentedPCPath = "";
	m_SampleTime = 0;
	m_SystemDelay = 0;

	m_N_SCAN = 0;
	m_Horizon_SCAN = 0;
	m_Ang_Res_X = 0;
	m_Ang_Res_Y = 0;
	m_Ang_Bottom = 0;
	m_GroundScanInd = 0;
	m_SensorMountAngle = 0;

	for (int i = 0; i < 3; i++)
	{
		Lidar2IMULeverArm[i] = 0;
		Lidar2IMURotation[i] = 0;
	}
}

int CLIOOPTION::ReadOptFile(string filename)
{
	//Init();

	FILE* f_opt = fopen(filename.c_str(), "rt");
	if (f_opt == NULL)
	{
		printf("Error : unable to open opt file!\n");
	}

	char oneline[1024];
	while (fgets(oneline, 1024, f_opt))
	{
		// ASCII 10 对应为换行符 LF 
		if (oneline[0] == 10 || oneline[0] == '#')
			continue;

		DecodeEachOpt(oneline);
	}

	fclose(f_opt);

	return 0;
}

int CLIOOPTION::DecodeEachOpt(const char * oneline)
{
	if (strstr(oneline, "END"))  return 1;

	string text, head;
	int i = 0;
	if (CheckOptFileOneline(oneline, head, text) == false)
		return 2;
	
	char ch[1024] = { '\0' };


	if (head == string("LIO_PointCloudPath")) {
		sscanf(text.c_str(), "%s", ch);
		CheckFilePath(ch);
		m_PrjDataPath = string(ch);
		m_PCPath = string(ch) + "\\velodyne";		
		m_PCTimesFile = string(ch) + "\\times.txt";
		m_SegmentedPCPath = string(ch) + "\\segmented";
		CreateDirectory(m_SegmentedPCPath.c_str(), NULL);// 创建Segment文件夹
	}

	if (head == string("LIO_SampleTime")) {
		sscanf(text.c_str(), "%lf", &m_SampleTime);
	}
	if (head == string("LIO_systemDelay")) {
		sscanf(text.c_str(), "%lf", &m_SystemDelay);
	}
	if (head == string("LIO_KeyFramesDistance")) {
		sscanf(text.c_str(), "%lf", &m_KeyFramesDistance);
	}
	if (head == string("LIO_KeyFramesAttitude")) {
		sscanf(text.c_str(), "%lf", &m_KeyFramesAttitude);
	}
	if (head == string("LIO_N_SCAN")) {
		sscanf(text.c_str(), "%d", &m_N_SCAN);
	}
	if (head == string("LIO_Horizon_SCAN")) {
		sscanf(text.c_str(), "%d", &m_Horizon_SCAN);
	}
	if (head == string("LIO_ang_res_x")) {
		sscanf(text.c_str(), "%lf", &m_Ang_Res_X);
	}
	if (head == string("LIO_ang_res_y")) {
		sscanf(text.c_str(), "%lf", &m_Ang_Res_Y);
	}
	if (head == string("LIO_ang_bottom")) {
		sscanf(text.c_str(), "%lf", &m_Ang_Bottom);
	}
	if (head == string("LIO_groundScanInd")) {
		sscanf(text.c_str(), "%d", &m_GroundScanInd);
	}
	if (head == string("LIO_sensorMountAngle")) {
		sscanf(text.c_str(), "%lf", &m_SensorMountAngle);
	}
	if (head == string("Lidar2IMULeverArm")) {
		sscanf(text.c_str(), "%lf %lf %lf", Lidar2IMULeverArm, Lidar2IMULeverArm + 1, Lidar2IMULeverArm + 2);
	}
	if (head == string("Lidar2IMURotation")) {
		sscanf(text.c_str(), "%lf %lf %lf", Lidar2IMURotation, Lidar2IMURotation + 1, Lidar2IMURotation + 2);
	}


	return 0;
}

/* 检查Option文件每一行是否符合规则 ------------------------------------------
	* input data
	* ----------
	* oneline     : Option 文件当前行
	*
	* output data
	* -----------
	* head        : 返回'='前面的内容, e.g 'GNSS_OBSFile = F:\\obs.10n (file)\n', head(GNSS_OBSFile)
	* text        : 返回'='到'('或'\n'中间的内容, e.g 'GNSS_OBSFile = F:\\obs.10n (file)\n', text(F:\\obs.10n)
	*
	* return data
	* -----------
	* bool        : 检查正确返回true, 错误返回false
	*-----------------------------------------------------------------------------*/
bool CLIOOPTION::CheckOptFileOneline(const char* oneline, string& head, string& text)
{
	char buff[1024], *types[2], *q;
	int n = 0;
	strcpy(buff, oneline);
	for (q = strtok(buff, " "); q&&n < 2; q = strtok(NULL, " ")) types[n++] = q;
	if (n == 1 || strcmp(types[1], "=") != 0)
		return false;

	head = string(types[0]);



	int i = 0;
	const char* p = strstr(oneline, "=");
	if (!p)
		return false;

	int len = strlen(p);

	while (*(++p) == ' ' && i++ < len)
		;

	const char* qt = strstr(oneline, ";");
	if (qt)
	{
		n = qt - p;
		char ch[MAXSIZE] = { '\0' };
		xstrmid(p, 0, n, ch);
		text = string(ch);
		return true;
	}

	if (i < len - 1)
	{
		text = string(p);

		string::iterator it = text.end();
		it--;
		if (*it == 10)
			text.erase(it);

		return true;
	}

	return false;
}

void CLIOOPTION::CheckFilePath(char* filepath)
{
	if (!filepath) return;

	int n = strlen(filepath);
	for (int i = 0; i < n; i++)
	{
		if (filepath[i] == '/') filepath[i] = '\\';
	}
}

void CLIOOPTION::CheckFilePath(string& filepath)
{
	if (filepath == "") return;

	int n = filepath.size();
	for (int i = 0; i < n; i++)
	{
		if (filepath[i] == '/') filepath[i] = '\\';
	}
}

/* 读取复制字符串 ------------------------------------------------------------
	* input data
	* ----------
	* szSrc       : 源字符串,如果末位有'\n',自动除去
	* nPos        : 读取的初始位置(从0开始)
	* nCount      : 读取的字符串数
	*
	* output data
	* -----------
	* szDest      : 目标字符串, 末尾已用\0填充
	*
	*-----------------------------------------------------------------------------*/
void CLIOOPTION::xstrmid(const char *szSrc, const int nPos, const int nCount, char *szDest)
{
	int		i;
	const char	*str;
	char	c;

	str = szSrc + nPos;

	for (i = 0; i < nCount; i++) {
		c = *(str + i);
		if (c) {
			*(szDest + i) = c;
		}
		else {
			// 除去末尾的'\n'
			if (szDest[i - 1] == '\n')
				szDest[i - 1] = '\0';

			*(szDest + i) = '\0';
			break;
		}
	}

	*(szDest + nCount) = '\0';
}
