/*
³ÌÐòÖ÷º¯Êý
*/

#ifndef LIODLL_CLIOAPPLICATION_H
#define LIODLL_CLIOAPPLICATION_H

#include "LIOBase.h"
#include "LIOCmnFunc.h"
#include "LIOSDC.h"
#include "CLIOOption.h"
#include "CLIOFFStream.h"
#include "CFeatureExtraction.h"
#include "CLidarOdometry.h"
#include "CMapOptimization.h"

using namespace std;

class CLIOAPP
{
public:
	CLIOAPP();
	~CLIOAPP();

	int RunLIOApp(string optFile);
	 
	int ClearLIOApp();

private:

	CLIOOPTION m_LIOOpt;
	CPointCloudFFSTREAM m_PCFFStream;
	CFeatureExtraction m_FeatureExtract;
	CLidarOdometry m_LidarOdometry;
	CMapOptimization m_MapOptimization;
};


#endif // !CLIOAPPLICATION
