#include "LIOCmnFunc.h"

bool Rawdata2PCL(vector<float> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout)
{
	return false;
}

bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<float> pointCloudout)
{
	return false;
}

// vector存储的原始无序点云-> PCL格式的点云
bool Rawdata2PCL(vector<vector<float>> pointCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudout)
{
	int num = pointCloudIn.size();
	laserCloudout->width = num;
	laserCloudout->height = 1;			///< 无序点云 高度置为1
	laserCloudout->points.resize(laserCloudout->width);
	for (int i = 0; i < num; i++)
	{
		laserCloudout->points[i].x = pointCloudIn[i].at(0);
		laserCloudout->points[i].y = pointCloudIn[i].at(1);
		laserCloudout->points[i].z = pointCloudIn[i].at(2);
		laserCloudout->points[i].intensity = pointCloudIn[i].at(3);
	}

	return true;
}

bool PCL2rawdata(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, vector<vector<float>> pointCloudout)
{
	int num = 0;
	if (laserCloudIn->height == 1)
	{
		num = laserCloudIn->width;		///< PCL点云无序
	}
	else
	{
		num = laserCloudIn->width*laserCloudIn->height;  ///< PCL点云有序
	}

	for (int i = 0; i < num; i++)
	{
		pointCloudout[i].push_back(laserCloudIn->points[i].x);
		pointCloudout[i].push_back(laserCloudIn->points[i].y);
		pointCloudout[i].push_back(laserCloudIn->points[i].z);
		pointCloudout[i].push_back(laserCloudIn->points[i].intensity);
	}

	return true;
}

double rad2deg(double radians)
{
	return radians * 180.0 / M_PI;
}

double deg2rad(double degrees)
{
	return degrees * M_PI / 180.0;
}

//bool MatrixSkewSymmetric(const double M1[3], double M2[9])
//{
//	M2[0] = 0; M2[1] = -M1[2]; M2[2] = M1[1];
//	M2[3] = M1[2]; M2[4] = 0; M2[5] = -M1[0];
//	M2[6] = -M1[1]; M2[7] = M1[0]; M2[8] = 0;
//	return true;
//}

void M13xM33(const double M1[3], const double M2[9], double M3[3])
{
	M3[0] = M1[0] * M2[0] + M1[1] * M2[3] + M1[2] * M2[6];
	M3[1] = M1[0] * M2[1] + M1[1] * M2[4] + M1[2] * M2[7];
	M3[2] = M1[0] * M2[2] + M1[1] * M2[5] + M1[2] * M2[8];
}

bool EulerAngle2RotationMat_RzRxRy(const double M1[3], double M2[9])
{
	// 定义的旋转矩阵的顺序是RzRxRy 顺时针为正
	double rx = M1[0];
	double ry = M1[1];
	double rz = M1[2];

	M2[0] = cos(ry)*cos(rz) - sin(rx)*sin(ry)*sin(rz);
	M2[1] = -cos(rx)*sin(rz);
	M2[2] = cos(rz)*sin(ry) + cos(ry)*sin(rx)*sin(rz);
	M2[3] = cos(ry)*sin(rz) + cos(rz)*sin(rx)*sin(ry);
	M2[4] = cos(rx)*cos(rz);
	M2[5] = sin(ry)*sin(rz) - cos(ry)*cos(rz)*sin(rx);
	M2[6] = -cos(rx)*sin(ry);
	M2[7] = sin(rx);
	M2[8] = cos(rx)*cos(ry);

	return true;
}

bool EulerAngle2RotationMat_RyRxRz(const double M1[3], double M2[9])
{
	// 定义的旋转矩阵的顺序是RyRxRz 顺时针为正
	double rx = M1[0];
	double ry = M1[1];
	double rz = M1[2];

	M2[0] = cos(ry)*cos(rz) + sin(rx)*sin(ry)*sin(rz);
	M2[1] = cos(rz)*sin(rx)*sin(ry) - cos(ry)*sin(rz);
	M2[2] = cos(rx)*sin(ry);
	M2[3] = cos(rx)*sin(rz);
	M2[4] = cos(rx)*cos(rz);
	M2[5] = -sin(rx);
	M2[6] = cos(ry)*sin(rx)*sin(rz) - cos(rz)*sin(ry);
	M2[7] = sin(ry)*sin(rz) + cos(ry)*cos(rz)*sin(rx);
	M2[8] = cos(rx)*cos(ry);
	return true;
}



bool RotationMat2EulerAngle(const double M1[9], double M2[3])
{
	
	return false;
}

//void MatrixTranspose(int r, int c, const double M[], double MT[])
//{
//	int i, j;
//
//	for (i = 0; i < r; i++)
//	{
//		for (j = 0; j < c; j++)
//		{
//			MT[j*r + i] = M[i*c + j];
//		}
//	}
//}
//
//void MatrixMultiply(int r1, int c1, const double M1[], int r2, int c2, const double M2[], double M3[])
//{
//#ifdef MATRIX_RANGE_CHECK
//	if (c1 != r2) DumpException(("MatrixMultiply: Inconsistent matrix size"), BUGT_AFX);
//#endif // MATRIX_RANGE_CHECK
//
//	int i, j, k;
//	double Sum;
//
//	for (i = 0; i < r1; i++)
//	{
//		for (j = 0; j < c2; j++)
//		{
//			Sum = 0.0;
//
//			for (k = 0; k < c1; k++)
//			{
//				Sum = Sum + *(M1 + i * c1 + k) * *(M2 + k * c2 + j);
//			}
//
//			*(M3 + i * c2 + j) = Sum;
//		}
//	}
//}
//
//bool MatrixInv(int r, int c, double M[], double M_Inv[])
//{
//#ifdef MATRIX_RANGE_CHECK
//	if (r != c) DumpException(("MatrixInvSP: Non - square matrix"), BUGT_AFX);
//#endif // MATRIX_RANGE_CHECK
//
//	int* is = new int[r];
//	int* js = new int[r];
//	int i, j, k, l, u, v;
//	double d, p;
//
//BEGIN:
//	for (i = 0; i < r; i++)
//	{
//		for (j = 0; j < r; j++)
//		{
//			M_Inv[i*r + j] = M[i*r + j];
//		}
//	}
//
//	for (k = 0; k < r; k++)
//	{
//		d = 0.0;
//		for (i = k; i < r; i++)
//		{
//			for (j = k; j < r; j++)
//			{
//				l = r * i + j;
//				p = fabs(M_Inv[l]);
//				if (p > d)
//				{
//					d = p;
//					is[k] = i;
//					js[k] = j;
//				}
//			}
//		}
//
//		if (d < MATRIX_EPSILON)
//		{
//			if (d < 1e-30) { DumpException(("MatrixInv: singular matrix"), BUGT_AFX); return false; }
//
//			for (i = 0; i < r; i++)   M[i*(r + 1)] += 1E-15; // M矩阵接近奇异时,主对角线加上一个小量
//			goto BEGIN;
//		}
//
//		if (is[k] != k)
//		{
//			for (j = 0; j < r; j++)
//			{
//				u = k * r + j;
//				v = is[k] * r + j;
//				p = M_Inv[u];
//				M_Inv[u] = M_Inv[v];
//				M_Inv[v] = p;
//			}
//		}
//
//		if (js[k] != k)
//		{
//			for (i = 0; i < r; i++)
//			{
//				u = i * r + k;
//				v = i * r + js[k];
//				p = M_Inv[u];
//				M_Inv[u] = M_Inv[v];
//				M_Inv[v] = p;
//			}
//		}
//
//		l = k * r + k;
//		M_Inv[l] = 1.0 / M_Inv[l];
//		for (j = 0; j < r; j++)
//		{
//			if (j != k)
//			{
//				u = k * r + j;
//				M_Inv[u] = M_Inv[u] * M_Inv[l];
//			}
//		}
//		for (i = 0; i < r; i++)
//		{
//			if (i != k)
//			{
//				for (j = 0; j < r; j++)
//				{
//					if (j != k)
//					{
//						u = i * r + j;
//						M_Inv[u] = M_Inv[u] - M_Inv[i*r + k] * M_Inv[k*r + j];
//					}
//				}
//			}
//		}
//		for (i = 0; i < r; i++)
//		{
//			if (i != k)
//			{
//				u = i * r + k;
//				M_Inv[u] = -M_Inv[u] * M_Inv[l];
//			}
//		}
//	}
//
//	for (k = r - 1; k >= 0; k--)
//	{
//		if (js[k] != k)
//		{
//			for (j = 0; j < r; j++)
//			{
//				u = k * r + j;
//				v = js[k] * r + j;
//				p = M_Inv[u];
//				M_Inv[u] = M_Inv[v];
//				M_Inv[v] = p;
//			}
//		}
//		if (is[k] != k)
//		{
//			for (i = 0; i < r; i++)
//			{
//				u = i * r + k;
//				v = is[k] + i * r;
//				p = M_Inv[u];
//				M_Inv[u] = M_Inv[v];
//				M_Inv[v] = p;
//			}
//		}
//	}
//
//	free(is); free(js);
//
//	return true;
//}
//
//bool MatrixInv(int r, int c, double M[])
//{
//#ifdef MATRIX_RANGE_CHECK
//	if (r != c) DumpException(("MatrixInvSP: Non - square matrix"), BUGT_AFX);
//#endif // MATRIX_RANGE_CHECK
//
//	int i;
//	double* b = (double*)malloc(r*r * sizeof(double)); // 分配完不赋零
//	bool flag = MatrixInv(r, c, M, b);
//
//	r = r * r;
//	for (i = 0; i < r; i++) M[i] = b[i];
//
//	free(b);
//
//	return flag;
//}
