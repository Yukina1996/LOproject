#include "LIOBase.h"
#include "LIOSDC.h"
#include "LIOCmnFunc.h"
#include "CLIOApplication.h"


int main()
{
	CLIOAPP mylioapp;
	mylioapp.RunLIOApp("D:\\LIOProject\\LIOProject\\LIODLL\\LIO_OPTION.txt");

	getchar();
	return 0;
}