typedef struct sharedmemory
{
	int Run;
	int  iValue;
	int  iData;
	wchar_t StringBuffer[100];
}SharedInformation, * pSharedInformation;
