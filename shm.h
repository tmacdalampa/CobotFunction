typedef struct sharedmemory
{
	int Run;
	int x;
	wchar_t StringBuffer[100];
}SharedInformation, * pSharedInformation;
