#include <array>
using namespace std;
typedef struct sharedmemory
{
	int Run;
	array<double, 6> dbpt;
	wchar_t StringBuffer[100];
}SharedInformation, * pSharedInformation;
