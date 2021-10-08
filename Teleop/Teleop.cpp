//////////////////////////////////////////////////////////////////
//
// Teleop.cpp - cpp file
//
// This file was generated using the RTX64 Application Template for Visual Studio.
//
// Created: 10/7/2021 5:04:27 PM 
// User: KINGSTAR_RTOS
//
//////////////////////////////////////////////////////////////////

#include "Teleop.h"
   

int _tmain(int argc, _TCHAR * argv[])
{
    printf("hello world");
    PRTCONSUMERCSB pSharedMemory = NULL;

    HANDLE hSharedMemory = RtOpenSharedMemory(SHM_MAP_ALL_ACCESS, FALSE, SharedMemoryName, (void**)&pSharedMemory);

    if (hSharedMemory == NULL)
    {
        DWORD error = GetLastError();
        RtPrintf("Failed to open shared memory region: error = %d\n"
            "You need to start the RtConsumer.rtss real-time process before starting this application.\n",
            error);
        ExitProcess(1);
    }
    //
    // TO DO:  your program code here
    //
    RtCloseHandle(hSharedMemory);
    ExitProcess(0);
    return 0;
}
