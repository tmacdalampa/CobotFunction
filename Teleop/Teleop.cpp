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

using namespace std;

int _tmain(int argc, _TCHAR * argv[])
{
    Sleep(1000);
    SharedInformationTwo *iShareTwo;
    HANDLE hsm = NULL;
    HANDLE hEvent1;
    hsm = RtOpenSharedMemory(SHM_MAP_ALL_ACCESS, FALSE, L"sharedspace", (VOID**)&iShareTwo);

    if (hsm == NULL)
    {
        cout << "Shared memory open failed" << endl;
    }
#if 1
    hEvent1 = RtCreateEvent(NULL, false, false, L"TestEvent");

    
    double x = iShareTwo->x;
    double y = iShareTwo->y;
    double z = iShareTwo->z;
    double roll = iShareTwo->roll;
    double pitch = iShareTwo->pitch;
    double yaw  = iShareTwo->yaw;
    
    bool flag = true;
    char command;
    
    while (flag == true)
    {    
        cout << "please input command data of roll, pitch, yaw" << endl;
        cin >> command;
        switch (command)
        {
        case 'q':
            x = x + 0.05;
            cout << "x = " << x << endl;
            iShareTwo->x = x;
            break;
        case 'a':
            x = x - 0.05;
            cout << "x = " << x << endl;
            iShareTwo->x = x;
            break;
        case 'w':
            y = y + 0.05;
            cout << "y = " << y << endl;
            iShareTwo->y = y;
            break;
        case 's':
            y = y - 0.05;
            cout << "y = " << y << endl;
            iShareTwo->y = y;
            break;
        case 'e':
            z = z + 0.05;
            cout << "z = " << z << endl;
            iShareTwo->z = z;
            break;
        case 'd':
            z = z - 0.05;
            cout << "z = " << z << endl;
            iShareTwo->z = z;
            break;
        case 'r':
            roll = roll + 10;
            cout << "roll = " << roll << endl;
            if (abs(roll) <= 180)
            {
                iShareTwo->roll = roll;
            }
            break;
        case 'f':
            roll = roll - 10;
            cout << "roll = " << roll << endl;
            if (abs(roll) <= 180)
            {
                iShareTwo->roll = roll;
            }
            break;
        case 't':
            pitch = pitch + 10;
            cout << "pitch = " << pitch << endl;
            if (abs(pitch) <= 180)
            {
                iShareTwo->pitch = pitch;
            }
            break;
        case 'g':
            pitch = pitch - 10;
            cout << "pitch = " << pitch << endl;
            if (abs(pitch) <= 180)
            {
                iShareTwo->pitch = pitch;
            }
            break;
        
        case 'y':
            yaw = yaw + 10;
            cout << "yaw = " << yaw << endl;
            if (abs(yaw) <= 180)
            {
                iShareTwo->yaw = yaw;
            }
            break;
        case 'h':
            yaw = yaw - 10;
            cout << "yaw = " << yaw << endl;
            if (abs(yaw) <= 180) 
            {
                iShareTwo->yaw = yaw;
            }
            break;
        case 'z':
            flag = false;
            break;
        }
        
        RtSetEvent(hEvent1);
    }
    //iShare->iValue = 100;
    //iShare->iData = 200;
    //
    // TO DO:  your program code here
    //
    RtCloseHandle(hEvent1);
#endif
    RtCloseHandle(hsm);
    ExitProcess(0);
    return 0;
}
