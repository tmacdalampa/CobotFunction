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
    printf("hello world");
    SharedInformation *iShare;
    HANDLE hsm = NULL;
    HANDLE hEvent1;
    hsm = RtOpenSharedMemory(SHM_MAP_ALL_ACCESS, FALSE, L"sharedspace", (VOID**)&iShare);
    if (hsm == NULL) cout << "Shared memory open failed" << endl;
    hEvent1 = RtCreateEvent(NULL, false, false, L"TestEvent");
    double x = iShare->dbpt[0];
    double y = iShare->dbpt[1];
    double z = iShare->dbpt[2];
    double roll = iShare->dbpt[3];
    double pitch = iShare->dbpt[4];
    double yaw  = iShare->dbpt[5];
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
            iShare->dbpt[0] = x;
            break;
        case 'a':
            x = x - 0.05;
            cout << "x = " << x << endl;
            iShare->dbpt[0] = x;
            break;
        case 'w':
            y = y + 0.05;
            cout << "y = " << y << endl;
            iShare->dbpt[1] = y;
            break;
        case 's':
            y = y - 0.05;
            cout << "y = " << y << endl;
            iShare->dbpt[1] = y;
            break;
        case 'e':
            z = z + 0.05;
            cout << "z = " << z << endl;
            iShare->dbpt[2] = z;
            break;
        case 'd':
            z = z - 0.05;
            cout << "z = " << z << endl;
            iShare->dbpt[2] = z;
            break;
        case 'r':
            roll = roll + 10;
            cout << "roll = " << roll << endl;
            iShare->dbpt[3] = roll;
            break;
        case 'f':
            roll = roll - 10;
            cout << "roll = " << roll << endl;
            iShare->dbpt[3] = roll;
            break;
        case 't':
            pitch = pitch + 10;
            cout << "pitch = " << pitch << endl;
            iShare->dbpt[4] = pitch;
            break;
        case 'g':
            pitch = pitch - 10;
            cout << "pitch = " << pitch << endl;
            iShare->dbpt[4] = pitch;
            break;
        
        case 'y':
            yaw = yaw + 10;
            cout << "yaw = " << yaw << endl;
            iShare->dbpt[5] = yaw;
            break;
        case 'h':
            yaw = yaw - 10;
            cout << "yaw = " << yaw << endl;
            iShare->dbpt[5] = yaw;
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
    ExitProcess(0);
    return 0;
}
