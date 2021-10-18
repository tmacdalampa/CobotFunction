//////////////////////////////////////////////////////////////////
//
// CobotFunction.cpp - cpp file
//
// This file was generated using the RTX64 Application Template for Visual Studio.
//
// Created: 7/20/2021 5:05:53 PM 
// User: KINGSTAR_RTOS
//
//////////////////////////////////////////////////////////////////

#include "CobotFunction.h"
using namespace std;

int CyclicTask(PVOID Context)
{

    int TargetAxis[6] = { 0, 1, 2, 3, 4, 5 };
    array<double, 6> CurrentPosition;
    array<double, 6> TargetPosition;

    ArmController* a = (ArmController*)Context;

    if (a->break_flag == false)
    {
        
        for (int i = 0; i < 6; i++)
        {
            GetAxisPosition(TargetAxis[i], mcSetValue, &CurrentPosition[i]);
        }
        a->UpdateRobotStates(CurrentPosition);

        TargetPosition = a->UpdateTargetPosition();
#if 0
        cout << "axis_target_posuition = " << TargetPosition[0] << " , "
            << TargetPosition[1] << " , "
            << TargetPosition[2] << " , "
            << TargetPosition[3] << " , "
            << TargetPosition[4] << " , "
            << TargetPosition[5] << endl;
#endif
        for (int i = 0; i < 6; i++)
        {
            SetAxisPosition(TargetAxis[i], TargetPosition[i]);
        }
        
    }


    return 0;
}

pair<bool, array<double, 6>> LoadPoint(array<double, 6>& deburring_point)
{
    double A, B, C; //relative roll pitch yaw as init pose
    A = 0; B = 0; C = 0;
    pair<bool, array<double, 6>> res;
    
    static array<double, 6> gp0 = { 0.05, 0.06, 0, A, B, C};
    static array<double, 6> gp1 = { 0.05, -0.06, 0, A, B, C};
    static array<double, 6> gp2 = { 0.05, 0.02, 0, A, B, C };
    static array<double, 6> gp3 = { 0.05, 0, 0, A, B, C };
    static array<double, 6> gp4 = { 0.05, -0.02, 0, A, B, C };
    static array<double, 6> gp5 = { 0.05, -0.04, 0, A, B, C };
    static array<double, 6> gp6 = { 0.05, -0.06, 0, A, B, C };
    //static array<double, 6> gp2 = { 0, 0, 0, A, B, C };
    static int i = 0;
    int j = i % 2;
    if (i<21)
    {
        res.first = true;
        switch (j)
        {
        case 0:
            res.second = gp0;
            break;
        case 1:
            res.second = gp1;
            break;
            /*
        case 2:
            res.second = gp2;
        case 3:
            res.second = gp3;
            break;
        case 4:
            res.second = gp4;
            break;
        case 5:
            res.second = gp5;
            break;
        case 6:
            res.second = gp6;
            break;
        case 7:
            res.second = gp5;
            break;
        case 8:
            res.second = gp4;
            break;
        case 9:
            res.second = gp3;
            break;
        case 10:
            res.second = gp2;
            break;
        case 11:
            res.second = gp1;
            break;
            */
        }
        //cout << "i = " << i << endl;
    }
    else res.first = false;
    
    i = i + 1;
    return res;
}
int _tmain(int argc, _TCHAR* argv[])
{
#if 1
    pSharedInformation pSHM;
    HANDLE  	hSHM = NULL;
    HANDLE     hEvent1;
    HANDLE     hSemphone;
    static	PVOID	location;
    
    // Create the shared memory region used to communicate with the user-space producer.
    if (!hSHM)
    {
        if (!(hSHM = RtCreateSharedMemory((DWORD)PAGE_READWRITE,
            (DWORD)0,
            (DWORD)(sizeof(SharedInformation)),
            L"sharedspace",
            (void**)&location)))
        {
            RtPrintf("Can't created shared memory!\n");

        }
        else	RtPrintf("Created shared memory success!\n");
        pSHM = (pSharedInformation)location;
        pSHM->Run = 1;
        //pSHM->iData = 10;
        //pSHM->iValue = 20;
        hEvent1 = RtCreateEvent(NULL, false, false, L"TestEvent");
        hSemphone = RtCreateSemaphore(NULL, 0, 1, L"TestSemaphone");
        if (hSemphone == NULL)
            RtPrintf(" Create Semaphone failed\n");
        //RtWprintf(L"iValue= %d , iData= % d , StringBuffer=%s \n", pSHM->iData, pSHM->iValue, pSHM->StringBuffer);
        //Sleep(2000);
        /*
        while (pSHM->Run != 0) \
        {
            if (RtWaitForSingleObject(hEvent1, 1000) == WAIT_OBJECT_0)
            {
                RtWprintf(L"iValue= %d , iData= % d , StringBuffer=%s \n", pSHM->iData, pSHM->iValue, pSHM->StringBuffer);

            }
        }
        */
    }
#endif
#if 1
    SlaveStatus axis = { 0 };

    int axisResolution = 131072;
    McPidSettings myPid = { 1, 0, 0, 0, 1, 0.003, 0.003, 0, 0.2, 0.1, FALSE, FALSE, 0, 5000 };

    McProfileSettings Motion = { 3, 3600, 3600, 3600, 3600, 3600000, 0 };
    KsCommandStatus Command = { 0 };
    KsError Code = errNoError;
#pragma region Initialization
    RtPrintf("Running KINGSTAR Basic Sample\n");
    // Allways call Create first. It will start the KINGSTAR subsystem if necessary and connect with it.
    // Use instance 0 unless you purchased a multi-runtime license.
    // Use Ideal Processor 0 for the default RTX64 processor.
    Code = Create(0, 0);
    if (Code != errNoError) {
        RtPrintf("Failed to create: 0x%x\n", Code);
        //goto End;
    }

#pragma endregion
#pragma region AxisConfiguration
    int TargetAxis[6] = { 0, 1, 2, 3, 4, 5 };

    // Get the axis identification and resolution
    for (int i = 0; i < 6; i++)
    {
        Code = GetAxisByIndex(TargetAxis[i], &axis, &axisResolution, NULL, NULL);
        if (Code != errNoError) {
            RtPrintf("Failed to get axis status: 0x%x\n", Code);
            //goto Exit;
        }
        cout << "axisResolution = " << axisResolution << endl;
        RtPrintf("Axis status\n  Name: %s\n  Vendor: 0x%x\n  Product: 0x%x\n  Revision: %d\n  Serial: %d\n  Alias: %d\n  Explicit ID: %d\n\n", axis.Name, axis.VendorId, axis.ProductCode, axis.RevisionNumber, axis.SerialNumber, axis.AliasAddress, axis.ExplicitId);

        // Convert the axis unit to degree/s
        //axisResolution = 131072;
        Code = SetAxisCountsPerUnit(TargetAxis[i], axisResolution, 360, FALSE);
        if (Code != errNoError) {
            RtPrintf("Failed to set axis unit: 0x%x\n", Code);
            //goto Exit;
        }

        Code = EnableAxisUnitConversion(TargetAxis[i], TRUE);
        if (Code != errNoError) {
            RtPrintf("Failed to enable axis unit: 0x%x\n", Code);
            //goto Exit;
        }

        // Select the control mode
        Command = WaitForCommand(1, TRUE, SetAxisControlMode(TargetAxis[i], modeDirectPos));

        if (!Command.Done) {
            RtPrintf("Failed to set axis control mode: 0x%x\n", Command.ErrorId);
            //goto Exit;
        }

        // After converting the unit you must update the motion and PID parameters

        // After converting the unit you must update the motion and PID parameters
        Code = SetAxisVelocityPid(TargetAxis[i], myPid);
        if (Code != errNoError) {
            RtPrintf("Failed to set axis PID: 0x%x\n", Code);
            //goto Exit;
        }
        Code = SetAxisMotionProfile(TargetAxis[i], profileUnitPerSecond, Motion);
        if (Code != errNoError) {
            RtPrintf("Failed to set axis motion profile: 0x%x\n", Code);
            //goto Exit;
        }
        RtPrintf("Axis configured\n");
    }
#endif

#pragma region PowerAxis
#if 0
    for (int i = 0; i < 6; i++)
    {
        // Reset Any error in the axis
        Command = WaitForCommand(5, TRUE, ResetAxis(TargetAxis[i]));
        if (!Command.Done) {
            RtPrintf("Failed to reset the axis: 0x%x\n", Command.ErrorId);
            //goto Exit;
        }
        // Enable the axis
        Command = WaitForCommand(1, FALSE, PowerAxis(TargetAxis[i], TRUE, TRUE, TRUE));
        if (!Command.Done) {
            RtPrintf("Failed to enable the axis: 0x%x\n", Command.ErrorId);
            //goto Exit;
        }
        RtPrintf("Axis enabled\n");
    }
#endif 
#pragma endregion
    array<double, 6> Init_Position = { 0,90*161,0,0,90*161, 0};

#if 1
    for (int i = 0; i < 6; i++)
    {
        GetAxisPosition(TargetAxis[i], mcActualValue, &Init_Position[i]);
        //RtPrintf("Init position:%d\n", (int)Init_Position[i]);
    }
#endif
    array<double, 6> deburring_point = { 0.475, 0, 0.7415, 180, 0, 0 };
    
    for (int i = 0; i < 6; i++)
    {
        pSHM->dbpt[i] = deburring_point[i];
    }
    bool isBlending = false;
    ArmController Scorpio_Arm(Init_Position);//input goal and current position in degrees
    Scorpio_Arm.DbPt6Setter(deburring_point);
    Scorpio_Arm.DeburringPtT06Setter(deburring_point);
    Scorpio_Arm.fStartPoseSetter();
    
    pair<bool, array<double, 6>> LP_res = LoadPoint(deburring_point);

    
    Scorpio_Arm.MotionPlanning(LP_res.second, 0.1, 0.1, 45, 450, isBlending);
    
#if 1
    Code = RegisterCallback(&CyclicTask, &Scorpio_Arm);
    
    while (1)
    {
        if (Scorpio_Arm.load_point_flag == true)
        {
            for (int i = 0; i < 6; i++)
            {
                deburring_point[i] = pSHM->dbpt[i];
            }
            
            LP_res = LoadPoint(deburring_point);
            if (LP_res.first == true)
            {
                
                /*cout << "x = " << deburring_point[0] << " , "
                    << "y = " << deburring_point[1] << " , "
                    << "z = " << deburring_point[2] << " , "
                    << "roll = " << deburring_point[3] << " , "
                    << "pitch = " << deburring_point[4] << " , "
                    << "yaw = " << deburring_point[5] << endl;*/
                    
                //cout << LP_res.second[0] << "," << LP_res.second[1] << "," << LP_res.second[2] << endl;
                Scorpio_Arm.DeburringPtT06Setter(deburring_point);
                //init_goal.push(LP_res.second);
                Scorpio_Arm.MotionPlanning(LP_res.second, 0.1, 0.1, 45, 450, isBlending);
                //init_goal.pop();
            }
            else
            {
                if (isBlending == true) Scorpio_Arm.MotionPlanningStop();

                Scorpio_Arm.last_point_flag = true;
            }
        }
        if (Scorpio_Arm.break_flag == true)
        {
            break;
        }
        Sleep(10);
    }
    
#endif
#if 0
#pragma region PowerAxis
    // Disable the axis

    for (int i = 0; i < 6; i++)
    {
        Command = WaitForCommand(1, FALSE, PowerAxis(TargetAxis[i], FALSE, TRUE, TRUE));
        if (!Command.Done) {
            RtPrintf("Failed to disable the axis: 0x%x\n", Command.ErrorId);
            //goto Exit;
        }
        RtPrintf("Axis disabled\n");
    }


#pragma endregion
#endif

#pragma region Closing
    //Exit :
    //// Stop EtherCAT
    //Command = WaitForCommand(10, FALSE, Stop());
    //if (!Command.Done) {
    //    RtPrintf("Failed to stop EtherCAT: %d\n", Command.ErrorId);
    //}
    //else {
    //    RtPrintf("EtherCAT stopped\n");
    //}
    //// Stop the subsystem.
    //// Never stop the subsystem if another application or the PLC is running. It would cause a BSOD.
    //Code = Destroy();
    //if (Code != errNoError) {
    //    RtPrintf("Destroy failed: %d\n", Code);
    //}
#pragma endregion
    //End:
    //RtWprintf(L"iValue= %d , iData= % d , StringBuffer=%s \n", pSHM->iData, pSHM->iValue, pSHM->StringBuffer);
    RtPrintf("Basic Sample ended\n");
    RtCloseHandle(hEvent1);
    RtCloseHandle(hSemphone);

    
    ExitProcess(0);
    return 0;
}