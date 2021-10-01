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
    ArmController* a = (ArmController*)Context;
    array<double, 6> TargetPosition;
    array<double, 6> CurrentPosition;


    return 0;
}
pair<bool, array<double, 6>> LoadPoint()
{
    pair<bool, array<double, 6>> res;
    static array<double, 6> gp0 = { 0, 0, 0, 0, 0, 0};
    static array<double, 6> gp1 = { 0.1, 0.2, 0, 0, 0, 0};
    static array<double, 6> gp2 = { 0.1, -0.2, 0, 0, 0, 0};
    static int i = 0;
    int j = i % 7;
    if (j<3)
    {
        switch (j)
        {
        case 0:
            res.second = gp0;
            break;
        case 1:
            res.second = gp1;
            break;
        case 2:
            res.second = gp2;
            break;
        }
    }
    else res.first = false;
    //cout << "i = " << i << endl;
    i = i + 1;
    return res;
}
int _tmain(int argc, _TCHAR* argv[])
{
#if 0
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
    array<double, 6> Init_Position = { 0,90*161,0,0,90*161,0 };

#if 0
    for (int i = 0; i < 6; i++)
    {
        GetAxisPosition(TargetAxis[i], mcActualValue, &Init_Position[i]);
        //RtPrintf("Init position:%d\n", (int)Init_Position[i]);
    }
#endif
    pair<bool, array<double, 6>> LP_res = LoadPoint();
    queue<array<double, 6>> init_goal;
    init_goal.push(LP_res.second);

    LP_res = LoadPoint();
    init_goal.push(LP_res.second);
    
    ArmController Scorpio_Arm(Init_Position);//input goal and current position in degrees

    Scorpio_Arm.MotionPlanning(init_goal, 0.2, 2, 45, 450);
    init_goal.pop();
    
    //Code = RegisterCallback(&CyclicTask, &Scorpio_Arm);
    

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
    RtPrintf("Basic Sample ended\n");

    return 0;
}