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
    array<double, 6> CurrentVelocity;
    array<double, 6> TargetTorque;
    ArmController* a = (ArmController*)Context;

    for (int i = 0; i < 6; i++)
    {
        GetAxisPosition(TargetAxis[i], mcActualValue, &CurrentPosition[i]);
        GetAxisVelocity(TargetAxis[i], mcActualValue, &CurrentVelocity[i]);
    }
    a->Current_Position_Setter(CurrentPosition);
    a->Current_Velocity_Setter(CurrentVelocity);
    for (int i = 0; i < 6; i++)
    {
        TargetTorque= a->UpdateGoalTorque();
        SetAxisTorque(TargetAxis[i], TargetTorque[i]);
    }

    return 0;
}

int _tmain(int argc, _TCHAR* argv[])
{
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

        RtPrintf("Axis status\n  Name: %s\n  Vendor: 0x%x\n  Product: 0x%x\n  Revision: %d\n  Serial: %d\n  Alias: %d\n  Explicit ID: %d\n\n", axis.Name, axis.VendorId, axis.ProductCode, axis.RevisionNumber, axis.SerialNumber, axis.AliasAddress, axis.ExplicitId);

        // Convert the axis unit to degree/s
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
        Command = WaitForCommand(1, TRUE, SetAxisControlMode(TargetAxis[i], modeDirectTor));

        if (!Command.Done) {
            RtPrintf("Failed to set axis control mode: 0x%x\n", Command.ErrorId);
            //goto Exit;
        }

        // After converting the unit you must update the motion and PID parameters

        Code = SetAxisTorquePid(TargetAxis[i], myPid);
        if (Code != errNoError) {
            RtPrintf("Failed to set axis PID: 0x%x\n", Code);
            //goto Exit;
        }

        Code = SetAxisTorquePolarity(TargetAxis[i], mcPositive);

        /*
        Code = SetAxisMotionProfile(TargetAxis[i], profileUnitPerSecond, Motion);
        if (Code != errNoError) {
            RtPrintf("Failed to set axis motion profile: 0x%x\n", Code);
            //goto Exit;
        }
        RtPrintf("Axis configured\n");
        */

    }
#endif

#pragma region PowerAxis
#if 1
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
    array<double, 6> Init_Position = { 57886.1965, 2964.5288, -5629.6417, -1005.4687, 32650.9744, 112491.76 };
    array<double, 6> Goal_Position;
#if 1
    for (int i = 0; i < 6; i++)
    {
        GetAxisPosition(TargetAxis[i], mcSetValue, &Init_Position[i]);
        if (i == 5)
        {
            Goal_Position[i] = Init_Position[i] + 90*101; //axis 6 move 90 degrees
        }
        /*
        else if (i == 3)
        {
            Goal_Position[i] = Init_Position[i]; //axis 4 move 90 degrees
        }
        */
        else
        {
            Goal_Position[i] = Init_Position[i];
        }
    }
#endif
    pair<array<double, 6>, array<double, 6>> Position;
    Position.first = Init_Position;
    Position.second = Goal_Position;
    array<double, 3> Gain = {0.05, 0.025, 0.005}; //{imp_gain, damp_gain, integral_gain}
    ArmController Scorpio_Arm(Position, Gain);//input goal and current position in degrees
    Code = RegisterCallback(&CyclicTask, &Scorpio_Arm);
    // Wait for motion to run in the cyclic task
    Sleep(8000);

#if 1
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
