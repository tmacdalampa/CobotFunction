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
    array<double, 6> CurrentPosition = { 0, 0, 0, 0, 0, 0 };
    array<double, 6> TargetTorque = { 0, 0, 0, 0, 0, 0 };
    array<double, 6> GTorque = { 0, 0, 0, 0, 0, 0 };
    array<double, 6> ATorque = { 0, 0, 0, 0, 0, 0 };
    array<double, 6> CurrentVelocity = { 0, 0, 0, 0, 0, 0 };

    ArmController* a = (ArmController*)Context;
#if 0 //here test what will happened when setting different percentage of torque
    for (int i = 5; i < 6; i++)
    {
        SetAxisTorque(TargetAxis[i], 40);
    }
#else
    for (int i = 0; i < 6; i++)
    {
        GetAxisPosition(TargetAxis[i], mcActualValue, &CurrentPosition[i]);
        GetAxisVelocity(TargetAxis[i], mcActualValue, &CurrentVelocity[i]);
    }
#if 1
    GTorque = a->Gravity_Compensation_Torque(CurrentPosition);//joint space
        //Torque_Vector = sqrt((Trigger_pos_1 - Pre_Trigger_pos_1) ^ 2 + (Trigger_pos_1 - Pre_Trigger_pos_1) ^ 2);
    double Target_Torque1 = a->Auxiliary_Torque_Axis1(CurrentPosition[0], CurrentVelocity[0]);
    SetAxisTorque(TargetAxis[0], Target_Torque1 + GTorque[0]);
    double Target_Torque2 = a->Auxiliary_Torque_Axis2(CurrentPosition[1], CurrentVelocity[1]);
    SetAxisTorque(TargetAxis[1], Target_Torque2 + GTorque[1]);
    double Target_Torque3 = a->Auxiliary_Torque_Axis3(CurrentPosition[2], CurrentVelocity[2]);
    SetAxisTorque(TargetAxis[2], Target_Torque3 + GTorque[2]);
    double Target_Torque4 = a->Auxiliary_Torque_Axis4(CurrentPosition[3], CurrentVelocity[3]);
    SetAxisTorque(TargetAxis[3], Target_Torque4 + GTorque[3]);
    double Target_Torque5 = a->Auxiliary_Torque_Axis5(CurrentPosition[4], CurrentVelocity[4]);
    SetAxisTorque(TargetAxis[4], Target_Torque5 + GTorque[4]);
    double Target_Torque6 = a->Auxiliary_Torque_Axis6(CurrentPosition[5], CurrentVelocity[5]);
    SetAxisTorque(TargetAxis[5], Target_Torque6 + GTorque[5]);
#else
    ATorque = a->Auxiliary_Torque(CurrentPosition, CurrentVelocity);
    GTorque = a->Gravity_Compensation_Torque(CurrentPosition);//joint space
    for (int i = 0; i < 6; i++)
    {
        TargetTorque[i] = GTorque[i];
        SetAxisTorque(TargetAxis[i], TargetTorque[i]);
    }
#endif
#endif

    return 0;
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
    double Init_Position[6] = { 57886.1965, 2964.5288, -5629.6417, -1005.4687, 32650.9744, 32975.4556 };// Init_Pose[6] = {729.755, 0, 634.191, -PI, 0, 0.5*PI}

#if 1
    for (int i = 0; i < 6; i++)
    {
        GetAxisPosition(TargetAxis[i], mcSetValue, &Init_Position[i]);
        //RtPrintf("Init position:%d\n", (int)Init_Position[i]);
    }
#endif
    ArmController Scorpio_Arm(Init_Position);//input goal and current position in degrees
    //array<double, 6> Current_Position = { 57886.1965, 23610.24, -82.9193, -1005.4330, 32374.2535, 32975.4638 };
    //Scorpio_Arm.Gravity_Compensation_Torque(Current_Position);

    Code = RegisterCallback(&CyclicTask, &Scorpio_Arm);
    // Wait for motion to run in the cyclic task
    Sleep(500000);

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
