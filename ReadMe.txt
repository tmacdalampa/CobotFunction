========================================================================
    RTX64 Application Template : "CobotFunction" Project Overview
========================================================================

This is a 6 axes robot arm controller developed under RTX64 environment without UI.

The main purpose of this project is to verify online trajectory generation algorithm in two different appilication:

1. TCP mode:
    Imagine that a robot arm hold a tool like laser head to cut a triangle shape, the user could adjust the offset of preset rotation point(x, y, z, roll, pitch, yaw) 
    and then the trajectory of robot arm end effector will be compensated automatically to keep the original shape.
    This function was completed in branch "try_shm".
    
2. FCP mode:
    Unlike TCP mode , there is a workpiece instead of a tool hold at the end effector of the robot arm, and there is a fix grinding point inside its working space.
    The user could adjust the rotation(roll, pitch, yaw) related to the preset fixed point,
    and then the trajectory of robot arm will be compensated automatically to keep the workpiece grinded as its shape but different angles.
    This function was completed in branch "last_dance".
    
RTX64 Application Template has created this "CobotFunction" project for you as a starting point.

This file contains a summary of what you will find in each of the files that make up your project.

Intepolator.cpp
    Given initial point, goal point, velocity, and acceleration and then generate trajectory that update every cycle time(1ms).
    
Kinematics.cpp
    Function related to robot arm kinematics like foward and inverse kinematics etc.
    
ArmController.cpp
    This is the main controller file including intepolator and kinematics.

CobotFunction.cpp
   This is a cpp file which including main function and LoadPoint function.

Each cpp file has its header file which contains all includes and function protoypes.

CobotFunction.vcxproj
   This is the main project file for projects generated using an Application Template. 
   It contains information about the version of the product that generated the file, and 
   information about the platforms, configurations, and project features selected with the
   RTX64 Application Template.
  
