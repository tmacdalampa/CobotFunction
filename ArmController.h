#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <queue>

#include "Kinematics.h"

using namespace std;

class ArmController
{
private:
	Kinematics* Kin;
	HardwareData* HwTm;

	
	
public:
	array<double, AXISNUM> robot_axis_deg;
	array<double, AXISNUM> robot_pose;

	ArmController(array<double, AXISNUM> init_position);
	~ArmController(void);
	void MotionPlanning(queue<array<double, 6>> init_goal, double vel, double acc, double ang_vel, double ang_acc);
};