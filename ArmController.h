#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <queue>

#include "Kinematics.h"
#include "Intepolator.h"

using namespace std;

class ArmController
{
private:
	Kinematics* Kin;
	HardwareData* HwTm;
	Intepolator* Intp;

	

	array<double, AXISNUM> _init_axis_deg;
	array<double, AXISNUM> _fstart_pose;
	array<double, AXISNUM> _fend_pose;
	deque<array<double, AXISNUM>> _target_pose_q;
	deque<array<double, AXISNUM>> _target_position_q;

public:
	bool break_flag;
	array<double, AXISNUM> robot_axis_deg;
	array<double, AXISNUM> robot_pose;
	ArmController(array<double, AXISNUM> init_position);
	~ArmController(void);
	void MotionPlanning(queue<array<double, AXISNUM>> init_goal, double vel, double acc, double ang_vel, double ang_acc);
	array<double, AXISNUM> UpdateTargetPosition();
	void UpdateRobotStates(array<double, AXISNUM> current_position);

	
};