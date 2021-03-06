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

	Matrix4d _deburringT06;
	Matrix3d _R;

	array<double, AXISNUM> _init_axis_deg;
	array<double, AXISNUM> _fstart_pose;
	array<double, AXISNUM> _fend_pose;

	array<double, 3> _rotation;

	deque<array<double, AXISNUM>> _target_pose_q;
	deque<array<double, AXISNUM>> _target_position_q;

	Vector4d _vdp6;
public:
	bool break_flag;
	bool load_point_flag;
	bool last_point_flag;
	array<double, AXISNUM> robot_axis_deg;
	array<double, AXISNUM> robot_pose;
	ArmController(array<double, AXISNUM> init_position);
	~ArmController(void);
	void MotionPlanning(array<double, AXISNUM> goal, double vel, double acc, double ang_vel, double ang_acc, bool blending);
	void MotionPlanningStop();
	array<double, AXISNUM> UpdateTargetPosition();
	void UpdateRobotStates(array<double, AXISNUM> current_position);
	void DeburringPtT06Setter(array<double, AXISNUM> deburring_point);
	void fStartPoseSetter();
	void DbPt6Setter(array<double, AXISNUM> deburring_point);
};