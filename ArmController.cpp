#include "ArmController.h"
using namespace std;
using namespace Eigen;

ArmController::ArmController(array<double, AXISNUM> init_position)
{
	Kin = new Kinematics;
	HwTm = new HardwareData;
	HwTm->ENC2DEG(init_position, robot_axis_deg);
	/*
	cout << robot_axis_deg[0] << ", "
		<< robot_axis_deg[1] << ", "
		<< robot_axis_deg[2] << ", "
		<< robot_axis_deg[3] << ", "
		<< robot_axis_deg[4] << ", "
		<< robot_axis_deg[5] << endl;
	*/
	Kin->FK(robot_axis_deg, robot_pose);
	
}
ArmController::~ArmController(void)
{
	delete Kin;
	delete HwTm;
}

void ArmController::MotionPlanning(queue<array<double, 6>> init_goal, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max)
{
	Matrix4d T06;
	
	Kin->FK_R(robot_axis_deg, T06);
	Matrix3d R06;
	R06(0, 0) = T06(0, 0); R06(0, 1) = T06(0, 1); R06(0, 2) = T06(0, 2);
	R06(1, 0) = T06(1, 0); R06(1, 1) = T06(1, 1); R06(1, 2) = T06(1, 2);
	R06(2, 0) = T06(2, 0); R06(2, 1) = T06(2, 1); R06(2, 2) = T06(2, 2);
	//cout << "R06 = " << R06 << endl;
	Matrix3d R;
	Kin->ABC2RT(init_goal.front()[3], init_goal.front()[4], init_goal.front()[5], R);
	Matrix3d R6 = R06 * R;
	//cout << "R6 = " << R6 << endl;
	array<double, 3> start_pose;
	Kin->RT2ABC(start_pose, R6);

	//cout <<"start_pose = " << start_pose[0]<<", "<<start_pose[1] << ", " << start_pose[2] << endl;

	Vector4d start_point, end_point;
	start_point << init_goal.front()[0], init_goal.front()[1], init_goal.front()[2], 1;
	end_point << init_goal.back()[0], init_goal.back()[1], init_goal.back()[2], 1;
	Vector4d start_point_end_frame, end_point_end_frame;
	start_point_end_frame = T06 * start_point;
	end_point_end_frame = T06 * end_point;
	cout << start_point_end_frame << endl;
	cout << end_point_end_frame << endl;
	double translation = hypot(hypot((end_point_end_frame[0] - start_point_end_frame[0]), (end_point_end_frame[1] - start_point_end_frame[1])), (end_point_end_frame[2] - start_point_end_frame[2]));
	double rotation = 0;
	cout << "translation = " << translation << endl;

	double Tat = vel_max / acc_max; double Tar = ang_vel_max / ang_acc_max;
	double Ta = (Tat > Tar) ? Tat : Tar;
	cout << "Ta = " << Ta << endl;

	double Tct = (translation / vel_max) - Tat; double Tcr = (rotation / ang_vel_max);
	double Tc = (Tct > Tcr) ? Tct : Tcr;
	cout << "Tc = " << Tc << endl;

	double vel = translation / (Ta + Tc);
	double acc = vel * Ta;
	double ang_vel = rotation / (Ta + Tc);
	double ang_acc = ang_vel * Ta;
	cout << "vel, acc, ang_vel, ang_acc = " << vel << " , " << acc << " , " << ang_vel << " , " << ang_acc << endl;
}

