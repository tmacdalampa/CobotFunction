#include "ArmController.h"
using namespace std;
using namespace Eigen;

ArmController::ArmController(array<double, AXISNUM> init_position)
{
	HwTm = new HardwareData;
	HwTm->ENC2DEG(init_position, _init_axis_deg);
	
	
	Kin = new Kinematics;
	Kin->FK(_init_axis_deg, robot_pose);

	
	Intp = new Intepolator(robot_pose);
	HwTm->ENC2DEG(init_position, robot_axis_deg);
	/*
	cout << robot_axis_deg[0] << ", "
		<< robot_axis_deg[1] << ", "
		<< robot_axis_deg[2] << ", "
		<< robot_axis_deg[3] << ", "
		<< robot_axis_deg[4] << ", "
		<< robot_axis_deg[5] << endl;
	*/
	
	for (int i = 0; i < AXISNUM; i++)
	{
		_fstart_pose[i] = 0;
		_fend_pose[i] = 0;
		//cout << robot_pose[i] << endl;
	}

	load_point_flag = true;
	last_point_flag = false;
	break_flag = false;

	_target_position_q.clear();
	_target_pose_q.clear();
}
ArmController::~ArmController(void)
{
	delete Kin;
	delete HwTm;
	delete Intp;
}

void ArmController::MotionPlanning(queue<array<double, AXISNUM>> init_goal, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max, bool blending)
{
	_target_pose_q.clear();
	Kin->PtSetter(init_goal, _deburringT06, _fstart_pose, _fend_pose);
	Intp->MotionProfileSetter(_fstart_pose, _fend_pose, vel_max, acc_max, ang_vel_max, ang_acc_max);
	switch (blending)
	{
	case false:
		Intp->TargetPoseGenerator(_target_pose_q);
		break;
	case true:
		Intp->TargetPoseGeneratorBlending(_target_pose_q);

	}
	//cout << size(_target_pose_q) << endl;
	
	array<double, AXISNUM> axis_target_position;
	array<double, AXISNUM> motor_target_position;
	for (int i = 0; i < size(_target_pose_q); i++)
	{	
		Kin->IK(axis_target_position, _target_pose_q[i], robot_axis_deg);
#if 0
		cout << axis_target_position[0] << " , "
			<< axis_target_position[1] << " , "
			<< axis_target_position[2] << " , "
			<< axis_target_position[3] << " , "
			<< axis_target_position[4] << " , "
			<< axis_target_position[5] << endl;

#endif
		HwTm->DEG2ENC(motor_target_position, axis_target_position);
#if 0
		cout << motor_target_position[0] << " , "
			   << motor_target_position[1] << " , "
			   << motor_target_position[2] << " , "
		       << motor_target_position[3] << " , "
		       << motor_target_position[4] << " , "
	           << motor_target_position[5] << endl;

#endif
		_target_position_q.push_back(motor_target_position);
	}
	load_point_flag = false;
	
}

void ArmController::MotionPlanningStop()
{
	_target_pose_q.clear();
	Intp->TargetPoseGeneratorStop(_target_pose_q);
	array<double, AXISNUM> axis_target_position;
	array<double, AXISNUM> motor_target_position;
	for (int i = 0; i < size(_target_pose_q); i++)
	{
		Kin->IK(axis_target_position, _target_pose_q[i], robot_axis_deg);
#if 0
		cout << axis_target_position[0] << " , "
			<< axis_target_position[1] << " , "
			<< axis_target_position[2] << " , "
			<< axis_target_position[3] << " , "
			<< axis_target_position[4] << " , "
			<< axis_target_position[5] << endl;

#endif
		HwTm->DEG2ENC(motor_target_position, axis_target_position);
#if 0
		cout << motor_target_position[0] << " , "
			<< motor_target_position[1] << " , "
			<< motor_target_position[2] << " , "
			<< motor_target_position[3] << " , "
			<< motor_target_position[4] << " , "
			<< motor_target_position[5] << endl;

#endif
		_target_position_q.push_back(motor_target_position);
	}
	last_point_flag = true;
	load_point_flag = false;
}

array<double, AXISNUM> ArmController::UpdateTargetPosition()
{
	array<double, AXISNUM> target_position;
	static array<double, AXISNUM> target_position_pre;
	if (_target_position_q.empty() == false)
	{
		target_position = _target_position_q.front();
		target_position_pre = target_position;
		_target_position_q.pop_front();
		if (_target_position_q.size() <= 1000 && last_point_flag == false)
		{
			load_point_flag = true;
		}
		return target_position;
	}
	else
	{
		break_flag = true;
		return target_position_pre;
	}
	
}

void ArmController::UpdateRobotStates(array<double, AXISNUM> current_position)
{
	HwTm->ENC2DEG(current_position, robot_axis_deg);
	Kin->FK(robot_axis_deg, robot_pose);
}

void ArmController::DeburringPtT06Setter(array<double, AXISNUM> deburring_point)
{
	_deburringT06(0, 3) = deburring_point[0];
	_deburringT06(1, 3) = deburring_point[1];
	_deburringT06(2, 3) = deburring_point[2];
	_deburringT06(3, 3) = 1;
	//KinRes ABC2RT(double A, double B, double C, Matrix3d & RT);
	Matrix3d dpRT;
	Kin->ABC2RT(deburring_point[3], deburring_point[4], deburring_point[5], dpRT);

	_deburringT06(0, 0) = dpRT(0, 0); _deburringT06(0, 1) = dpRT(0, 1); _deburringT06(0, 2) = dpRT(0, 2);
	_deburringT06(1, 0) = dpRT(1, 0); _deburringT06(1, 1) = dpRT(1, 1); _deburringT06(1, 2) = dpRT(1, 2);
	_deburringT06(2, 0) = dpRT(2, 0); _deburringT06(2, 1) = dpRT(2, 1); _deburringT06(2, 2) = dpRT(2, 2);
	_deburringT06(3, 0) = 0; _deburringT06(3, 1) = 0; _deburringT06(3, 2) = 0;
	//cout << _deburringT06 << endl;

}