#include "ArmController.h"
using namespace std;
using namespace Eigen;

ArmController::ArmController(array<double, AXISNUM> init_position)
{
	HwTm = new HardwareData;
	HwTm->ENC2DEG(init_position, _init_axis_deg);
	
	
	Kin = new Kinematics;
	Kin->FK(_init_axis_deg, robot_pose);
	/*cout << "robot_pose= "
		<< robot_pose[3] << ", "
		<< robot_pose[4] << ", "
		<< robot_pose[5] << endl;*/
	Kin->ABC2RT(robot_pose[3], robot_pose[4], robot_pose[5], _R);
	//cout << "init_R = " << _R << endl;
	
	Intp = new Intepolator(robot_pose);
	HwTm->ENC2DEG(init_position, robot_axis_deg);

	
	for (int i = 0; i < AXISNUM; i++)
	{
		_fstart_pose[i] = 0;
		_fend_pose[i] = 0;
		
	}
	for (int i = 0; i < 3; i++)
	{
		_rotation[i] = 0;
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

void ArmController::DbPt6Setter(array<double, AXISNUM> deburring_point)
{
	Matrix4d T06, invT06;
	Vector4d v_dp0;
	v_dp0 << deburring_point[0], deburring_point[1], deburring_point[2], 1;
	
	Kin->FK_R(_init_axis_deg, T06);
	invT06 = T06.inverse();
	_vdp6 = invT06 * v_dp0;
	cout << _vdp6 << endl;
}

void ArmController::fStartPoseSetter()
{
#if 0
	_fstart_pose = { 0.425, 0, 0.7755, 180, 0, 0 };
#else	
	for (int i = 0; i < AXISNUM; i++)
	{
		_fstart_pose[i] = robot_pose[i];
		//cout << _fstart_pose[i] << endl;
	}
#endif
	
	
}

void ArmController::MotionPlanning(array<double, AXISNUM> goal, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max, bool blending)
{
	_target_pose_q.clear();
	Vector4d goal_vector;
	goal_vector << _vdp6[0] - goal[0] , _vdp6[1] - goal[1], _vdp6[2] - goal[2], 1;
	//cout << goal_vector << endl;
	Kin->PtSetter(goal_vector, _deburringT06, _fend_pose);
#if 0
	cout << "==========================" << endl;
	for (int i = 0; i < 6; i++)
	{
		cout << _fstart_pose[i] << endl;
	}
	for (int i = 0; i < 6; i++)
	{
		cout << _fend_pose[i] << endl;
	}
	cout << "+++++++++++++++++++++++++++" << endl;

#endif
	
	cout << "goal_pose = " << _fend_pose[0] << ", "
		<< _fend_pose[1] << ", "
		<< _fend_pose[2] << ", "
		<< _fend_pose[3] << ", "
		<< _fend_pose[4] << ", "
		<< _fend_pose[5] << endl;
		
		

	Intp->MotionProfileSetter(_fstart_pose, _fend_pose, vel_max, acc_max, ang_vel_max, ang_acc_max);
	
	for (int i = 0; i < AXISNUM; i++)
	{
		_fstart_pose[i] = _fend_pose[i];
	}

	switch (blending)
	{
	case false:
		Intp->TargetPoseGenerator(_target_pose_q);
		break;
	case true:
		Intp->TargetPoseGeneratorBlending(_target_pose_q);
		break;
	}
	//cout << "target pose queue size =" << size(_target_pose_q) << endl;
	/*
	cout << "last pose after intp = " << _target_pose_q.back()[0] << ", "
		<< _target_pose_q.back()[1] << ", "
		<< _target_pose_q.back()[2] << ", "
		<< _target_pose_q.back()[3] << ", "
		<< _target_pose_q.back()[4] << ", "
		<< _target_pose_q.back()[5] << endl;*/
		
	
	array<double, AXISNUM> axis_target_position;
	array<double, AXISNUM> motor_target_position;
	//cout << "++++++++++++++++++++++++++++" << endl;
	for (int i = 0; i < size(_target_pose_q); i++)
	{	
		Kin->IK(axis_target_position, _target_pose_q[i], robot_axis_deg);
		//cout << axis_target_position[5] << endl;
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
	//cout << "==========================" << endl;
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
	array<double, 3> deltatheta;
	for (int i = 0; i < 3; i++)
	{
		deltatheta[i] = deburring_point[i + 3] - _rotation[i];
		_rotation[i] = deburring_point[i + 3];
	}
	Matrix3d _deltaR;
	Kin->ABC2RT(deltatheta[0], deltatheta[1], deltatheta[2], _deltaR);
	_R = _deltaR * _R;
	//cout << "_R = " << _R << endl;
	_deburringT06(0, 3) = deburring_point[0] - _R(0, 0) * _vdp6[0] - _R(0, 1) * _vdp6[1] - _R(0, 2) * _vdp6[2];
	_deburringT06(1, 3) = deburring_point[1] - _R(1, 0) * _vdp6[0] - _R(1, 1) * _vdp6[1] - _R(1, 2) * _vdp6[2];
	_deburringT06(2, 3) = deburring_point[2] - _R(2, 0) * _vdp6[0] - _R(2, 1) * _vdp6[1] - _R(2, 2) * _vdp6[2];
	
	_deburringT06(3, 3) = 1;

	//KinRes ABC2RT(double A, double B, double C, Matrix3d & RT);


	_deburringT06(0, 0) = _R(0, 0); _deburringT06(0, 1) = _R(0, 1); _deburringT06(0, 2) = _R(0, 2);
	_deburringT06(1, 0) = _R(1, 0); _deburringT06(1, 1) = _R(1, 1); _deburringT06(1, 2) = _R(1, 2);
	_deburringT06(2, 0) = _R(2, 0); _deburringT06(2, 1) = _R(2, 1); _deburringT06(2, 2) = _R(2, 2);
	_deburringT06(3, 0) = 0; _deburringT06(3, 1) = 0; _deburringT06(3, 2) = 0;
	//cout << _deburringT06 << endl;

}