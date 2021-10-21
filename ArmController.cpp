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

	
	for (int i = 0; i < AXISNUM; i++)
	{
		_fstart_pose[i] = 0;
		_fend_pose[i] = 0;
		
	}

	load_point_flag = true;
	last_point_flag = false;
	break_flag = false;

	_RMdbpt << 1, 0, 0, 
			   0, 1, 0, 
		       0, 0, 1;

	_target_position_q.clear();
	_target_pose_q.clear();
}
ArmController::~ArmController(void)
{
	delete Kin;
	delete HwTm;
	delete Intp;
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

void ArmController::MotionPlanning(array<double, 3> goal, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max, bool blending)
{
	_target_pose_q.clear();
	Vector3d init_vector;
	VectorXd goal_vector;
	init_vector << goal[0], goal[1], goal[2];
	goal_vector = _RMdbpt * init_vector; //view as dbpt frame
	Vector4d ggoal_vector;
	ggoal_vector << goal_vector[0], goal_vector[1], goal_vector[2], 1;
	//cout << ggoal_vector << endl;
	Vector4d goal_vector_0;
	goal_vector_0 = _T0dbpt * ggoal_vector;
	//cout << goal_vector_0 << endl;
	Kin->PtSetter(goal_vector_0, _RMdbpt, _fend_pose);
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
	/*
	cout << "goal_pose = " << _fend_pose[0] << ", "
		<< _fend_pose[1] << ", "
		<< _fend_pose[2] << ", "
		<< _fend_pose[3] << ", "
		<< _fend_pose[4] << ", "
		<< _fend_pose[5] << endl;*/
		
		

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



void ArmController::T0dbptSetter(array<double, AXISNUM> deburring_point)
{
	_T0dbpt(0, 3) = deburring_point[0]; _T0dbpt(1, 3) = deburring_point[1]; _T0dbpt(2, 3) = deburring_point[2]; _T0dbpt(3, 3) = 1;
	Matrix3d RotationMatrix;
	Kin->ABC2RT(deburring_point[3], deburring_point[4], deburring_point[5],RotationMatrix);
	_T0dbpt(0, 0) = RotationMatrix(0, 0); _T0dbpt(0, 1) = RotationMatrix(0, 1); _T0dbpt(0, 2) = RotationMatrix(0, 2);
	_T0dbpt(1, 0) = RotationMatrix(1, 0); _T0dbpt(1, 1) = RotationMatrix(1, 1); _T0dbpt(1, 2) = RotationMatrix(1, 2);
	_T0dbpt(2, 0) = RotationMatrix(2, 0); _T0dbpt(2, 1) = RotationMatrix(2, 1); _T0dbpt(2, 2) = RotationMatrix(2, 2);
	_T0dbpt(3, 0) = 0; _T0dbpt(3, 1) = 0; _T0dbpt(3, 2) = 0;
	//cout << _T0dbpt << endl;
}