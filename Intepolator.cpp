#include "Intepolator.h"

Intepolator::Intepolator()
{
	_Ta = 0;
	_Tc = 0;
	_vel = 0;
	_ang_vel = 0;
	_vel_pre = 0;
	_ang_vel = 0;
	_ang_vel_pre = 0;
	_acc = 0; 
	_ang_acc = 0;
	_dcc = -_acc; 
	_ang_dcc = -_ang_acc;
	_dt = 0.001;

	_v_now = 0;
	_v_target = 0;
	_w_now = 0;
	_w_target = 0;

	for (int i = 0; i < 3; i++)
	{
		_u_trans[i] = 0;
		_u_rotate[i] = 0;

		

		_x_target[i] = 0;
		_theta_target[i] = 0;
	}
}

Intepolator::~Intepolator()
{
}

void Intepolator::MotionProfileSetter(array<double, 6> fstart_pose, array<double, 6> fend_pose, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max)
{
	double translation = rounding(hypot(hypot((fend_pose[0] - fstart_pose[0]), (fend_pose[1] - fstart_pose[1])), (fend_pose[2] - fstart_pose[2])));
	double rotation = rounding(hypot(hypot((fend_pose[3] - fstart_pose[3]), (fend_pose[4] - fstart_pose[4])), (fend_pose[5] - fstart_pose[5])));
	//cout << "translation = " << translation << endl;
	//cout << "rotation = " << rotation << endl;
	for (int i = 0; i < 3; i++)
	{
		if (translation == 0 ) _u_trans[i] = 0;
		else _u_trans[i] = (fend_pose[i] - fstart_pose[i]) / translation;
		
		if (rotation == 0) _u_rotate[i] = 0;
		else _u_rotate[i] = (fend_pose[i + 3] - fstart_pose[i + 3]) / rotation;
		
		_x_target[i] = fstart_pose[i];
		_theta_target[i] = fstart_pose[i + 3];
		
	}

	double Tat = vel_max / acc_max; double Tar = ang_vel_max / ang_acc_max;
	_Ta = (Tat > Tar) ? Tat : Tar;
	//cout << "Ta = " << _Ta << endl;

	double Tct = (translation / vel_max) - Tat; double Tcr = (rotation / ang_vel_max);
	_Tc = (Tct > Tcr) ? Tct : Tcr;
	//cout << "Tc = " << _Tc << endl;

	_vel = translation / (_Ta + _Tc);
	//cout << "_vel = " << _vel << endl;
	_acc = _vel / _Ta;
	//cout << "_acc = " << _acc << endl;
	_ang_vel = rotation / (_Ta + _Tc);
	_ang_acc = _ang_vel / _Ta;
	_dcc = -_acc;
	//cout << "_dcc = " << _dcc << endl;
	_ang_dcc = -_ang_acc;
	_acc_b = (_vel - _vel_pre) / _Ta;
	//cout << "_acc_b = " << _acc_b << endl;
	_vel_pre = _vel;
	_ang_acc_b = (_ang_vel - _ang_vel_pre) / _Ta;
	_ang_vel_pre = _ang_vel;
}

 void Intepolator::TargetPoseGenerator(deque<array<double, 6>>& target_pose_q)
{
	double a, alpha; 
	array<double, 6 > target_pose;
	for (int i = 0; i < 1000 * (2*_Ta + _Tc); i++)
	{
		if (i < 1000 * _Ta)
		{
			a = _acc_b;
			alpha = _ang_acc_b;
		}
		else if (i >= 1000 * _Ta && i < 1000 * (_Ta + _Tc))
		{
			a = 0;
			alpha = 0;
		}
		else
		{
			a = _dcc;
			alpha = _ang_dcc;
		}
		_v_target = _v_target + a * _dt;
		_w_target = _w_target + alpha * _dt;
		for (int i = 0; i < 3; i++)
		{
			_x_target[i] = _x_target[i] + _u_trans[i] * (_v_target * _dt + 0.5 * a * _dt * _dt);
			_theta_target[i] = _theta_target[i] + _u_rotate[i] * (_w_target * _dt + 0.5 * alpha * _dt * _dt);
			target_pose[i] = rounding(_x_target[i]);
			target_pose[i + 3] = rounding(_theta_target[i]);
		}
		//cout << _v_target << endl;
		//cout << _u_rotate[0]<<" , " << _u_rotate[1] << " , " << _u_rotate[2] << endl;
		//cout << target_pose[0] << endl;
		target_pose_q.push_back(target_pose);
	}
}

 double Intepolator::rounding(double num)
 {
	 int index = 5; //to number 5 
	 bool isNegative = false; // whether is negative number or not

	 if (num < 0) // if this number is negative, then convert to positive number
	 {
		 isNegative = true;
		 num = -num;
	 }

	 if (index >= 0)
	 {
		 int multiplier;
		 multiplier = pow(10, index);
		 num = (int)(num * multiplier + 0.5) / (multiplier * 1.0);
	 }

	 if (isNegative) // if this number is negative, then convert to negative number
	 {
		 num = -num;
	 }

	 return num;
 }