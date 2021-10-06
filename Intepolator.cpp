#include "Intepolator.h"

Intepolator::Intepolator()
{
	_translation = 0;
	_rotation = 0;
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

	/*_v_now = 0;
	_v_target = 0;
	_w_now = 0;
	_w_target = 0;
	*/

	for (int i = 0; i < 3; i++)
	{
		_u_trans[i] = 0;
		_u_rotate[i] = 0;
		_x_target[i] = 0;
		_x_target[i + 3] = 0;
		//_theta_target[i] = 0;
	}
}

Intepolator::~Intepolator()
{
}

void Intepolator::MotionProfileSetter(array<double, 6> fstart_pose, array<double, 6> fend_pose, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max)
{
	_translation = rounding(hypot(hypot((fend_pose[0] - fstart_pose[0]), (fend_pose[1] - fstart_pose[1])), (fend_pose[2] - fstart_pose[2])));
	_rotation = rounding(hypot(hypot((fend_pose[3] - fstart_pose[3]), (fend_pose[4] - fstart_pose[4])), (fend_pose[5] - fstart_pose[5])));
	cout << "translation = " << _translation << endl;
	//cout << "rotation = " << rotation << endl;
	for (int i = 0; i < 3; i++)
	{
		if (_translation == 0 ) _u_trans[i] = 0;
		else _u_trans[i] = (fend_pose[i] - fstart_pose[i]) / _translation;
		
		if (_rotation == 0) _u_rotate[i] = 0;
		else _u_rotate[i] = (fend_pose[i + 3] - fstart_pose[i + 3]) / _rotation;
		
		_x_target[i] = fstart_pose[i];
		
		_x_target[i+3] = fstart_pose[i + 3];
		
	}
	//cout << "_x_target = " << _x_target[0] << " , " << _x_target[1] << " , " << _x_target[2] << endl;

	double Tat = vel_max / acc_max; double Tar = ang_vel_max / ang_acc_max;
	_Ta = (Tat > Tar) ? Tat : Tar;
	//cout << "Ta = " << _Ta << endl;

	double Tct = (_translation / vel_max) - Tat; double Tcr = (_rotation / ang_vel_max) - Tar;
	_Tc = (Tct > Tcr) ? Tct : Tcr;
	//cout << "Tc = " << _Tc << endl;

	_vel = _translation / (_Ta + _Tc);
	//cout << "_vel = " << _vel << endl;
	_acc = _vel / _Ta;
	//cout << "_acc = " << _acc << endl;
	_ang_vel = _rotation / (_Ta + _Tc);
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
	/*
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
			_x_target[i] = _x_target[i] + _u_trans[i] * (_v_target * _dt + 0.5*a*_dt*_dt);
			_theta_target[i] = _theta_target[i] + _u_rotate[i] * (_w_target * _dt+ 0.5 * alpha * _dt * _dt);
			target_pose[i] = rounding(_x_target[i]);
			target_pose[i + 3] = rounding(_theta_target[i]);
		}
		//cout << _v_target << endl;
		//cout << _u_rotate[0]<<" , " << _u_rotate[1] << " , " << _u_rotate[2] << endl;
		cout << target_pose[0] << " , " << target_pose[1] << " , " << target_pose[2] << endl;
		target_pose_q.push_back(target_pose);
		
	}
	*/
}

 void Intepolator::TargetPoseGeneratorNew(deque<array<double, 6>>& target_pose_q)
 {
	 double error = 0.0001;
	 double a, remain_L, v_target, L_dcc;

	 double ds, v_next;
	 double v_now = 0;

	 int sign;

	 for (int i = 0; i < 6; i++)
	 {
		 cout << "i = " << i << endl;
		 if (i < 3)
		 {
			 a = _u_trans[i] * _acc;
			 remain_L = _u_trans[i] * _translation;
			 cout << "remain_L = " << remain_L << endl;
			 L_dcc = _u_trans[i] * (0.5 * _vel * _vel) / _acc;
			 v_target = _u_trans[i] * _vel;
		 }
		 else
		 {
			 a = _u_rotate[i-3] * _ang_acc;
			 remain_L = _u_rotate[i-3] * _rotation;
			 cout << "remain_L = " << remain_L << endl;
			 L_dcc = _u_rotate[i-3] * (0.5 * _ang_vel * _ang_vel) / _ang_acc;
			 v_target = _u_rotate[i-3] * _ang_vel;
		 }

		 while (1)
		 {
			 if (remain_L <= error)
			 {
				 ds = remain_L;
				 _x_target[i] = _x_target[i] + ds;
				 remain_L = remain_L - ds;
				 break;
			 }

			 if (remain_L <= L_dcc) //time to dcc
			 {
				 v_target = 0;
			 }

			 if (v_now != v_target) //acc or dcc
			 {
				 if (v_now > v_target) sign = -1;
				 else sign = 1;

				 v_next = v_now + sign * a * _dt;


				 if (sign * v_next > sign * v_target)
				 {
					 v_next = v_target;
				 }

				 ds = (v_next + v_now) * _dt * 0.5;
				 v_now = v_next;
			 }
			 else //const vel
			 {
				 v_now = v_next;
				 ds = v_next * _dt;
			 }
			 _x_target[i] = _x_target[i] + ds;
			 remain_L = remain_L - ds;
			 cout << i << " , " << _x_target[i] << endl;
		 }
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