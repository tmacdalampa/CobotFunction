#include "Intepolator.h"

Intepolator::Intepolator(array<double, 6> robot_pose)
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

	

	for (int i = 0; i < 3; i++)
	{
		_u_trans[i] = 0;
		_u_rotate[i] = 0;
		_u_trans_pre[i] = 0;
		_u_rotate_pre[i] = 0;
		
		_x_target[i] = robot_pose[i];
		_theta_target[i] = robot_pose[i+3];

		_v_now[i] = 0;
		_v_target[i] = 0;
		_w_now[i] = 0;
		_w_target[i] = 0;

		
	}
}

Intepolator::~Intepolator()
{
}

void Intepolator::MotionProfileSetter(array<double, 6> fstart_pose, array<double, 6> fend_pose, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max)
{
	_translation = rounding(hypot(hypot((fend_pose[0] - fstart_pose[0]), (fend_pose[1] - fstart_pose[1])), (fend_pose[2] - fstart_pose[2])));
	_rotation = rounding(hypot(hypot((fend_pose[3] - fstart_pose[3]), (fend_pose[4] - fstart_pose[4])), (fend_pose[5] - fstart_pose[5])));
	//cout << "translation = " << _translation << endl;
	//cout << "rotation = " << rotation << endl;
	for (int i = 0; i < 3; i++)
	{
		if (_translation == 0 ) _u_trans[i] = 0;
		else _u_trans[i] = (fend_pose[i] - fstart_pose[i]) / _translation;
		
		if (_rotation == 0) _u_rotate[i] = 0;
		else _u_rotate[i] = (fend_pose[i + 3] - fstart_pose[i + 3]) / _rotation;
		
		
		
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
	for (int i = 0; i < 3; i++)
	{
		_acc_b[i] = (_u_trans[i]*_vel - _u_trans_pre[i]*_vel_pre) / _Ta;
		_ang_acc_b[i] = (_u_rotate[i] *_ang_vel - _u_rotate_pre[i]*_ang_vel_pre) / _Ta;
		_u_trans_pre[i] = _u_trans[i];
		_u_rotate_pre[i] = _u_rotate[i];
	}
	
	//cout << "_acc_b = " << _acc_b << endl;
	_vel_pre = _vel;
	//_ang_acc_b = (_ang_vel - _ang_vel_pre) / _Ta;
	_ang_vel_pre = _ang_vel;
}

void Intepolator::TargetPoseGenerator(deque<array<double, 6>>& target_pose_q)
{
	array<double, 3> ds, dtheta;
	array<double, 3> a, alpha; 
	array<double, 6 > target_pose;
	for (int i = 0; i < 1000 * (2*_Ta + _Tc); i++)
	{
		if (i < 1000 * _Ta) //acc
		{
			for (int i = 0; i < 3;i++)
			{
				a[i] = _u_trans[i]*_acc;
				alpha[i] = _u_rotate[i]*_ang_acc;
				_v_target[i] = _v_now[i] + a[i] * _dt;
				_w_target[i] = _w_now[i] + alpha[i] * _dt;
				ds[i] = (_v_target[i] + _v_now[i]) * 0.5 * _dt;
				dtheta[i] = (_w_target[i] + _w_now[i]) * 0.5 * _dt;
				_v_now[i] = _v_target[i];
				_w_now[i] = _w_target[i];
				
				_x_target[i] = _x_target[i] + ds[i];
				target_pose[i] = _x_target[i];
				_theta_target[i] = _theta_target[i] + dtheta[i];
				target_pose[i + 3] = _theta_target[i];
			}
			
			
		}
		else if (i >= 1000 * _Ta && i < 1000 * (_Ta + _Tc))
		{
			for (int i = 0; i < 3;i++)
			{
				
				_v_target[i] = _v_now[i];
				_w_target[i] = _w_now[i];
				ds[i] = _v_target[i]  * _dt;
				dtheta[i] = _w_target[i]  * _dt;
				_v_now[i] = _v_target[i];
				_w_now[i] = _w_target[i];
				

				_x_target[i] = _x_target[i] + ds[i];
				target_pose[i] = _x_target[i];
				_theta_target[i] = _theta_target[i] + dtheta[i];
				target_pose[i + 3] = _theta_target[i];
			}
			
		}
		else
		{
			for (int i = 0; i < 3;i++)
			{
				a[i] = _u_trans[i]*_dcc;
				alpha[i] = _u_rotate[i]*_ang_dcc;
				_v_target[i] = _v_now[i] + a[i] * _dt;
				_w_target[i] = _w_now[i] + alpha[i] * _dt;
				ds[i] = (_v_target[i] + _v_now[i]) * 0.5 * _dt;
				dtheta[i] = (_w_target[i] + _w_now[i]) * 0.5 * _dt;
				_v_now[i] = _v_target[i];
				_w_now[i] = _w_target[i];
				
				_x_target[i] = _x_target[i] + ds[i];
				target_pose[i] = _x_target[i];
				_theta_target[i] = _theta_target[i] + dtheta[i];
				target_pose[i + 3] = _theta_target[i];
			}
		}
		
		//cout << _v_target << endl;
		//cout << _u_rotate[0]<<" , " << _u_rotate[1] << " , " << _u_rotate[2] << endl;
		cout<< target_pose[0] << " , " 
			<< target_pose[1] << " , " 
			<< target_pose[2] << " , "
			<< target_pose[3] << " , "
			<< target_pose[4] << " , "
			<< target_pose[5] << endl;
		target_pose_q.push_back(target_pose);
	}
}

void Intepolator::TargetPoseGeneratorBlending(deque<array<double, 6>>& target_pose_q)
{
	array<double, 3> ds, dtheta;
	array<double, 3> a, alpha;
	array<double, 6 > target_pose;
	for (int i = 0; i < 1000 * (_Ta + _Tc); i++)
	{
		if (i < 1000 * _Ta) //acc
		{
			for (int i = 0; i < 3; i++)
			{
				a[i] = _acc_b[i];
				alpha[i] = _ang_acc_b[i];
				_v_target[i] = _v_now[i] + a[i] * _dt;
				_w_target[i] = _w_now[i] + alpha[i] * _dt;
				ds[i] = (_v_target[i] + _v_now[i]) * 0.5 * _dt;
				dtheta[i] = (_w_target[i] + _w_now[i]) * 0.5 * _dt;
				_v_now[i] = _v_target[i];
				_w_now[i] = _w_target[i];
				//if (i == 1) cout << "blending_part" << _v_now[i] << endl;

				//if (i == 0) cout << "blending_part" << _x_target[i] << endl;
				//if (i == 0) cout << "blending_part ds" << ds[i] << endl;
				_x_target[i] = _x_target[i] + ds[i];
				
				
				target_pose[i] = _x_target[i];
				_theta_target[i] = _theta_target[i] + dtheta[i];
				target_pose[i + 3] = _theta_target[i];
			}


		}
		else if (i >= 1000 * _Ta && i < 1000 * (_Ta + _Tc))
		{
			for (int i = 0; i < 3; i++)
			{
				_v_target[i] = _v_now[i];
				_w_target[i] = _w_now[i];
				ds[i] = _v_target[i] * _dt;
				dtheta[i] = _w_target[i] * _dt;
				_v_now[i] = _v_target[i];
				_w_now[i] = _w_target[i];
				//if (i == 1) cout << " const_part " << _v_now[i] << endl;
				//if (i == 0) cout << "const_part" << _x_target[i] << endl;
				//if (i == 0) cout << "const_part ds" << ds[i] << endl;
				_x_target[i] = _x_target[i] + ds[i];
				

				target_pose[i] = _x_target[i];
				_theta_target[i] = _theta_target[i] + dtheta[i];
				target_pose[i + 3] = _theta_target[i];
			}

		}
		//cout << _v_target << endl;
		//cout << _u_rotate[0]<<" , " << _u_rotate[1] << " , " << _u_rotate[2] << endl;
		//cout << target_pose[0] << " , " << target_pose[1] << " , " << target_pose[2] << endl;
		target_pose_q.push_back(target_pose);
	}
}

void Intepolator::TargetPoseGeneratorStop(deque<array<double, 6>>& target_pose_q)
{
	array<double, 3> ds, dtheta;
	array<double, 3> a, alpha;
	array<double, 6 > target_pose;
	for (int i = 0; i < 1000 * _Ta; i++)
	{
		for (int i = 0; i < 3; i++)
		{
			a[i] = _u_trans[i] * _dcc;
			alpha[i] = _u_rotate[i] * _ang_dcc;
			_v_target[i] = _v_now[i] + a[i] * _dt;
			_w_target[i] = _w_now[i] + alpha[i] * _dt;
			ds[i] = (_v_target[i] + _v_now[i]) * 0.5 * _dt;
			dtheta[i] = (_w_target[i] + _w_now[i]) * 0.5 * _dt;
			_v_now[i] = _v_target[i];
			_w_now[i] = _w_target[i];
			//if (i == 0) cout << _v_now[i] << endl;

			_x_target[i] = _x_target[i] + ds[i];
			target_pose[i] = _x_target[i];
			_theta_target[i] = _theta_target[i] + dtheta[i];
			target_pose[i + 3] = _theta_target[i];
	    }
		//cout << _v_target << endl;
		//cout << _u_rotate[0]<<" , " << _u_rotate[1] << " , " << _u_rotate[2] << endl;
		//cout << target_pose[0] << " , " << target_pose[1] << " , " << target_pose[2] << endl;
		target_pose_q.push_back(target_pose);
	}
}

 void Intepolator::TargetPoseGeneratorNew(deque<array<double, 6>>& target_pose_q)
 {
	 double error = 0.0001;
	 double a, alpha;
	 array<double, 6> target_pose;
	 double remain_L = _translation;
	 double remain_theta = _rotation;
	 double v_now = 0; double v_target = _vel; double L_dcc = (0.5 * _vel * _vel) / _acc; 
	 double ang_v_now = 0; double ang_v_target = _ang_vel;
	 int sign, ang_sign; double ds, dtheta; double v_next, ang_v_next;
	 
	 while (1)
	 {
		 if (remain_L <= error)
		 {
			 ds = remain_L;
			 dtheta = remain_L;
			 for (int i = 0; i < 3; i++)
			 {
				 _x_target[i] = _x_target[i] + _u_trans[i] * ds;
				 _theta_target[i] = _theta_target[i] + _u_rotate[i] * dtheta;
				 target_pose[i] = _x_target[i];
				 target_pose[i + 3] = _theta_target[i];
			 }
#if 0
			 cout << "last" << target_pose[0] << " , "
				 << target_pose[1] << " , "
				 << target_pose[2] << " , "
				 << target_pose[3] << " , "
				 << target_pose[4] << " , "
				 << target_pose[5] << endl;
#endif
			 //cout << "last = " << target_pose[0] << endl;
			 //target_pose[3] = 180;  target_pose[4] = 0;  target_pose[5] = 0;
			 remain_L = remain_L - ds;
			 remain_theta = remain_theta - dtheta;
			 target_pose_q.push_back(target_pose);
			 break;
		 }
		 if (remain_L <= L_dcc) //time to dcc
		 {
			 v_target = 0;
			 ang_v_target = 0;
		 }

		 if (v_now != v_target) //acc or dcc
		 {
			 if (v_now > v_target) sign = -1;
			 else sign = 1;
			 
			 if (ang_v_now > ang_v_target) ang_sign = -1;
			 else ang_sign = 1;

			 v_next = v_now + sign * _acc * _dt;
			 ang_v_next = ang_v_now + ang_sign * _ang_acc * _dt;

			 if (sign * v_next > sign * v_target)
			 {
				 v_next = v_target;
				 ang_v_next = ang_v_target;
			 }

			 ds = (v_next + v_now) * _dt * 0.5;
			 dtheta = (ang_v_next + ang_v_now) * _dt * 0.5;
			 v_now = v_next;
			 ang_v_now = ang_v_next;
		 }
		 else //const vel
		 {
			 v_now = v_next;
			 ds = v_next * _dt;
			 ang_v_now = ang_v_next;
			 dtheta = ang_v_next * _dt;
		 }

		 for (int i = 0; i < 3; i++)
		 {
			 _x_target[i] = _x_target[i] + _u_trans[i] * ds;
			 target_pose[i] = _x_target[i];
			 _theta_target[i] = _theta_target[i] + _u_rotate[i] * dtheta;
			 target_pose[i + 3] = _theta_target[i];
		 }
		 //cout << target_pose[0] << " , " << target_pose[1] << " , " << target_pose[2] << endl;
		 //target_pose[3] = 180;  target_pose[4] = 0;  target_pose[5] = 0;
#if 0
		 cout << target_pose[0] << " , "
			 << target_pose[1] << " , "
			 << target_pose[2] << " , "
			 << target_pose[3] << " , "
			 << target_pose[4] << " , "
			 << target_pose[5] << endl;
#endif
		 remain_L = remain_L - ds;
		 remain_theta = remain_theta - dtheta;
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