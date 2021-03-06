#include <iostream>
#include <array>
#include <deque>


using namespace std;

class Intepolator
{
private:
	double _dt;
	double _translation, _rotation;
	double _Ta, _Tc;
	double _vel, _ang_vel, _vel_pre, _ang_vel_pre;
	double _acc, _dcc, _ang_acc, _ang_dcc;
	array<double, 3> _acc_b, _ang_acc_b;
	array<double, 3> _u_trans, _u_rotate, _u_trans_pre, _u_rotate_pre;

	array<double, 3> _v_now, _v_target, _w_now, _w_target;
	array<double, 3> _x_now, _x_target, _theta_now, _theta_target;
	double rounding(double num);

	double _error;
	double _remain_L, _remain_Theta;
	


public:
	
	Intepolator(array<double, 6> robot_pose);
	~Intepolator();
	void MotionProfileSetter(array<double, 6> fstart_pose, array<double, 6> fend_pose, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max);
	void TargetPoseGenerator(deque<array<double, 6>>& target_pose_q);
	void TargetPoseGeneratorNew(deque<array<double, 6>>& target_pose_q);
	void TargetPoseGeneratorBlending(deque<array<double, 6>>& target_pose_q);
	void TargetPoseGeneratorStop(deque<array<double, 6>>& target_pose_q);
};