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
	double _acc, _dcc, _ang_acc, _ang_dcc, _acc_b, _ang_acc_b;
	array<double, 3> _u_trans, _u_rotate;

	//double _v_now, _v_target, _w_now, _w_target;
	array<double,6> _x_target;
	double rounding(double num);

	


public:
	Intepolator();
	~Intepolator();
	void MotionProfileSetter(array<double, 6> fstart_pose, array<double, 6> fend_pose, double vel_max, double acc_max, double ang_vel_max, double ang_acc_max);
	void TargetPoseGenerator(deque<array<double, 6>>& target_pose_q);
	void TargetPoseGeneratorNew(deque<array<double, 6>>& target_pose_q);
};