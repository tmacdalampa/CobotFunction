#include <Eigen>
#include <iostream>
#include <queue>
#include "def.h"

using namespace std;
using namespace Eigen;
class Kinematics
{
private:
	DHTable* dh;
	Matrix4d _T01, _T12, _T23, _T34, _T45, _T56, _T06;
	double rounding(double num);
	
	double ChooseNearst(double a, double b, double c);

	array<double, 3> Tcp6Setter();
	
public:
	KinRes PtSetter(Vector4d goal_vector, Matrix4d deburringT06, array<double, AXISNUM>& fend_pose);
	Matrix4d GetTFMatrix(double axis_deg, int id);
	KinRes FK(array<double, AXISNUM> axis_deg, array<double, AXISNUM>& robot_pose);
	KinRes IK(array<double, AXISNUM>& axis_deg, array<double, AXISNUM> robot_pose, array<double, 6> current_position);
	KinRes FK_R(array<double, AXISNUM> axis_deg, Matrix4d& T06);
	KinRes ABC2RT(double A, double B, double C, Matrix3d& RT);
	KinRes RT2ABC(array<double, 3>& ABC, Matrix3d RT);
	

	Kinematics();
	~Kinematics();
};
