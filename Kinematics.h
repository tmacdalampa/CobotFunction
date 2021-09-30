#include <Eigen>
#include <iostream>
#include "def.h"


#define RAD2DEG 57.324
#define DEG2RAD 0.01744
#define PI 3.1415926


using namespace std;
using namespace Eigen;
class Kinematics
{
private:
	DHTable dh;

	Matrix4d _T01, _T12, _T23, _T34, _T45, _T56, _T06;
	double ChooseNearst(double a, double b, double c);
	
public:

	Matrix4d GetTFMatrix(double axis_deg, int id);
	KinRes FK(array<double, 6>& axis_deg, array<double, 6>& robot_pose);
	KinRes IK(array<double, 6>& axis_deg, array<double, 6>& robot_pose, array<double, 6>& current_position);
	
	Kinematics();
	~Kinematics();
};
