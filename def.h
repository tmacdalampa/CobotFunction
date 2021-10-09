#include <array>
#include <utility>
using namespace std;

#define AXISNUM 6
#define ENC_FULL 131072
#define PI 3.1415926
#define RAD2DEG 180/PI
#define DEG2RAD PI/180

struct DHTable
{
	array<double, AXISNUM> a = { 0, 0, 0.425, 0, 0, 0};
	array<double, AXISNUM> alpha = { 0, 90, 0, 90, 90, -90};
	array<double, AXISNUM> d = { 0.426, 0, 0, 0.425, 0, 0.0755 };
	array<double, AXISNUM> upper_limit = {180, 150, 150, 180, 150, 180}; //axis upper limit
	array<double, AXISNUM> lower_limit = { -180, -150, -150, -180, -150, -180 };
};

struct HardwareData
{
	array<double, AXISNUM> zero_points = { 57886.1416, -4116.8298, -10552.0523, -1005.4330, 21242, 28205.0408 };
	//array<double, AXISNUM> zero_points = { 0,0,0,0,0,0 };
	//{57886.1416, 10373.8298, -10552.0523, -1005.4330, 35732, 28205.0408}//this 6 number let arm go to initial pose
	//{57886.1416, 10373.8298, 338        , -1005.4330, 35732, 28205.0408}//this 6 number let arm go to straight
	array<int, AXISNUM> gear_ratios = {161, 161, 121, 161, 161, 161};
	void ENC2DEG(array<double, AXISNUM>& enc_cnts, array<double, AXISNUM>& axis_deg)
	{
		for (int i = 0; i < AXISNUM; i++)
		{
			axis_deg[i] = ((enc_cnts[i] - zero_points[i]) / gear_ratios[i]);//in degree
		}
	}
	void DEG2ENC(array<double, AXISNUM>& enc_cnts, array<double, AXISNUM>& axis_deg)
	{
		for (int i = 0; i < AXISNUM; i++)
		{
			enc_cnts[i] = axis_deg[i] * gear_ratios[i] + zero_points[i];
		}
	}

};

enum KinRes
{
	SUCCEED = 0,
	FAILED = 1,
	EXCEED_AXIS_LIMIT = 2,
	EXCEED_WORKSPACE = 3,
	SINGULAR = 4
};

