#include <array>
#include <utility>
using namespace std;

#define AXISNUM 6
#define ENC_FULL 131072

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
	array<double, AXISNUM> zero_points = { 57886.1416, 10373.1702, -82.9193, -1005.4330, 21242, 28205 };
	array<int, AXISNUM> gear_ratios = {161, 161, 121, 161, 161, 161};

};

enum KinRes
{
	SUCCEED = 0,
	FAILED = 1,
	EXCEED_AXIS_LIMIT = 2,
	EXCEED_WORKSPACE = 3,
	SINGULAR = 4
};