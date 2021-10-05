#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <queue>


#define PI 3.1415926
#define g 9.81
#define HD_eff 0.7 //effiency of HD
#define RAD2DEG  57.324

using namespace std;

class ArmController
{
private:
	double error;
	array<double, 6> _current_position;
	
	array<double, 2> u;
public:
	const int AxisGearRatio[6] = { 161, 161, 121, 161, 161, 101 }; //gear raio
	const double AxisZeroPoint[6] = { 57886.1416, 10373.1702, 338, -1005.4330, 21242, 28205 };//zero point
	const double AxisZeroPointFake[6] = {0,0,0,0,0,0};//zero point
	double t;
	double dt;
	queue<array<double, 3>> goal_position_q;
	
	bool motion_end_flag;
	bool start_move_flag;
	void MoveLinear(array<double, 3> &goal_pose, double &vel, double &acc);
	void FK(array<double, 3> &position, array<double, 3> &pose);
	void IK(array<double, 3> &position, array<double, 3> &pose);
	array<double, 6> UpdatePosition();
	ArmController(array<double, 6> init_position);
	~ArmController(void);
};