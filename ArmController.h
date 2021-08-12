#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>


#define PI 3.1415926
#define g 9.81
#define HD_eff 0.7 //effiency of HD

using namespace std;

class ArmController
{
private:
	double _Kp; //stifness gain
	double _Kd; //damping gain
	double _Ki; //integral gain

	array<double, 6> _init_position;
	array<double, 6> _goal_position;
	array<double, 6> _current_position;
	array<double, 6> _current_velocity;
	array<double, 6> _pos_error;
	array<double, 6> _pos_sum_error;

	
	
	const double MotorRatedCurrent[6] = { 21, 21, 8.8, 2.83, 3.18, 2.43 }; //Amp
	const double MotorTorqueConst[6] = { 0.15, 0.15, 0.17, 0.157, 0.157, 0.157 }; //Nm/Arms
	const double MotorStaticFrictionTorque[6] = { 8.5, 8.5, 10, 20, 20, 20 };

public:
	const int AxisGearRatio[6] = { 161, 161, 121, 161, 161, 101 }; //gear raio
	const double AxisZeroPoint[6] = { 57886.1416, 10373.1702, -82.9193, -1005.4330, 32374.2535, 112491.38 };//zero point
	ArmController(pair<array<double, 6>, array<double, 6>> position, array<double, 3> gain);
	~ArmController(void);
	array<double, 6> UpdateGoalTorque();
	void Current_Position_Setter(array<double, 6> curr_position);
	void Current_Velocity_Setter(array<double, 6> curr_velocity);
};
