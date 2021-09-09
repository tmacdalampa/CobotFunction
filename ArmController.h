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
	//basic variable
	//variable used for motion planning

	//double ang_displacement[3];
	double Ta;
	double Tc1;
	double Tc2;
	double Tt1;
	double Tt2;

	double d1;
	double d2;

	double Acc[6];
	double Vel[6];
	double joint_acc_1[6];
	double joint_acc_b[6];
	double joint_acc_2[6];

	double new_mid_pose[6];

	//member function
	array<double, 6> pre_pos;
	array<double, 6> pre_vel;


	double pre_vel_6;
	double pre_pos_6;
	double goal_vel_6; //used for goal_vel for auxiliary torque
	double aux_torque_6;

	const double MotorRatedCurrent[6] = { 21, 21, 8.8, 2.83, 3.18, 2.43 }; //Amp
	const double MotorTorqueConst[6] = { 0.15, 0.15, 0.17, 0.157, 0.157, 0.157 }; //Nm/Arms
	const double MotorStaticFrictionTorque[6] = { 8.5, 8.5, 10, 20, 20, 20 };


public:

	const int AxisGearRatio[6] = { 161, 161, 121, 161, 161, 101 }; //gear raio
	const double AxisZeroPoint[6] = { 57886.1416, 10373.1702, -82.9193, -1005.4330, 32374.2535, 32975.4638 };//zero point
	double Tt;
	double t;
	double dt;
	double INIT_POSITION[6];
	bool flag;

	ArmController(double* init_position);
	~ArmController(void);

	array<double, 6> Gravity_Compensation_Torque(array<double, 6> position);
	array<double, 6> Auxiliary_Torque(array<double, 6> pos, array<double, 6> vel);
	double Auxiliary_Torque_Axis6(double pos, double vel);
};