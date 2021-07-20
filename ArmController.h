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
	double rounding(double num);
	
	array<double, 6> pre_pos;
	array<double, 6> pre_vel;

	const double MotorRatedCurrent[6] = { 21, 21, 8.8, 2.83, 3.18, 2.43 }; //Amp
	const double MotorTorqueConst[6] = { 0.15, 0.15, 0.17, 0.157, 0.157, 0.157 }; //Nm/Arms
	const double MotorStaticFrictionTorque[6] = { 8.5, 8.5, 10, 20, 20, 20 };

	double pre_vel_1;
	double pre_pos_1;
	double goal_vel_1; //used for goal_vel for auxiliary torque
	double aux_torque_1;
	bool start_move_1;
	bool dwell_flag_1;
	bool move_already_1;
	bool pos_stop_already_1;
	bool neg_stop_already_1;
	bool ini_flag_1;
	int dwell_counter_1;
	int Static_Torque_Counter_1;
	double Trigger_pos_1;
	double Pre_Trigger_pos_1;
	int Trigger_counter_1;
	double Start_move_deg_1;
	double Stop_move_deg_1;
	int Trigger_counter_No_1;
	int Static_Torque_Counter_No_1;
	int Static_Torque_Value1_1;
	int Static_Torque_Value2_1;
	int Dynamic_Torque_Value_1;
	int Dynamic_Torque_Value_Interval_1;
	int Dynamic_Torque_Low_Velocity_1;
	int Dynamic_Torque_Hi_Velocity_1;
	int Dwell_counter_value_1;
	double Static_Torque_Counter_No_Percentage_1;

	double pre_vel_2;
	double pre_pos_2;
	double goal_vel_2; //used for goal_vel for auxiliary torque
	double aux_torque_2;
	bool start_move_2;
	bool dwell_flag_2;
	bool move_already_2;
	bool pos_stop_already_2;
	bool neg_stop_already_2;
	bool ini_flag_2;
	int dwell_counter_2;
	int Static_Torque_Counter_2;
	double Trigger_pos_2;
	double Pre_Trigger_pos_2;
	int Trigger_counter_2;
	double Start_move_deg_2;
	double Stop_move_deg_2;
	int Trigger_counter_No_2;
	int Static_Torque_Counter_No_2;
	int Static_Torque_Value1_2;
	int Static_Torque_Value2_2;
	int Dynamic_Torque_Value_2;
	int Dynamic_Torque_Value_Interval_2;
	int Dynamic_Torque_Low_Velocity_2;
	int Dynamic_Torque_Hi_Velocity_2;
	int Dwell_counter_value_2;
	double Static_Torque_Counter_No_Percentage_2;

	double pre_vel_3;
	double pre_pos_3;
	double goal_vel_3; //used for goal_vel for auxiliary torque
	double aux_torque_3;
	bool start_move_3;
	bool dwell_flag_3;
	bool move_already_3;
	bool pos_stop_already_3;
	bool neg_stop_already_3;
	bool ini_flag_3;
	int dwell_counter_3;
	int Static_Torque_Counter_3;
	double Trigger_pos_3;
	double Pre_Trigger_pos_3;
	int Trigger_counter_3;
	double Start_move_deg_3;
	double Stop_move_deg_3;
	int Trigger_counter_No_3;
	int Static_Torque_Counter_No_3;
	int Static_Torque_Value1_3;
	int Static_Torque_Value2_3;
	int Dynamic_Torque_Value_3;
	int Dynamic_Torque_Value_Interval_3;
	int Dynamic_Torque_Low_Velocity_3;
	int Dynamic_Torque_Hi_Velocity_3;
	int Dwell_counter_value_3;
	double Static_Torque_Counter_No_Percentage_3;

	double pre_vel_4;
	double pre_pos_4;
	double goal_vel_4; //used for goal_vel for auxiliary torque
	double aux_torque_4;
	bool start_move_4;
	bool dwell_flag_4;
	bool move_already_4;
	bool pos_stop_already_4;
	bool neg_stop_already_4;
	bool ini_flag_4;
	int dwell_counter_4;
	int Static_Torque_Counter_4;
	double Trigger_pos_4;
	double Pre_Trigger_pos_4;
	int Trigger_counter_4;
	double Start_move_deg_4;
	double Stop_move_deg_4;
	int Trigger_counter_No_4;
	int Static_Torque_Counter_No_4;
	int Static_Torque_Value1_4;
	int Static_Torque_Value2_4;
	int Dynamic_Torque_Value_4;
	int Dynamic_Torque_Value_Interval_4;
	int Dynamic_Torque_Low_Velocity_4;
	int Dynamic_Torque_Hi_Velocity_4;
	int Dwell_counter_value_4;
	double Static_Torque_Counter_No_Percentage_4;

	double pre_vel_5;
	double pre_pos_5;
	double goal_vel_5; //used for goal_vel for auxiliary torque
	double aux_torque_5;
	bool start_move_5;
	bool dwell_flag_5;
	bool move_already_5;
	bool pos_stop_already_5;
	bool neg_stop_already_5;
	bool ini_flag_5;
	int dwell_counter_5;
	int Static_Torque_Counter_5;
	double Trigger_pos_5;
	double Pre_Trigger_pos_5;
	int Trigger_counter_5;
	double Start_move_deg_5;
	double Stop_move_deg_5;
	int Trigger_counter_No_5;
	int Static_Torque_Counter_No_5;
	int Static_Torque_Value1_5;
	int Static_Torque_Value2_5;
	int Dynamic_Torque_Value_5;
	int Dynamic_Torque_Value_Interval_5;
	int Dynamic_Torque_Low_Velocity_5;
	int Dynamic_Torque_Hi_Velocity_5;
	int Dwell_counter_value_5;
	double Static_Torque_Counter_No_Percentage_5;

	double pre_vel_6;
	double pre_pos_6;
	double goal_vel_6; //used for goal_vel for auxiliary torque
	double aux_torque_6;
	bool start_move_6;
	bool dwell_flag_6;
	bool move_already_6;
	bool pos_stop_already_6;
	bool neg_stop_already_6;
	bool ini_flag_6;
	int dwell_counter_6;
	int Static_Torque_Counter_6;
	double Trigger_pos_6;
	double Pre_Trigger_pos_6;
	int Trigger_counter_6;
	double Start_move_deg_6;
	double Stop_move_deg_6;
	int Trigger_counter_No_6;
	int Static_Torque_Counter_No_6;
	int Static_Torque_Value1_6;
	int Static_Torque_Value2_6;
	int Dynamic_Torque_Value_6;
	int Dynamic_Torque_Value_Interval_6;
	int Dynamic_Torque_Low_Velocity_6;
	int Dynamic_Torque_Hi_Velocity_6;
	int Dwell_counter_value_6;
	double Static_Torque_Counter_No_Percentage_6;

	double Torque_Vector;  //test
	bool toggle23_flag;
	int test_time;


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
	double Auxiliary_Torque_Axis1(double pos, double vel);
	double Auxiliary_Torque_Axis2(double pos, double vel);
	double Auxiliary_Torque_Axis3(double pos, double vel);
	double Auxiliary_Torque_Axis4(double pos, double vel);
	double Auxiliary_Torque_Axis5(double pos, double vel);
	double Auxiliary_Torque_Axis6(double pos, double vel);
};
