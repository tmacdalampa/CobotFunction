#include "ArmController.h"
using namespace std;


enum motion_trend
{
    positive, negative, zero
};
ArmController::ArmController(double* init_position)
{
    for (int i = 0; i < 6; i++)
    {
        INIT_POSITION[i] = init_position[i];
        pre_pos[i] = init_position[i];
        pre_vel[i] = 0;
        Acc[i] = 0;
        Vel[i] = 0;
        joint_acc_1[i] = 0;
        joint_acc_2[i] = 0;
        joint_acc_b[i] = 0;
        new_mid_pose[i] = 0;
    }

    Ta = 0;
    Tc1 = 0;
    Tc2 = 0;
    Tt1 = 0;
    Tt2 = 0;
    Tt = 0;
    d1 = 0;
    d2 = 0;
    dt = 0.001;
    t = 0;
    flag = true;
    pre_vel_6 = 0;
    pre_pos_6 = init_position[5];
    goal_vel_6 = 0;
    aux_torque_6 = 0;


}
ArmController::~ArmController(void)
{
}

array<double, 6> ArmController::Gravity_Compensation_Torque(array<double, 6> position)
{
    double M2 = 5; double M3 = 5; double M4 = 2.5; double M5 = 2.5; double M6 = 2.5; //kg
    double La2 = 0.5; double La3 = 0.5;
    double th[6];
    array<double, 6> g_torque = { 0, 0, 0, 0, 0, 0 };
    array<double, 6> m_torque = { 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 6; i++)
    {
        th[i] = ((position[i] - AxisZeroPoint[i]) / AxisGearRatio[i]) * (PI / 180);//in rad
        //cout << "th" << i << " = " << th[i] << endl;

    }

    //g_torque[1] = (7073 * M4 * cos(th[1] + th[2])) / 8625 + (7073 * M5 * cos(th[1] + th[2])) / 8625 + (7073 * M6 * cos(th[1] + th[2])) / 8625 + (42547 * M3 * cos(th[1])) / 51750 + (42547 * M4 * cos(th[1])) / 51750 + (42547 * M5 * cos(th[1])) / 51750 + (42547 * M6 * cos(th[1])) / 51750 + (7073 * La3 * M3 * cos(th[1] + th[2])) / 8625 + (42547 * La2 * M2 * cos(th[1])) / 51750;

    g_torque[1] = (17 * g * (M4 * sin(th[1] + th[2]) + M5 * sin(th[1] + th[2]) + M6 * sin(th[1] + th[2]) + M3 * sin(th[1]) + M4 * sin(th[1]) + M5 * sin(th[1]) + M6 * sin(th[1]))) / 40;
    //axis2: (17*g*(M4*sin(t2 + t3) + M5*sin(t2 + t3) + M6*sin(t2 + t3) + M3*sin(t2) + M4*sin(t2) + M5*sin(t2) + M6*sin(t2)))/40
//g_torque[2] = cos(th[1] + th[2]) * ((94521 * M4) / 41140 + (94521 * M5) / 41140 + (94521 * M6) / 41140 + (94521 * La3 * M3) / 41140);
    g_torque[2] = (17 * g * sin(th[1] + th[2]) * (M4 + M5 + M6)) / 40;
    //axis3:(17*g*sin(t2 + t3)*(M4 + M5 + M6))/40

    //m_torque*AxisGearration*HD_eff = g_torque

    for (int i = 0; i < 6; i++)
    {
        m_torque[i] = g_torque[i] / (AxisGearRatio[i] * HD_eff);
        m_torque[i] = -100 * (m_torque[i] / MotorTorqueConst[i]) / MotorRatedCurrent[i];
    }

    return m_torque;
}

array<double, 6> ArmController::Auxiliary_Torque(array<double, 6> pos, array<double, 6> vel)
{
    enum motion_trend vel_trend;
    array<double, 6> aux_torque = { 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 6; i++)
    {
        if (vel[i] > 300) //move positive
        {
            vel_trend = positive;
            aux_torque[i] = MotorStaticFrictionTorque[i];

        }
        else if (vel[i] < -300)
        {
            vel_trend = negative;
            aux_torque[i] = -MotorStaticFrictionTorque[i];
        }
        else
        {
            vel_trend = zero;
            aux_torque[i] = 0;
        }
        pre_pos[i] = pos[i];
    }
    return aux_torque;
}

double ArmController::Auxiliary_Torque_Axis6(double pos, double vel)
{
    double aux_torque_6;

    if (vel > 500)
    {
        aux_torque_6 = 20;
    }
    else if (vel < -500)
    {
        aux_torque_6 = -20;
    }
    else
    {
        aux_torque_6 = 0;
    }
    

    return aux_torque_6;
}