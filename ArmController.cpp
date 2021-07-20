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

    pre_vel_1 = 0;
    pre_pos_1 = init_position[0];
    goal_vel_1 = 0;
    aux_torque_1 = 0;
    Static_Torque_Counter_1 = 0;
    start_move_1 = false;
    dwell_flag_1 = false;
    pos_stop_already_1 = true;
    neg_stop_already_1 = true;
    ini_flag_1 = false;
    dwell_counter_1 = 0;
    Pre_Trigger_pos_1 = 0;
    Trigger_pos_1 = 0;
    Trigger_counter_1 = 0;
    Start_move_deg_1 = 0.025;
    Stop_move_deg_1 = 1;
    Trigger_counter_No_1 = 5;
    Static_Torque_Counter_No_1 = 100;
    Static_Torque_Value1_1 = 40;
    Static_Torque_Value2_1 = 10;
    Dynamic_Torque_Value_1 = 20;
    Dynamic_Torque_Value_Interval_1 = 5;
    Dynamic_Torque_Low_Velocity_1 = 30;
    Dynamic_Torque_Hi_Velocity_1 = 60;
    Dwell_counter_value_1 = 300;
    Static_Torque_Counter_No_Percentage_1 = 0.7;

    pre_vel_2 = 0;
    pre_pos_2 = init_position[1];
    goal_vel_2 = 0;
    aux_torque_2 = 0;
    Static_Torque_Counter_2 = 0;
    start_move_2 = false;
    dwell_flag_2 = false;
    pos_stop_already_2 = true;
    neg_stop_already_2 = true;
    ini_flag_2 = false;
    dwell_counter_2 = 0;
    Pre_Trigger_pos_2 = 0;
    Trigger_pos_2 = 0;
    Trigger_counter_2 = 0;
    Start_move_deg_2 = 0.025;
    Stop_move_deg_2 = 1;
    Trigger_counter_No_2 = 5;
    Static_Torque_Counter_No_2 = 100;
    Static_Torque_Value1_2 = 40;
    Static_Torque_Value2_2 = 10;
    Dynamic_Torque_Value_2 = 20;
    Dynamic_Torque_Value_Interval_2 = 5;
    Dynamic_Torque_Low_Velocity_2 = 30;
    Dynamic_Torque_Hi_Velocity_2 = 60;
    Dwell_counter_value_2 = 300;
    Static_Torque_Counter_No_Percentage_2 = 0.7;

    pre_vel_3 = 0;
    pre_pos_3 = init_position[2];
    goal_vel_3 = 0;
    aux_torque_3 = 0;
    Static_Torque_Counter_3 = 0;
    start_move_3 = false;
    dwell_flag_3 = false;
    pos_stop_already_3 = true;
    neg_stop_already_3 = true;
    ini_flag_3 = false;
    dwell_counter_3 = 0;
    Pre_Trigger_pos_3 = 0;
    Trigger_pos_3 = 0;
    Trigger_counter_3 = 0;
    Start_move_deg_3 = 0.025;
    Stop_move_deg_3 = 1;
    Trigger_counter_No_3 = 5;
    Static_Torque_Counter_No_3 = 100;
    Static_Torque_Value1_3 = 40;
    Static_Torque_Value2_3 = 10;
    Dynamic_Torque_Value_3 = 20;
    Dynamic_Torque_Value_Interval_3 = 5;
    Dynamic_Torque_Low_Velocity_3 = 30;
    Dynamic_Torque_Hi_Velocity_3 = 60;
    Dwell_counter_value_3 = 300;
    Static_Torque_Counter_No_Percentage_3 = 0.7;

    pre_vel_4 = 0;
    pre_pos_4 = init_position[3];
    goal_vel_4 = 0;
    aux_torque_4 = 0;
    Static_Torque_Counter_4 = 0;
    start_move_4 = false;
    dwell_flag_4 = false;
    pos_stop_already_4 = true;
    neg_stop_already_4 = true;
    ini_flag_4 = false;
    dwell_counter_4 = 0;
    Pre_Trigger_pos_4 = 0;
    Trigger_pos_4 = 0;
    Trigger_counter_4 = 0;
    Start_move_deg_4 = 0.025;
    Stop_move_deg_4 = 1;
    Trigger_counter_No_4 = 5;
    Static_Torque_Counter_No_4 = 100;
    Static_Torque_Value1_4 = 40;
    Static_Torque_Value2_4 = 10;
    Dynamic_Torque_Value_4 = 20;
    Dynamic_Torque_Value_Interval_4 = 5;
    Dynamic_Torque_Low_Velocity_4 = 30;
    Dynamic_Torque_Hi_Velocity_4 = 60;
    Dwell_counter_value_4 = 300;
    Static_Torque_Counter_No_Percentage_4 = 0.7;

    pre_vel_5 = 0;
    pre_pos_5 = init_position[4];
    goal_vel_5 = 0;
    aux_torque_5 = 0;
    Static_Torque_Counter_5 = 0;
    start_move_5 = false;
    dwell_flag_5 = false;
    pos_stop_already_5 = true;
    neg_stop_already_5 = true;
    ini_flag_5 = false;
    dwell_counter_5 = 0;
    Pre_Trigger_pos_5 = 0;
    Trigger_pos_5 = 0;
    Trigger_counter_5 = 0;
    Start_move_deg_5 = 0.025;
    Stop_move_deg_5 = 1;
    Trigger_counter_No_5 = 5;
    Static_Torque_Counter_No_5 = 100;
    Static_Torque_Value1_5 = 40;
    Static_Torque_Value2_5 = 10;
    Dynamic_Torque_Value_5 = 20;
    Dynamic_Torque_Value_Interval_5 = 5;
    Dynamic_Torque_Low_Velocity_5 = 30;
    Dynamic_Torque_Hi_Velocity_5 = 60;
    Dwell_counter_value_5 = 300;
    Static_Torque_Counter_No_Percentage_5 = 0.7;

    pre_vel_6 = 0;
    pre_pos_6 = init_position[5];
    goal_vel_6 = 0;
    aux_torque_6 = 0;
    Static_Torque_Counter_6 = 0;
    start_move_6 = false;
    dwell_flag_6 = false;
    pos_stop_already_6 = true;
    neg_stop_already_6 = true;
    ini_flag_6 = false;
    dwell_counter_6 = 0;
    Pre_Trigger_pos_6 = 0;
    Trigger_pos_6 = 0;
    Trigger_counter_6 = 0;
    Start_move_deg_6 = 0.025;
    Stop_move_deg_6 = 1;
    Trigger_counter_No_6 = 5;
    Static_Torque_Counter_No_6 = 100;
    Static_Torque_Value1_6 = 40;
    Static_Torque_Value2_6 = 10;
    Dynamic_Torque_Value_6 = 20;
    Dynamic_Torque_Value_Interval_6 = 5;
    Dynamic_Torque_Low_Velocity_6 = 30;
    Dynamic_Torque_Hi_Velocity_6 = 60;
    Dwell_counter_value_6 = 300;
    Static_Torque_Counter_No_Percentage_6 = 0.7;

    toggle23_flag = false;
    test_time = 60 * 6;
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

        if (i == 1)
        {
            th[i] = th[i] - 0.5 * PI;
            //cout << "theta2 = " << th[1] << endl;
        }

    }

    g_torque[1] = (7073 * M4 * cos(th[1] + th[2])) / 8625 + (7073 * M5 * cos(th[1] + th[2])) / 8625 + (7073 * M6 * cos(th[1] + th[2])) / 8625 + (42547 * M3 * cos(th[1])) / 51750 + (42547 * M4 * cos(th[1])) / 51750 + (42547 * M5 * cos(th[1])) / 51750 + (42547 * M6 * cos(th[1])) / 51750 + (7073 * La3 * M3 * cos(th[1] + th[2])) / 8625 + (42547 * La2 * M2 * cos(th[1])) / 51750;

    //g_torque[1] = (17 * g * (M4 * sin(th[1] + th[2]) + M5 * sin(th[1] + th[2]) + M6 * sin(th[1] + th[2]) + M3 * sin(th[1]) + M4 * sin(th[1]) + M5 * sin(th[1]) + M6 * sin(th[1]))) / 40;
    //axis2: (17*g*(M4*sin(t2 + t3) + M5*sin(t2 + t3) + M6*sin(t2 + t3) + M3*sin(t2) + M4*sin(t2) + M5*sin(t2) + M6*sin(t2)))/40
    g_torque[2] = cos(th[1] + th[2]) * ((94521 * M4) / 41140 + (94521 * M5) / 41140 + (94521 * M6) / 41140 + (94521 * La3 * M3) / 41140);
    //g_torque[2] = (17 * g * sin(th[1] + th[2]) * (M4 + M5 + M6)) / 40;
    //axis3:(17*g*sin(t2 + t3)*(M4 + M5 + M6))/40

    //m_torque*AxisGearration*HD_eff = g_torque

    for (int i = 0; i < 6; i++)
    {
        //m_torque[i] = g_torque[i] / (AxisGearRatio[i]*HD_eff);
        //m_torque[i] = -100 * (m_torque[i] / MotorTorqueConst[i]) / MotorRatedCurrent[i];
        g_torque[i] = -g_torque[i];
    }

    return g_torque;
}


array<double, 6> ArmController::Auxiliary_Torque(array<double, 6> pos, array<double, 6> vel)
{
    enum motion_trend vel_trend;
    array<double, 6> aux_torque = { 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 6; i++)
    {
        if (pos[i] - pre_pos[i] > 0.1) //move positive
        {
            vel_trend = positive;
            aux_torque[i] = MotorStaticFrictionTorque[i];

        }
        else if (pos[i] - pre_pos[i] < -0.1)
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

double ArmController::Auxiliary_Torque_Axis1(double pos, double vel)
{
    enum motion_trend acc_trend1;
    enum motion_trend vel_trend1;
    if (ini_flag_1 == false)
    {
        ini_flag_1 = true;
        pre_pos_1 = pos;
        pre_vel_1 = vel;
        Trigger_pos_1 = pos;
        Pre_Trigger_pos_1 = Trigger_pos_1;
        aux_torque_1 = 0;
        Static_Torque_Counter_1 = 0;
        start_move_1 = false;
        dwell_flag_1 = false;
        pos_stop_already_1 = true;
        neg_stop_already_1 = true;
        dwell_counter_1 = 0;
        Trigger_counter_1 = 0;
        Start_move_deg_1 = 0.025;
        Stop_move_deg_1 = 1;
        Trigger_counter_No_1 = 5;
        Static_Torque_Counter_No_1 = 100;
        Static_Torque_Value1_1 = 50;
        Static_Torque_Value2_1 = 10;
        Dynamic_Torque_Value_1 = 20;
        Dynamic_Torque_Value_Interval_1 = 5;
        Dynamic_Torque_Low_Velocity_1 = 30;
        Dynamic_Torque_Hi_Velocity_1 = 60;
        Dwell_counter_value_1 = 300;
        Static_Torque_Counter_No_Percentage_1 = 0.5;
    }
    Trigger_pos_1 = pos;
    if ((((Trigger_pos_1 - Pre_Trigger_pos_1) > Start_move_deg_1 && neg_stop_already_1 == true) || ((Trigger_pos_1 - Pre_Trigger_pos_1) < -Start_move_deg_1 && pos_stop_already_1 == true)) && Trigger_counter_1 > (Trigger_counter_No_1 - 1))
    {
        if (start_move_1 == false)
        {
            start_move_1 = true;
            cout << "move11111111111111111111111111111111111111!" << endl;
        }

    }

    if (dwell_flag_1 == true)
    {
        dwell_counter_1++;
        aux_torque_1 = 0;
        if (dwell_counter_1 > Dwell_counter_value_1)
        {
            dwell_counter_1 = 0;
            dwell_flag_1 = false;
            pos_stop_already_1 = true;
            neg_stop_already_1 = true;
            Static_Torque_Counter_1 = 0;
        }
    }

    if (start_move_1 == true)
    {
        if ((fabs(pos - pre_pos_1) < Stop_move_deg_1) && Static_Torque_Counter_1 >= Static_Torque_Counter_No_1)
        {
            start_move_1 = false;
            dwell_flag_1 = true;
            neg_stop_already_1 = false;
            pos_stop_already_1 = false;
            cout << "stop11111111111111111111111111111111111111!" << endl;
        }
        else //while moving
        {
            if (Static_Torque_Counter_1 <= Static_Torque_Counter_No_1)
            {
                if ((Trigger_pos_1 - Pre_Trigger_pos_1) >= 0 && Trigger_counter_1 > (Trigger_counter_No_1 - 1) && neg_stop_already_1 == true)
                {
                    neg_stop_already_1 = false;
                    if (Static_Torque_Counter_1 <= (Static_Torque_Counter_No_1 * Static_Torque_Counter_No_Percentage_1))
                        aux_torque_1 = Static_Torque_Value1_1;
                    else
                        aux_torque_1 = Static_Torque_Value2_1;
                }
                if ((Trigger_pos_1 - Pre_Trigger_pos_1) < 0 && Trigger_counter_1 > (Trigger_counter_No_1 - 1) && pos_stop_already_1 == true)
                {
                    pos_stop_already_1 = false;
                    if (Static_Torque_Counter_1 <= (Static_Torque_Counter_No_1 * Static_Torque_Counter_No_Percentage_1))
                        aux_torque_1 = -Static_Torque_Value1_1;
                    else
                        aux_torque_1 = -Static_Torque_Value2_1;
                }
                Static_Torque_Counter_1 = Static_Torque_Counter_1 + 1;
                //char showcounter[100];
                //sprintf(showcounter, "Static_Torque_Counter_1=%d", Static_Torque_Counter_1);
                cout << "Static_Torque_Counter_1=%d" << endl;
            }
            else if (fabs(vel - pre_vel_1) < Dynamic_Torque_Low_Velocity_1) //vel too slow
            {
                if ((pos - pre_pos_1) >= 0)
                {
                    if (aux_torque_1 <= Dynamic_Torque_Value_1)
                        aux_torque_1 = aux_torque_1 + Dynamic_Torque_Value_Interval_1;
                }
                else
                {
                    if (aux_torque_1 >= -Dynamic_Torque_Value_1)
                        aux_torque_1 = aux_torque_1 - Dynamic_Torque_Value_Interval_1;
                }
                cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa!" << endl;
            }
            else if (fabs(vel - pre_vel_1) > Dynamic_Torque_Hi_Velocity_1) //vel too fast
            {
                if ((pos - pre_pos_1) >= 0)
                {
                    if (aux_torque_1 > 0)
                    {
                        aux_torque_1 = aux_torque_1 - Dynamic_Torque_Value_Interval_1;
                        cout << "b" << endl;
                    }
                }
                else
                {
                    if (aux_torque_1 < 0)
                    {
                        aux_torque_1 = aux_torque_1 + Dynamic_Torque_Value_Interval_1;
                    }
                }
                cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb!" << endl;
            }
        }
    }
    //cout << aux_torque_1 << endl;
    Trigger_counter_1++;
    if (Trigger_counter_1 > Trigger_counter_No_1)
    {
        Trigger_counter_1 = 0;
        Pre_Trigger_pos_1 = Trigger_pos_1;
    }
    pre_pos_1 = pos;
    pre_vel_1 = vel;

    t = t + dt;
    if (t > test_time)
    {
        aux_torque_1 = 0;
    }
    return aux_torque_1;
}

double ArmController::Auxiliary_Torque_Axis2(double pos, double vel)
{
    enum motion_trend acc_trend2;
    enum motion_trend vel_trend2;
    if (ini_flag_2 == false)
    {
        ini_flag_2 = true;
        pre_pos_2 = pos;
        pre_vel_2 = vel;
        Trigger_pos_2 = pos;
        Pre_Trigger_pos_2 = Trigger_pos_2;
        aux_torque_2 = 0;
        Static_Torque_Counter_2 = 0;
        start_move_2 = false;
        dwell_flag_2 = false;
        pos_stop_already_2 = true;
        neg_stop_already_2 = true;
        dwell_counter_2 = 0;
        Trigger_counter_2 = 0;
        Start_move_deg_2 = 0.025;
        Stop_move_deg_2 = 1;
        Trigger_counter_No_2 = 5;
        Static_Torque_Counter_No_2 = 100;
        Static_Torque_Value1_2 = 40;
        Static_Torque_Value2_2 = 10;
        Dynamic_Torque_Value_2 = 20;
        Dynamic_Torque_Value_Interval_2 = 5;
        Dynamic_Torque_Low_Velocity_2 = 30;
        Dynamic_Torque_Hi_Velocity_2 = 60;
        Dwell_counter_value_2 = 300;
        Static_Torque_Counter_No_Percentage_2 = 0.7;
    }
    Trigger_pos_2 = pos;
    if ((((Trigger_pos_2 - Pre_Trigger_pos_2) > Start_move_deg_2 && neg_stop_already_2 == true) || ((Trigger_pos_2 - Pre_Trigger_pos_2) < -Start_move_deg_2 && pos_stop_already_2 == true)) && Trigger_counter_2 > (Trigger_counter_No_2 - 1) && toggle23_flag == false && pos_stop_already_3 == true && neg_stop_already_3 == true)
        //if ((((Trigger_pos_2 - Pre_Trigger_pos_2) > Start_move_deg_2 && neg_stop_already_2 == true) || ((Trigger_pos_2 - Pre_Trigger_pos_2) < -Start_move_deg_2 && pos_stop_already_2 == true)) && Trigger_counter_2 > (Trigger_counter_No_2 - 1))
    {
        if (start_move_2 == false)
        {
            toggle23_flag = true;
            start_move_2 = true;
            cout << "move22222222222222222222222222222222222222!" << endl;
        }

    }

    if (dwell_flag_2 == true)
    {
        dwell_counter_2++;
        aux_torque_2 = 0;
        if (dwell_counter_2 > Dwell_counter_value_2)
        {
            dwell_counter_2 = 0;
            dwell_flag_2 = false;
            pos_stop_already_2 = true;
            neg_stop_already_2 = true;
            Static_Torque_Counter_2 = 0;
        }
    }

    if (start_move_2 == true)
    {
        if ((fabs(pos - pre_pos_2) < Stop_move_deg_2) && Static_Torque_Counter_2 >= Static_Torque_Counter_No_2)
        {
            start_move_2 = false;
            dwell_flag_2 = true;
            neg_stop_already_2 = false;
            pos_stop_already_2 = false;
            cout << "stop22222222222222222222222222222222222222!" << endl;
        }
        else //while moving
        {
            if (Static_Torque_Counter_2 <= Static_Torque_Counter_No_2)
            {
                if ((Trigger_pos_2 - Pre_Trigger_pos_2) >= 0 && Trigger_counter_2 > (Trigger_counter_No_2 - 1) && neg_stop_already_2 == true)
                {
                    neg_stop_already_2 = false;
                    if (Static_Torque_Counter_2 <= (Static_Torque_Counter_No_2 * Static_Torque_Counter_No_Percentage_2))
                        aux_torque_2 = Static_Torque_Value1_2;
                    else
                        aux_torque_2 = Static_Torque_Value2_2;
                }
                if ((Trigger_pos_2 - Pre_Trigger_pos_2) < 0 && Trigger_counter_2 > (Trigger_counter_No_2 - 1) && pos_stop_already_2 == true)
                {
                    pos_stop_already_2 = false;
                    if (Static_Torque_Counter_2 <= (Static_Torque_Counter_No_2 * Static_Torque_Counter_No_Percentage_2))
                        aux_torque_2 = -Static_Torque_Value1_2;
                    else
                        aux_torque_2 = -Static_Torque_Value2_2;
                }
                Static_Torque_Counter_2 = Static_Torque_Counter_2 + 1;
                //char showcounter[100];
                //sprintf(showcounter, "Static_Torque_Counter_2=%d", Static_Torque_Counter_2);
                cout << "Static_Torque_Counter_2=%d" << endl;
            }
            else if (fabs(vel - pre_vel_2) < Dynamic_Torque_Low_Velocity_2) //vel too slow
            {
                if ((pos - pre_pos_2) >= 0)
                {
                    if (aux_torque_2 <= Dynamic_Torque_Value_2)
                        aux_torque_2 = aux_torque_2 + Dynamic_Torque_Value_Interval_2;
                }
                else
                {
                    if (aux_torque_2 >= -Dynamic_Torque_Value_2)
                        aux_torque_2 = aux_torque_2 - Dynamic_Torque_Value_Interval_2;
                }
                cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa!" << endl;
            }
            else if (fabs(vel - pre_vel_2) > Dynamic_Torque_Hi_Velocity_2) //vel too fast
            {
                if ((pos - pre_pos_2) >= 0)
                {
                    if (aux_torque_2 > 0)
                    {
                        aux_torque_2 = aux_torque_2 - Dynamic_Torque_Value_Interval_2;
                        cout << "b" << endl;
                    }
                }
                else
                {
                    if (aux_torque_2 < 0)
                    {
                        aux_torque_2 = aux_torque_2 + Dynamic_Torque_Value_Interval_2;
                    }
                }
                cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb!" << endl;
            }
        }
    }
    //cout << aux_torque_2 << endl;
    Trigger_counter_2++;
    if (Trigger_counter_2 > Trigger_counter_No_2)
    {
        Trigger_counter_2 = 0;
        Pre_Trigger_pos_2 = Trigger_pos_2;
    }
    pre_pos_2 = pos;
    pre_vel_2 = vel;

    t = t + dt;
    if (t > test_time)
    {
        aux_torque_2 = 0;
    }
    //if (start_move_3 == true)
    //   aux_torque_2 = 0;
    return aux_torque_2;
}

double ArmController::Auxiliary_Torque_Axis3(double pos, double vel)
{
    enum motion_trend acc_trend3;
    enum motion_trend vel_trend3;
    if (ini_flag_3 == false)
    {
        ini_flag_3 = true;
        pre_pos_3 = pos;
        pre_vel_3 = vel;
        Trigger_pos_3 = pos;
        Pre_Trigger_pos_3 = Trigger_pos_3;
        aux_torque_3 = 0;
        Static_Torque_Counter_3 = 0;
        start_move_3 = false;
        dwell_flag_3 = false;
        pos_stop_already_3 = true;
        neg_stop_already_3 = true;
        dwell_counter_3 = 0;
        Trigger_counter_3 = 0;
        Start_move_deg_3 = 0.025;
        Stop_move_deg_3 = 1;
        Trigger_counter_No_3 = 5;
        Static_Torque_Counter_No_3 = 100;
        Static_Torque_Value1_3 = 50;
        Static_Torque_Value2_3 = 10;
        Dynamic_Torque_Value_3 = 40;
        Dynamic_Torque_Value_Interval_3 = 5;
        Dynamic_Torque_Low_Velocity_3 = 30;
        Dynamic_Torque_Hi_Velocity_3 = 60;
        Dwell_counter_value_3 = 300;
        Static_Torque_Counter_No_Percentage_3 = 0.7;
    }
    Trigger_pos_3 = pos;
    if ((((Trigger_pos_3 - Pre_Trigger_pos_3) > Start_move_deg_3 && neg_stop_already_3 == true) || ((Trigger_pos_3 - Pre_Trigger_pos_3) < -Start_move_deg_3 && pos_stop_already_3 == true)) && Trigger_counter_3 > (Trigger_counter_No_3 - 1) && toggle23_flag == true && pos_stop_already_2 == true && neg_stop_already_2 == true)
        //if ((((Trigger_pos_3 - Pre_Trigger_pos_3) > Start_move_deg_3 && neg_stop_already_3 == true) || ((Trigger_pos_3 - Pre_Trigger_pos_3) < -Start_move_deg_3 && pos_stop_already_3 == true)) && Trigger_counter_3 > (Trigger_counter_No_3 - 1))
    {
        if (start_move_3 == false)
        {
            toggle23_flag = false;
            start_move_3 = true;
            cout << "move33333333333333333333333333333333333333!" << endl;
        }

    }

    if (dwell_flag_3 == true)
    {
        dwell_counter_3++;
        aux_torque_3 = 0;
        if (dwell_counter_3 > Dwell_counter_value_3)
        {
            dwell_counter_3 = 0;
            dwell_flag_3 = false;
            pos_stop_already_3 = true;
            neg_stop_already_3 = true;
            Static_Torque_Counter_3 = 0;
        }
    }

    if (start_move_3 == true)
    {
        if ((fabs(pos - pre_pos_3) < Stop_move_deg_3) && Static_Torque_Counter_3 >= Static_Torque_Counter_No_3)
        {
            start_move_3 = false;
            dwell_flag_3 = true;
            neg_stop_already_3 = false;
            pos_stop_already_3 = false;
            cout << "stop33333333333333333333333333333333333333!" << endl;
        }
        else //while moving
        {
            if (Static_Torque_Counter_3 <= Static_Torque_Counter_No_3)
            {
                if ((Trigger_pos_3 - Pre_Trigger_pos_3) >= 0 && Trigger_counter_3 > (Trigger_counter_No_3 - 1) && neg_stop_already_3 == true)
                {
                    neg_stop_already_3 = false;
                    if (Static_Torque_Counter_3 <= (Static_Torque_Counter_No_3 * Static_Torque_Counter_No_Percentage_3))
                        aux_torque_3 = Static_Torque_Value1_3;
                    else
                        aux_torque_3 = Static_Torque_Value2_3;
                }
                if ((Trigger_pos_3 - Pre_Trigger_pos_3) < 0 && Trigger_counter_3 > (Trigger_counter_No_3 - 1) && pos_stop_already_3 == true)
                {
                    pos_stop_already_3 = false;
                    if (Static_Torque_Counter_3 <= (Static_Torque_Counter_No_3 * Static_Torque_Counter_No_Percentage_3))
                        aux_torque_3 = -Static_Torque_Value1_3;
                    else
                        aux_torque_3 = -Static_Torque_Value2_3;
                }
                Static_Torque_Counter_3 = Static_Torque_Counter_3 + 1;
                //char showcounter[100];
                //sprintf(showcounter, "Static_Torque_Counter_3=%d", Static_Torque_Counter_3);
                cout << "Static_Torque_Counter_3=%d" << endl;
            }
            else if (fabs(vel - pre_vel_3) < Dynamic_Torque_Low_Velocity_3) //vel too slow
            {
                if ((pos - pre_pos_3) >= 0)
                {
                    if (aux_torque_3 <= Dynamic_Torque_Value_3)
                        aux_torque_3 = aux_torque_3 + Dynamic_Torque_Value_Interval_3;
                }
                else
                {
                    if (aux_torque_3 >= -Dynamic_Torque_Value_3)
                        aux_torque_3 = aux_torque_3 - Dynamic_Torque_Value_Interval_3;
                }
                cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa!" << endl;
            }
            else if (fabs(vel - pre_vel_3) > Dynamic_Torque_Hi_Velocity_3) //vel too fast
            {
                if ((pos - pre_pos_3) >= 0)
                {
                    if (aux_torque_3 > 0)
                    {
                        aux_torque_3 = aux_torque_3 - Dynamic_Torque_Value_Interval_3;
                        cout << "b" << endl;
                    }
                }
                else
                {
                    if (aux_torque_3 < 0)
                    {
                        aux_torque_3 = aux_torque_3 + Dynamic_Torque_Value_Interval_3;
                    }
                }
                cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb!" << endl;
            }
        }
    }
    //cout << aux_torque_3 << endl;
    Trigger_counter_3++;
    if (Trigger_counter_3 > Trigger_counter_No_3)
    {
        Trigger_counter_3 = 0;
        Pre_Trigger_pos_3 = Trigger_pos_3;
    }
    pre_pos_3 = pos;
    pre_vel_3 = vel;

    t = t + dt;
    if (t > test_time)
    {
        aux_torque_3 = 0;
    }
    //if (start_move_2 == true)
    //      aux_torque_3 = 0;
    return aux_torque_3;
}

double ArmController::Auxiliary_Torque_Axis4(double pos, double vel)
{
    enum motion_trend acc_trend4;
    enum motion_trend vel_trend4;
    if (ini_flag_4 == false)
    {
        ini_flag_4 = true;
        pre_pos_4 = pos;
        pre_vel_4 = vel;
        Trigger_pos_4 = pos;
        Pre_Trigger_pos_4 = Trigger_pos_4;
        aux_torque_4 = 0;
        Static_Torque_Counter_4 = 0;
        start_move_4 = false;
        dwell_flag_4 = false;
        pos_stop_already_4 = true;
        neg_stop_already_4 = true;
        dwell_counter_4 = 0;
        Trigger_counter_4 = 0;
        Start_move_deg_4 = 0.025;
        Stop_move_deg_4 = 1;
        Trigger_counter_No_4 = 5;
        Static_Torque_Counter_No_4 = 100;
        Static_Torque_Value1_4 = 50;
        Static_Torque_Value2_4 = 10;
        Dynamic_Torque_Value_4 = 25;
        Dynamic_Torque_Value_Interval_4 = 5;
        Dynamic_Torque_Low_Velocity_4 = 30;
        Dynamic_Torque_Hi_Velocity_4 = 60;
        Dwell_counter_value_4 = 300;
        Static_Torque_Counter_No_Percentage_4 = 0.7;
    }
    Trigger_pos_4 = pos;
    if ((((Trigger_pos_4 - Pre_Trigger_pos_4) > Start_move_deg_4 && neg_stop_already_4 == true) || ((Trigger_pos_4 - Pre_Trigger_pos_4) < -Start_move_deg_4 && pos_stop_already_4 == true)) && Trigger_counter_4 > (Trigger_counter_No_4 - 1))
    {
        if (start_move_4 == false)
        {
            start_move_4 = true;
            cout << "move44444444444444444444444444444444444444!" << endl;
        }

    }

    if (dwell_flag_4 == true)
    {
        dwell_counter_4++;
        aux_torque_4 = 0;
        if (dwell_counter_4 > Dwell_counter_value_4)
        {
            dwell_counter_4 = 0;
            dwell_flag_4 = false;
            pos_stop_already_4 = true;
            neg_stop_already_4 = true;
            Static_Torque_Counter_4 = 0;
        }
    }

    if (start_move_4 == true)
    {
        if ((fabs(pos - pre_pos_4) < Stop_move_deg_4) && Static_Torque_Counter_4 >= Static_Torque_Counter_No_4)
        {
            start_move_4 = false;
            dwell_flag_4 = true;
            neg_stop_already_4 = false;
            pos_stop_already_4 = false;
            cout << "stop44444444444444444444444444444444444444!" << endl;
        }
        else //while moving
        {
            if (Static_Torque_Counter_4 <= Static_Torque_Counter_No_4)
            {
                if ((Trigger_pos_4 - Pre_Trigger_pos_4) >= 0 && Trigger_counter_4 > (Trigger_counter_No_4 - 1) && neg_stop_already_4 == true)
                {
                    neg_stop_already_4 = false;
                    if (Static_Torque_Counter_4 <= (Static_Torque_Counter_No_4 * Static_Torque_Counter_No_Percentage_4))
                        aux_torque_4 = Static_Torque_Value1_4;
                    else
                        aux_torque_4 = Static_Torque_Value2_4;
                }
                if ((Trigger_pos_4 - Pre_Trigger_pos_4) < 0 && Trigger_counter_4 > (Trigger_counter_No_4 - 1) && pos_stop_already_4 == true)
                {
                    pos_stop_already_4 = false;
                    if (Static_Torque_Counter_4 <= (Static_Torque_Counter_No_4 * Static_Torque_Counter_No_Percentage_4))
                        aux_torque_4 = -Static_Torque_Value1_4;
                    else
                        aux_torque_4 = -Static_Torque_Value2_4;
                }
                Static_Torque_Counter_4 = Static_Torque_Counter_4 + 1;
                //char showcounter[100];
                //sprintf(showcounter, "Static_Torque_Counter_4=%d", Static_Torque_Counter_4);
                cout << "Static_Torque_Counter_4=%d" << endl;
            }
            else if (fabs(vel - pre_vel_4) < Dynamic_Torque_Low_Velocity_4) //vel too slow
            {
                if ((pos - pre_pos_4) >= 0)
                {
                    if (aux_torque_4 <= Dynamic_Torque_Value_4)
                        aux_torque_4 = aux_torque_4 + Dynamic_Torque_Value_Interval_4;
                }
                else
                {
                    if (aux_torque_4 >= -Dynamic_Torque_Value_4)
                        aux_torque_4 = aux_torque_4 - Dynamic_Torque_Value_Interval_4;
                }
                cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa!" << endl;
            }
            else if (fabs(vel - pre_vel_4) > Dynamic_Torque_Hi_Velocity_4) //vel too fast
            {
                if ((pos - pre_pos_4) >= 0)
                {
                    if (aux_torque_4 > 0)
                    {
                        aux_torque_4 = aux_torque_4 - Dynamic_Torque_Value_Interval_4;
                        cout << "b" << endl;
                    }
                }
                else
                {
                    if (aux_torque_4 < 0)
                    {
                        aux_torque_4 = aux_torque_4 + Dynamic_Torque_Value_Interval_4;
                    }
                }
                cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb!" << endl;
            }
        }
    }
    //cout << aux_torque_4 << endl;
    Trigger_counter_4++;
    if (Trigger_counter_4 > Trigger_counter_No_4)
    {
        Trigger_counter_4 = 0;
        Pre_Trigger_pos_4 = Trigger_pos_4;
    }
    pre_pos_4 = pos;
    pre_vel_4 = vel;

    t = t + dt;
    if (t > test_time)
    {
        aux_torque_4 = 0;
    }
    return aux_torque_4;
}

double ArmController::Auxiliary_Torque_Axis5(double pos, double vel)
{
    enum motion_trend acc_trend5;
    enum motion_trend vel_trend5;
    if (ini_flag_5 == false)
    {
        ini_flag_5 = true;
        pre_pos_5 = pos;
        pre_vel_5 = vel;
        Trigger_pos_5 = pos;
        Pre_Trigger_pos_5 = Trigger_pos_5;
        aux_torque_5 = 0;
        Static_Torque_Counter_5 = 0;
        start_move_5 = false;
        dwell_flag_5 = false;
        pos_stop_already_5 = true;
        neg_stop_already_5 = true;
        dwell_counter_5 = 0;
        Trigger_counter_5 = 0;
        Start_move_deg_5 = 0.025;
        Stop_move_deg_5 = 1;
        Trigger_counter_No_5 = 5;
        Static_Torque_Counter_No_5 = 100;
        Static_Torque_Value1_5 = 50;
        Static_Torque_Value2_5 = 10;
        Dynamic_Torque_Value_5 = 25;
        Dynamic_Torque_Value_Interval_5 = 5;
        Dynamic_Torque_Low_Velocity_5 = 30;
        Dynamic_Torque_Hi_Velocity_5 = 60;
        Dwell_counter_value_5 = 300;
        Static_Torque_Counter_No_Percentage_5 = 0.7;
    }
    Trigger_pos_5 = pos;
    if ((((Trigger_pos_5 - Pre_Trigger_pos_5) > Start_move_deg_5 && neg_stop_already_5 == true) || ((Trigger_pos_5 - Pre_Trigger_pos_5) < -Start_move_deg_5 && pos_stop_already_5 == true)) && Trigger_counter_5 > (Trigger_counter_No_5 - 1))
    {
        if (start_move_5 == false)
        {
            start_move_5 = true;
            cout << "move55555555555555555555555555555555555555!" << endl;
        }

    }

    if (dwell_flag_5 == true)
    {
        dwell_counter_5++;
        aux_torque_5 = 0;
        if (dwell_counter_5 > Dwell_counter_value_5)
        {
            dwell_counter_5 = 0;
            dwell_flag_5 = false;
            pos_stop_already_5 = true;
            neg_stop_already_5 = true;
            Static_Torque_Counter_5 = 0;
        }
    }

    if (start_move_5 == true)
    {
        if ((fabs(pos - pre_pos_5) < Stop_move_deg_5) && Static_Torque_Counter_5 >= Static_Torque_Counter_No_5)
        {
            start_move_5 = false;
            dwell_flag_5 = true;
            neg_stop_already_5 = false;
            pos_stop_already_5 = false;
            cout << "stop55555555555555555555555555555555555555!" << endl;
        }
        else //while moving
        {
            if (Static_Torque_Counter_5 <= Static_Torque_Counter_No_5)
            {
                if ((Trigger_pos_5 - Pre_Trigger_pos_5) >= 0 && Trigger_counter_5 > (Trigger_counter_No_5 - 1) && neg_stop_already_5 == true)
                {
                    neg_stop_already_5 = false;
                    if (Static_Torque_Counter_5 <= (Static_Torque_Counter_No_5 * Static_Torque_Counter_No_Percentage_5))
                        aux_torque_5 = Static_Torque_Value1_5;
                    else
                        aux_torque_5 = Static_Torque_Value2_5;
                }
                if ((Trigger_pos_5 - Pre_Trigger_pos_5) < 0 && Trigger_counter_5 > (Trigger_counter_No_5 - 1) && pos_stop_already_5 == true)
                {
                    pos_stop_already_5 = false;
                    if (Static_Torque_Counter_5 <= (Static_Torque_Counter_No_5 * Static_Torque_Counter_No_Percentage_5))
                        aux_torque_5 = -Static_Torque_Value1_5;
                    else
                        aux_torque_5 = -Static_Torque_Value2_5;
                }
                Static_Torque_Counter_5 = Static_Torque_Counter_5 + 1;
                //char showcounter[100];
                //sprintf(showcounter, "Static_Torque_Counter_5=%d", Static_Torque_Counter_5);
                cout << "Static_Torque_Counter_5=%d" << endl;
            }
            else if (fabs(vel - pre_vel_5) < Dynamic_Torque_Low_Velocity_5) //vel too slow
            {
                if ((pos - pre_pos_5) >= 0)
                {
                    if (aux_torque_5 <= Dynamic_Torque_Value_5)
                        aux_torque_5 = aux_torque_5 + Dynamic_Torque_Value_Interval_5;
                }
                else
                {
                    if (aux_torque_5 >= -Dynamic_Torque_Value_5)
                        aux_torque_5 = aux_torque_5 - Dynamic_Torque_Value_Interval_5;
                }
                cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa!" << endl;
            }
            else if (fabs(vel - pre_vel_5) > Dynamic_Torque_Hi_Velocity_5) //vel too fast
            {
                if ((pos - pre_pos_5) >= 0)
                {
                    if (aux_torque_5 > 0)
                    {
                        aux_torque_5 = aux_torque_5 - Dynamic_Torque_Value_Interval_5;
                        cout << "b" << endl;
                    }
                }
                else
                {
                    if (aux_torque_5 < 0)
                    {
                        aux_torque_5 = aux_torque_5 + Dynamic_Torque_Value_Interval_5;
                    }
                }
                cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb!" << endl;
            }
        }
    }
    //cout << aux_torque_5 << endl;
    Trigger_counter_5++;
    if (Trigger_counter_5 > Trigger_counter_No_5)
    {
        Trigger_counter_5 = 0;
        Pre_Trigger_pos_5 = Trigger_pos_5;
    }
    pre_pos_5 = pos;
    pre_vel_5 = vel;

    t = t + dt;
    if (t > test_time)
    {
        aux_torque_5 = 0;
    }
    return aux_torque_5;
}

double ArmController::Auxiliary_Torque_Axis6(double pos, double vel)
{
    enum motion_trend acc_trend6;
    enum motion_trend vel_trend6;
    if (ini_flag_6 == false)
    {
        ini_flag_6 = true;
        pre_pos_6 = pos;
        pre_vel_6 = vel;
        Trigger_pos_6 = pos;
        Pre_Trigger_pos_6 = Trigger_pos_6;
        aux_torque_6 = 0;
        Static_Torque_Counter_6 = 0;
        start_move_6 = false;
        dwell_flag_6 = false;
        pos_stop_already_6 = true;
        neg_stop_already_6 = true;
        dwell_counter_6 = 0;
        Trigger_counter_6 = 0;
        Start_move_deg_6 = 0.025;
        Stop_move_deg_6 = 1;
        Trigger_counter_No_6 = 5;
        Static_Torque_Counter_No_6 = 100;
        Static_Torque_Value1_6 = 50;
        Static_Torque_Value2_6 = 10;
        Dynamic_Torque_Value_6 = 25;
        Dynamic_Torque_Value_Interval_6 = 5;
        Dynamic_Torque_Low_Velocity_6 = 30;
        Dynamic_Torque_Hi_Velocity_6 = 60;
        Dwell_counter_value_6 = 300;
        Static_Torque_Counter_No_Percentage_6 = 0.7;
    }
    Trigger_pos_6 = pos;
    if ((((Trigger_pos_6 - Pre_Trigger_pos_6) > Start_move_deg_6 && neg_stop_already_6 == true) || ((Trigger_pos_6 - Pre_Trigger_pos_6) < -Start_move_deg_6 && pos_stop_already_6 == true)) && Trigger_counter_6 > (Trigger_counter_No_6 - 1))
    {
        if (start_move_6 == false)
        {
            start_move_6 = true;
            cout << "move66666666666666666666666666666666666666!" << endl;
        }

    }

    if (dwell_flag_6 == true)
    {
        dwell_counter_6++;
        aux_torque_6 = 0;
        if (dwell_counter_6 > Dwell_counter_value_6)
        {
            dwell_counter_6 = 0;
            dwell_flag_6 = false;
            pos_stop_already_6 = true;
            neg_stop_already_6 = true;
            Static_Torque_Counter_6 = 0;
        }
    }

    if (start_move_6 == true)
    {
        if ((fabs(pos - pre_pos_6) < Stop_move_deg_6) && Static_Torque_Counter_6 >= Static_Torque_Counter_No_6)
        {
            start_move_6 = false;
            dwell_flag_6 = true;
            neg_stop_already_6 = false;
            pos_stop_already_6 = false;
            cout << "stop66666666666666666666666666666666666666!" << endl;
        }
        else //while moving
        {
            if (Static_Torque_Counter_6 <= Static_Torque_Counter_No_6)
            {
                if ((Trigger_pos_6 - Pre_Trigger_pos_6) >= 0 && Trigger_counter_6 > (Trigger_counter_No_6 - 1) && neg_stop_already_6 == true)
                {
                    neg_stop_already_6 = false;
                    if (Static_Torque_Counter_6 <= (Static_Torque_Counter_No_6 * Static_Torque_Counter_No_Percentage_6))
                        aux_torque_6 = Static_Torque_Value1_6;
                    else
                        aux_torque_6 = Static_Torque_Value2_6;
                }
                if ((Trigger_pos_6 - Pre_Trigger_pos_6) < 0 && Trigger_counter_6 > (Trigger_counter_No_6 - 1) && pos_stop_already_6 == true)
                {
                    pos_stop_already_6 = false;
                    if (Static_Torque_Counter_6 <= (Static_Torque_Counter_No_6 * Static_Torque_Counter_No_Percentage_6))
                        aux_torque_6 = -Static_Torque_Value1_6;
                    else
                        aux_torque_6 = -Static_Torque_Value2_6;
                }
                Static_Torque_Counter_6 = Static_Torque_Counter_6 + 1;
                //char showcounter[100];
                //sprintf(showcounter, "Static_Torque_Counter_6=%d", Static_Torque_Counter_6);
                cout << "Static_Torque_Counter_6=%d" << endl;
            }
            else if (fabs(vel - pre_vel_6) < Dynamic_Torque_Low_Velocity_6) //vel too slow
            {
                if ((pos - pre_pos_6) >= 0)
                {
                    if (aux_torque_6 <= Dynamic_Torque_Value_6)
                        aux_torque_6 = aux_torque_6 + Dynamic_Torque_Value_Interval_6;
                }
                else
                {
                    if (aux_torque_6 >= -Dynamic_Torque_Value_6)
                        aux_torque_6 = aux_torque_6 - Dynamic_Torque_Value_Interval_6;
                }
                cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa!" << endl;
            }
            else if (fabs(vel - pre_vel_6) > Dynamic_Torque_Hi_Velocity_6) //vel too fast
            {
                if ((pos - pre_pos_6) >= 0)
                {
                    if (aux_torque_6 > 0)
                    {
                        aux_torque_6 = aux_torque_6 - Dynamic_Torque_Value_Interval_6;
                        cout << "b" << endl;
                    }
                }
                else
                {
                    if (aux_torque_6 < 0)
                    {
                        aux_torque_6 = aux_torque_6 + Dynamic_Torque_Value_Interval_6;
                    }
                }
                cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb!" << endl;
            }
        }
    }
    //cout << aux_torque_6 << endl;
    Trigger_counter_6++;
    if (Trigger_counter_6 > Trigger_counter_No_6)
    {
        Trigger_counter_6 = 0;
        Pre_Trigger_pos_6 = Trigger_pos_6;
    }
    pre_pos_6 = pos;
    pre_vel_6 = vel;

    t = t + dt;
    if (t > test_time)
    {
        aux_torque_6 = 0;
    }
    return aux_torque_6;
}

