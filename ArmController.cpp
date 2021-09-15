#include "ArmController.h"
using namespace std;

ArmController::ArmController(array<double, 6> init_position)
{
    for (int i = 0; i < 6; i++)
    {
        _current_position[i] = init_position[i];
    }
   
    t = 0;
    dt = 0.001;
    error = 0.0001;
    start_move_flag = false;
    motion_end_flag = false;
}
ArmController::~ArmController(void)
{
}
void ArmController::MoveLinear(array<double, 3> &goal_pose, double &vel, double &acc)
{
    array<double, 3> init_position = { -(_current_position[1] - AxisZeroPoint[1]) / AxisGearRatio[1] ,-(_current_position[2] - AxisZeroPoint[2]) / AxisGearRatio[2], (_current_position[4] - AxisZeroPoint[4]) / AxisGearRatio[4] };
    //cout << init_position[0]<< " , " << init_position[1] << " , " << init_position[2] << endl;
    array<double, 3> init_pose; 
    array<double, 3> goal_position;
    FK(init_position, init_pose);
    array<double, 3> pose = init_pose;
    double Ta = vel / acc;
    //cout << "Ta = " << Ta << endl;
    double L = hypot((goal_pose[0] - init_pose[0]) ,(goal_pose[1] - init_pose[1]));
    //cout << "L = " << L << endl;
    double Tt = (L / vel) + Ta;
    //cout << "Tt = " << Tt << endl;
    u = { (goal_pose[0] - init_pose[0])/L ,(goal_pose[1] - init_pose[1])/L };
    //cout << "u1 u2 = " << u[0] << " , " << u[1] << endl;
    double remain_L = L;
    double v_now = 0; double v_target = vel; double L_dcc = (0.5 * vel * vel) / acc; int sign; double ds; double v_next;
    while (1)
    {
        if (remain_L <= error)
        {
            ds = remain_L;
            for (int i = 0; i < 2; i++)
            {
                pose[i] = pose[i] + u[i] * ds;
            }
            remain_L = remain_L - ds;
            IK(goal_position, pose);
            //cout << goal_position[2] << endl;
            goal_position_q.push(goal_position);
            break;
        }
        if (remain_L <= L_dcc) //time to dcc
        {
            v_target = 0;
        }
        
        if (v_now != v_target) //acc
        {
            if (v_now > v_target) sign = -1;
            else sign = 1;
            v_next = v_now + sign * acc * dt;
            
            if (sign * v_next > sign * v_target) v_next = v_target;
            
            ds = (v_next + v_now) * dt * 0.5;
            v_now = v_next;
        }
        else
        {
            v_now = v_next;
            ds = v_next * dt;
        }

        for (int i = 0; i < 2; i++)
        {
            pose[i] = pose[i] + u[i] * ds;
        }
        remain_L = remain_L - ds;
        //cout << remain_L << endl;
        IK(goal_position, pose);
        //cout << goal_position[2] << endl;
        goal_position_q.push(goal_position);
    }
    start_move_flag = true;
    return;
}


void ArmController::FK(array<double, 3> &position, array<double, 3> &pose)
{   
    double l1, l2, l3;
    l1 = 0.425; l2 = 0.425; l3 = 0.0775;
    double s1 = position[0] / RAD2DEG; double s2 = position[1] / RAD2DEG; double s3 = position[2] / RAD2DEG;
    pose[0] = l1 * cos(s1) + l2 * cos(s1 + s2) + l3 * cos(s1 + s2 + s3);
    pose[1] = l1 * sin(s1) + l2 * sin(s1 + s2) + l3 * sin(s1 + s2 + s3);
    pose[2] = (s1 + s2 + s3)*RAD2DEG;
    //cout << pose[0] << " , " << pose[1] << " , " << pose[2] << endl;
}
void ArmController::IK(array<double, 3> &position, array<double, 3> &pose)
{
    double l1, l2, l3;
    l1 = 0.425; l2 = 0.425; l3 = 0.0775;
    double x = pose[0]; double y = pose[1]; double theta = pose[2] / RAD2DEG;
    array<double, 2> head = { x - l3 * cos(theta), y - l3 * sin(theta) }; 
    double s2 = acos((head[0]*head[0] + head[1]*head[1] -l1*l1-l2*l2) / (2 * l1 * l2));
    double A1  = l1 + l2 * cos(s2);
    double A2 = l2* sin(s2);
    double A3 =  -l2 * sin(s2);
    double A4 = l1 + l2 * cos(s2);
    double ss1 = (A3*head[0] + A4*head[1])/hypot(head[0], head[1]);
    double cs1 = (A1 * head[0] + A2 * head[1]) / hypot(head[0], head[1]);
    double s1 = atan2(ss1, cs1);
    position[0] = s1 * RAD2DEG;
    position[1] = s2 * RAD2DEG;
    position[2] = pose[2] - position[0] - position[1];
    //cout << position[0] << " , " << position[1] << " , " << position[2] << endl;
}

array<double, 6> ArmController::UpdatePosition()
{
    static array<double, 6> target_position;
    if (goal_position_q.empty() != true)
    {
        target_position[1] = AxisZeroPoint[1] - (goal_position_q.front()[0] * AxisGearRatio[1]);
        target_position[2] = AxisZeroPoint[2] - (goal_position_q.front()[1] * AxisGearRatio[2]);
        target_position[4] = AxisZeroPoint[4] + (goal_position_q.front()[2] * AxisGearRatio[4]);

        target_position[0] = AxisZeroPoint[0];
        target_position[3] = AxisZeroPoint[3];
        target_position[5] = AxisZeroPoint[5];
        goal_position_q.pop();
    }
    else
    {
        motion_end_flag = true;
    }
    //cout << target_position[4] << endl;
    return target_position;
}