#include "Kinematics.h"

using namespace std;
using namespace Eigen;


Kinematics::Kinematics(void)
{
    dh = new DHTable;
}

Kinematics::~Kinematics(void)
{
    delete dh;
}

KinRes Kinematics::PtSetter(queue<array<double, AXISNUM>> init_goal, array<double, AXISNUM> axis_deg, array<double, AXISNUM>& fstart_pose, array<double, AXISNUM>& fend_pose)
{
    Matrix4d T06;
    FK_R(axis_deg, T06);
    Matrix3d R06;
    R06(0, 0) = T06(0, 0); R06(0, 1) = T06(0, 1); R06(0, 2) = T06(0, 2);
    R06(1, 0) = T06(1, 0); R06(1, 1) = T06(1, 1); R06(1, 2) = T06(1, 2);
    R06(2, 0) = T06(2, 0); R06(2, 1) = T06(2, 1); R06(2, 2) = T06(2, 2);
    
    Matrix3d R_start, R_end;
    ABC2RT(init_goal.front()[3], init_goal.front()[4], init_goal.front()[5], R_start);
    ABC2RT(init_goal.back()[3], init_goal.back()[4], init_goal.back()[5], R_end);
    
    Matrix3d fR_start = R06 * R_start;
    Matrix3d fR_end = R06 * R_end;
    //cout << "fR_start = " << fR_start << endl;
    //cout << "fR_end = " << fR_end << endl;

    array<double, 3> start_pose;
    array<double, 3> end_pose;
    RT2ABC(start_pose, fR_start);
    RT2ABC(end_pose, fR_start);
    //cout << "start_pose = " << start_pose[0] << ", " << start_pose[1] << ", " << start_pose[2] << endl;
    //cout << "end_pose = " << end_pose[0] << ", " << end_pose[1] << ", " << end_pose[2] << endl;

    Vector4d start_point, end_point;
    start_point << init_goal.front()[0], init_goal.front()[1], init_goal.front()[2], 1;
    end_point << init_goal.back()[0], init_goal.back()[1], init_goal.back()[2], 1;
    Vector4d fstart_point, fend_point;
    fstart_point = T06 * start_point;
    fend_point = T06 * end_point;
    //cout << fstart_point << endl;
    //cout << fend_point << endl;
    for (int i = 0; i < 3; i++)
    {
        fstart_pose[i] = fstart_point[i];
        fend_pose[i] = fend_point[i];
    }
    for (int i = 3; i < 6; i++)
    {
        fstart_pose[i] = start_pose[i-3];
        fend_pose[i] = end_pose[i-3];
    }
#if 0
    for (int i = 0; i < 6; i++)
    {
        cout << fstart_pose[i] << endl;
        
    }
    for (int i = 0; i < 6; i++)
    {
        cout << fend_pose[i] << endl;
    }
#endif
    return KinRes::SUCCEED;
}

Matrix4d Kinematics::GetTFMatrix(double axis_deg, int id)
{
    Matrix4d out_T;
    double A, D, sa, ca, cs, ss;

    id = id - 1;

    A = dh->a[id];
    D = dh->d[id];

    sa = sin(dh->alpha[id] * DEG2RAD);
    ca = cos(dh->alpha[id] * DEG2RAD);

    ss = sin(axis_deg * DEG2RAD);
    cs = cos(axis_deg * DEG2RAD);

    out_T(0, 0) = cs;	out_T(0, 1) = -ss;	out_T(0, 2) = 0;    out_T(0, 3) = A;
    out_T(1, 0) = ss * ca;	out_T(1, 1) = cs * ca;   out_T(1, 2) = -sa;    out_T(1, 3) = -sa * D;
    out_T(2, 0) = ss * sa;	out_T(2, 1) = cs * sa;   out_T(2, 2) = ca;    out_T(2, 3) = ca * D;
    out_T(3, 0) = 0;	out_T(3, 1) = 0;   out_T(3, 2) = 0;    out_T(3, 3) = 1;

    /*
          [			 cos(£c)		  		 -sin(£c)		 0				  A	]
      T = [	sin(£c) * cos(£\)		 cos(£c) * cos(£\)	-sin(£\)		-sin(£\) * D	]
          [	sin(£c) * sin(£\)		 cos(£c) * sin(£\)	 cos(£\)		 cos(£\) * D	]
          [				 0					  0			 0				  1	]
    */
    return out_T;
}

KinRes Kinematics::FK(array<double, AXISNUM> axis_deg, array<double, AXISNUM>& robot_pose)
{
    _T01 = GetTFMatrix(axis_deg[0], 1);
    _T12 = GetTFMatrix(axis_deg[1], 2);
    _T23 = GetTFMatrix(axis_deg[2], 3);
    _T34 = GetTFMatrix(axis_deg[3], 4);
    _T45 = GetTFMatrix(axis_deg[4], 5);
    _T56 = GetTFMatrix(axis_deg[5], 6);
    _T06 = _T01 * _T12 * _T23 * _T34 * _T45 * _T56;
    robot_pose[0] = _T06(0, 3);
    robot_pose[1] = _T06(1, 3);
    robot_pose[2] = _T06(2, 3);
    robot_pose[4] = RAD2DEG * atan2(-_T06(2, 0), sqrt(_T06(0, 0) * _T06(0, 0) + _T06(1, 0) * _T06(1, 0))); //pitch
    robot_pose[3] = RAD2DEG * atan2(_T06(2, 1) / cos(robot_pose[4]), _T06(2, 2) / cos(robot_pose[4])); //roll
    robot_pose[5] = RAD2DEG * atan2(_T06(1, 0) / cos(robot_pose[4]), _T06(0, 0) / cos(robot_pose[4])); //yaw

    return KinRes::SUCCEED;
}

KinRes Kinematics::FK_R(array<double, AXISNUM> axis_deg, Matrix4d& T06)
{
    _T01 = GetTFMatrix(axis_deg[0], 1);
    _T12 = GetTFMatrix(axis_deg[1], 2);
    _T23 = GetTFMatrix(axis_deg[2], 3);
    _T34 = GetTFMatrix(axis_deg[3], 4);
    _T45 = GetTFMatrix(axis_deg[4], 5);
    _T56 = GetTFMatrix(axis_deg[5], 6);
    T06 = _T01 * _T12 * _T23 * _T34 * _T45 * _T56;
    //cout << "T06 = " << T06 << endl;
    return KinRes::SUCCEED;
}

double Kinematics::ChooseNearst(double a, double b, double c)
{
    double res;
    if (abs(a - c) <= abs(b - c))
        res = a;
    else
        res = b;
    return res;
}

KinRes Kinematics::IK(array<double, 6>& axis_deg, array<double, 6> robot_pose, array<double, 6> current_position)
{
    //cout << robot_pose[3] << " , " << robot_pose[4] << " , " << robot_pose[5] << endl;
    double roll = robot_pose[3] * DEG2RAD; double pitch = robot_pose[4] * DEG2RAD; double yaw = robot_pose[5] * DEG2RAD;
    //cout << roll << " , " << pitch << " , " << yaw << endl;

    double wrist_x = robot_pose[0] - dh->d[5] * (cos(roll) * cos(yaw) * sin(pitch) + sin(roll) * sin(yaw));
    double wrist_y = robot_pose[1] - dh->d[5] * (-cos(yaw) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw));
    double wrist_z = robot_pose[2] - dh->d[5] * cos(pitch) * cos(roll);
    //cout << wrist_y << " , "<< wrist_x << endl;
    double t1 = atan2(wrist_y, wrist_x);
    double t1_tmp = atan2(-wrist_y, -wrist_x);
    double axis1_position = ChooseNearst(t1, t1_tmp, current_position[0] * DEG2RAD); //axis 1
    //cout << axis1_position * RAD2DEG << endl;
    double x_dot = wrist_x * cos(axis1_position) + wrist_y * sin(axis1_position);
    double y_dot = wrist_y * cos(axis1_position) - wrist_x * sin(axis1_position);
    double z_dot = (wrist_z - dh->d[0]);
    double t3 = asin((x_dot * x_dot + z_dot * z_dot - dh->a[2] * dh->a[2] - dh->d[3] * dh->d[3]) / (2 * dh->a[2] * dh->d[3]));
    double t3_tmp = PI - t3;
    double axis3_position = ChooseNearst(t3, t3_tmp, current_position[2] * DEG2RAD); //axis3

    double f1 = dh->a[2] + dh->d[3] * sin(axis3_position);
    double f2 = -dh->d[3] * cos(axis3_position);
    double axis2_position;
    if (f2 * f2 + f1 * f1 >= x_dot * x_dot)
    {
        double u = (-f2 * x_dot + f1 * sqrt(f1 * f1 + f2 * f2 - x_dot * x_dot)) / (f1 * f1 + f2 * f2);
        double u_tmp = (-f2 * x_dot - f1 * sqrt(f1 * f1 + f2 * f2 - x_dot * x_dot)) / (f1 * f1 + f2 * f2);
        if (f2 == 0)
            axis2_position = 0;
        else
        {
            double t2 = atan2(u, (x_dot + f2 * u) / f1);
            double t2_tmp = atan2(u_tmp, (x_dot + f2 * u_tmp) / f1);
            //cout << t2 << " , " << t2_tmp << " , " << current_position[1] * DEG2RAD << endl;
            axis2_position = ChooseNearst(t2, t2_tmp, current_position[1] * DEG2RAD);
            //cout << axis2_position << endl;
        }
    }

    Matrix4d T06, inv_T03, T36;

    T06 << cos(yaw) * cos(pitch), cos(yaw)* sin(pitch)* sin(roll) - sin(yaw) * cos(roll), cos(yaw)* sin(pitch)* cos(roll) + sin(yaw) * sin(roll), robot_pose[0],
        sin(yaw)* cos(pitch), sin(yaw)* sin(pitch)* sin(roll) + cos(yaw) * cos(roll), sin(yaw)* sin(pitch)* cos(roll) - cos(yaw) * sin(roll), robot_pose[1],
        -sin(pitch), cos(pitch)* sin(roll), cos(pitch)* cos(roll), robot_pose[2],
        0, 0, 0, 1;

    inv_T03 << (cos(axis1_position) * cos(axis2_position + axis2_position)), (cos(axis2_position + axis3_position) * sin(axis1_position)), (sin(axis2_position + axis3_position)), (-dh->a[2] * cos(axis3_position) - dh->d[0] * sin(axis2_position + axis3_position)),
        (-cos(axis1_position) * sin(axis2_position + axis3_position)), (-sin(axis1_position) * sin(axis2_position + axis3_position)), (cos(axis2_position + axis3_position)), (dh->a[2] * sin(axis3_position) - dh->d[0] * cos(axis2_position + axis3_position)),
        (sin(axis1_position)), (-cos(axis1_position)), 0, 0,
        0, 0, 0, 1;

    T36 = inv_T03 * T06;

    double axis4_position, axis5_position, axis6_position;

    if (T36(1, 2) != -1 && T36(1, 2) != 1)
    {
        double t5 = atan2(sqrt(T36(1, 0) * T36(1, 0) + T36(1, 1) * T36(1, 1)), -T36(1, 2));
        double t5_tmp = atan2(-sqrt(T36(1, 0) * T36(1, 0) + T36(1, 1) * T36(1, 1)), -T36(1, 2));
        axis5_position = ChooseNearst(t5, t5_tmp, current_position[4] / RAD2DEG);

        axis4_position = atan2(-T36(2, 2) / sin(axis5_position), -T36(0, 2) / sin(axis5_position));
        axis6_position = atan2(T36(1, 1) / sin(axis5_position), -T36(1, 0) / sin(axis5_position));
    }
    else
    {
        //singular point
        axis4_position = 0.5 * atan2(T36(2, 0), T36(0, 0));
        axis5_position = 0;
        axis6_position = axis4_position;
    }

    axis_deg[0] = (axis1_position * RAD2DEG);
    axis_deg[1] = (axis2_position * RAD2DEG);
    axis_deg[2] = (axis3_position * RAD2DEG);
    axis_deg[3] = (axis4_position * RAD2DEG);
    axis_deg[4] = (axis5_position * RAD2DEG);
    axis_deg[5] = (axis6_position * RAD2DEG);

#if 0
    cout << axis_deg[0] << " , "
        << axis_deg[1] << " , "
        << axis_deg[2] << " , "
        << axis_deg[3] << " , "
        << axis_deg[4] << " , "
        << axis_deg[5] << " , " << endl;
#endif
    return KinRes::SUCCEED;
}

KinRes Kinematics::ABC2RT(double A, double B, double C, Matrix3d& RT)
{
    double roll = A * DEG2RAD;
    double pitch = B * DEG2RAD;
    double yaw = C * DEG2RAD;
    RT <<  cos(yaw) * cos(pitch), cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll), cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll),
    sin(yaw)* cos(pitch), sin(yaw)* sin(pitch)* sin(roll) + cos(yaw) * cos(roll), sin(yaw)* sin(pitch)* cos(roll) - cos(yaw) * sin(roll),
    -sin(pitch), cos(pitch)* sin(roll), cos(pitch)* cos(roll);
    //cout << RT << endl;
    return KinRes::SUCCEED;
}

KinRes Kinematics::RT2ABC(array<double, 3>& ABC, Matrix3d RT)
{
    double roll, pitch, yaw;
    pitch = atan2(-RT(2, 0), sqrt(RT(0, 0) * RT(0, 0) + RT(1, 0) * RT(1, 0)));

    if (cos(pitch) != 0)
    {
        yaw = atan2((RT(1, 0) / cos(pitch)), RT(0, 0) / cos(pitch));
        roll = atan2(RT(2, 1) / cos(pitch), RT(2, 2) / cos(pitch));
    }
    else
    {
        yaw = 0;
        roll = 0;
    }
    //cout << "roll, pitch, yaw = " << roll << ", " << pitch << ", " << yaw << endl;
    ABC[0] = roll * RAD2DEG;
    ABC[1] = pitch * RAD2DEG;
    ABC[2] = yaw * RAD2DEG;

    return KinRes::SUCCEED;
}

