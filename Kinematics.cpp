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

KinRes Kinematics::PtSetter(Vector4d goal_vector, Matrix4d deburringT06, array<double, AXISNUM>& fend_pose)
{

    Matrix3d R06;
    R06(0, 0) = deburringT06(0, 0); R06(0, 1) = deburringT06(0, 1); R06(0, 2) = deburringT06(0, 2);
    R06(1, 0) = deburringT06(1, 0); R06(1, 1) = deburringT06(1, 1); R06(1, 2) = deburringT06(1, 2);
    R06(2, 0) = deburringT06(2, 0); R06(2, 1) = deburringT06(2, 1); R06(2, 2) = deburringT06(2, 2);
        
    //cout << fend_point << endl;
    for (int i = 0; i < 3; i++)
    {
        fend_pose[i] = goal_vector[i];
    }
    for (int i = 3; i < 6; i++)
    {
        fend_pose[i] = goal_pose[i-3];
    }
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
    _T06(0, 0) = rounding(_T06(0, 0)); _T06(0, 1) = rounding(_T06(0, 1)); _T06(0, 2) = rounding(_T06(0, 2)); _T06(0, 3) = rounding(_T06(0, 3));
    _T06(1, 0) = rounding(_T06(1, 0)); _T06(1, 1) = rounding(_T06(1, 1)); _T06(1, 2) = rounding(_T06(1, 2)); _T06(1, 3) = rounding(_T06(1, 3));
    _T06(2, 0) = rounding(_T06(2, 0)); _T06(2, 1) = rounding(_T06(2, 1)); _T06(2, 2) = rounding(_T06(2, 2)); _T06(2, 3) = rounding(_T06(2, 3));
    _T06(3, 0) = rounding(_T06(3, 0)); _T06(3, 1) = rounding(_T06(3, 1)); _T06(3, 2) = rounding(_T06(3, 2)); _T06(3, 3) = rounding(_T06(3, 3));

    //cout << _T06 << endl;
    
    robot_pose[0] = (_T06(0, 3));
    robot_pose[1] = (_T06(1, 3));
    robot_pose[2] = (_T06(2, 3));
#if 1   
    robot_pose[4] = RAD2DEG * atan2(-_T06(2, 0), sqrt(_T06(0, 0) * _T06(0, 0) + _T06(1, 0) * _T06(1, 0))); //pitch
    //cout << "cos(roll) = " << cos(robot_pose[4]) << endl;
    //cout << "front = " << _T06(2, 1) / cos(robot_pose[4]) << endl;
    //cout << "rear = " << _T06(2, 2) / cos(robot_pose[4]) << endl;

    robot_pose[3] = RAD2DEG * atan2((_T06(2, 1)) / cos(robot_pose[4]), (_T06(2, 2)) / cos(robot_pose[4])); //roll
    robot_pose[5] = RAD2DEG * atan2((_T06(1, 0)) / cos(robot_pose[4]), (_T06(0, 0)) / cos(robot_pose[4])); //yaw
    /*cout << "robot pose = "
        << robot_pose[0] << " , "
        << robot_pose[1] << " , "
        << robot_pose[2] << " , "
        << robot_pose[3] << " , "
        << robot_pose[4] << " , "
        << robot_pose[5] << endl;
        */
#else
    array<double, 3> ABC;
    Matrix3d RT;
    RT(0, 0) = _T06(0, 0); RT(0, 1) = _T06(0, 1); RT(0, 2) = _T06(0, 2);
    RT(1, 0) = _T06(1, 0); RT(1, 1) = _T06(1, 1); RT(1, 2) = _T06(1, 2);
    RT(2, 0) = _T06(2, 0); RT(2, 1) = _T06(2, 1); RT(2, 2) = _T06(2, 2);
    RT2ABC(ABC, RT);
    robot_pose[3] = ABC[0];
    robot_pose[4] = ABC[1];
    robot_pose[5] = ABC[2];
    
#endif
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
    T06(0, 0) = rounding(T06(0, 0)); T06(0, 1) = rounding(T06(0, 1)); T06(0, 2) = rounding(T06(0, 2)); T06(0, 3) = rounding(T06(0, 3));
    T06(1, 0) = rounding(T06(1, 0)); T06(1, 1) = rounding(T06(1, 1)); T06(1, 2) = rounding(T06(1, 2)); T06(1, 3) = rounding(T06(1, 3));
    T06(2, 0) = rounding(T06(2, 0)); T06(2, 1) = rounding(T06(2, 1)); T06(2, 2) = rounding(T06(2, 2)); T06(2, 3) = rounding(T06(2, 3));
    T06(3, 0) = rounding(T06(3, 0)); T06(3, 1) = rounding(T06(3, 1)); T06(3, 2) = rounding(T06(3, 2)); T06(3, 3) = rounding(T06(3, 3));
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

double Kinematics::rounding(double num)
{
    int index = 4; //to number 4 
    bool isNegative = false; // whether is negative number or not

    if (abs(num) < 0.00001)
    {
        num = 0;
        return num;
    }
    if (num < 0) // if this number is negative, then convert to positive number
    {
        isNegative = true;
        num = -num;
    }

    if (index >= 0)
    {
        int multiplier;
        multiplier = pow(10, index);
        num = (int)(num * multiplier + 0.5) / (multiplier * 1.0);
    }

    if (isNegative) // if this number is negative, then convert to negative number
    {
        num = -num;
    }

    return num;
}


