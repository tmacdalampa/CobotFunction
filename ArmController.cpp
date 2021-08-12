#include "ArmController.h"
using namespace std;

enum motion_trend
{
    positive, negative, zero
};

ArmController::ArmController(pair<array<double, 6>, array<double, 6>> position, array<double, 3> gain)
{
    _Kp = gain[0];
    _Kd = gain[1];
    _Ki = gain[2];
    _init_position = position.first;
    _current_position = position.first;
    _goal_position = position.second;
    for (int i = 0; i < 6; i++)
    {
        _current_velocity[i] = 0;
        _pos_sum_error[i] = 0;
        _pos_error[i] = 0;
    }

}
ArmController::~ArmController(void)
{
}

void ArmController::Current_Position_Setter(array<double, 6> curr_position)
{
    _current_position = curr_position;
    return;
}

void ArmController::Current_Velocity_Setter(array<double, 6> curr_velocity)
{
    _current_velocity = curr_velocity;
    return;
}

array<double, 6> ArmController::UpdateGoalTorque()
{
    double goal_vel = 0;
    array<double, 6> goal_torque;
    
    
    for (int i = 0; i < 6; i++)
    {
        if (i == 5)
        {
            _pos_error[i] = _goal_position[i] - _current_position[i];
            goal_torque[i] = _Kp * (_pos_error[i]) + _Kd * (goal_vel - _current_velocity[i]) + _Ki * _pos_sum_error[i];
            if (goal_torque[i] > 120)
            {
                goal_torque[i] = 120;
                if (_Ki != 0)
                {
                    _pos_sum_error[i] = (goal_torque[i] - _Kp * _pos_error[i] - _Kd * (goal_vel - _current_velocity[i])) / _Ki;
                }
            }
            else if (goal_torque[i] < -120)
            {
                goal_torque[i] = -120;
                if (_Ki != 0)
                {
                    _pos_sum_error[i] = (goal_torque[i] - _Kp * _pos_error[i] - _Kd * (goal_vel - _current_velocity[i])) / _Ki;
                }
            }
            _pos_sum_error[i] = _pos_sum_error[i] + _pos_error[i] * 0.001;
        }
        else
        {
            goal_torque[i] = 0;
        }
        
        
    }
    return goal_torque;
}