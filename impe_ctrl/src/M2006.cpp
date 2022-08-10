//
// Created by mzy on 2022/8/6.
//

#include "M2006.h"

void M2006::PosCtrl(float Pos_ref, float T)
{
    _pos_pid.PID_Cal(Pos_ref, _angle_fdb, T);
    _speed_pid.PID_Cal(_pos_pid._output, _rpm_fdb, T);

    _current_out = _speed_pid._output;
}

void M2006::VelCtrl(float Vel_ref, float T)
{
    _speed_pid.PID_Cal(Vel_ref, _rpm_fdb, T);
    _current_out = _speed_pid._output;
}