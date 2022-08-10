//
// Created by mzy on 2022/8/6.
//

#ifndef IMPE_CTRL_M2006_H
#define IMPE_CTRL_M2006_H


class PID
{
public:
    float _output;
    float _T;

    PID(){;}
    PID(float KP, float KI, float KD, float OutputMax, float OutputMin):_Kp(KP),_Ki(KI),_Kd(KD),_outputMax(OutputMax),_outputMin(OutputMin)
    {
        _output = 0;
        _err[0] = 0;
        _err[1] = 0;
    }
    ~PID(){;};

    void PID_Cal(float Ref, float Fdb, float T)
    {
        float err_now = Ref - Fdb;

        _output += _Kp*(err_now - _err[1]) + _Ki*T/2.0*err_now + _Kd/T*(err_now - 2*_err[0] + _err[1]);
        if(_output > _outputMax) _output = _outputMax;
        if(_output < _outputMin) _output = _outputMin;

        _err[1] = _err[0];
        _err[0] = err_now;
    }
    void PID_INTI(float KP, float KI, float KD, float OutputMax, float OutputMin)
    {
        _Kp = KP;
        _Ki = KI;
        _Kd = KD;
        _outputMax = OutputMax;
        _outputMin = OutputMin;

        _output = 0;
        _err[0] = 0;
        _err[1] = 0;
    }


private:
    float _Kp;
    float _Ki;
    float _Kd;
    float _outputMax;
    float _outputMin;

    float _err[2];
};



class M2006 {
public:
    int _id;
    float _angle_fdb;
    float _rpm_fdb;
    float _current_fdb;
    float _current_out;


    M2006(int Id)
    {
        _id = Id;
        _angle_fdb = 0;
        _rpm_fdb = 0;
        _current_fdb = 0;
        _current_out = 0;

        _pos_pid.PID_INTI(100, 0, 0, 15000, -15000);
        _speed_pid.PID_INTI(2.5, 0, 0.001, 4000, -4000);
    }
    ~M2006(){;}

    void PosCtrl(float Pos_ref, float T);
    void VelCtrl(float Vel_ref, float T);

    PID _pos_pid;
    PID _speed_pid;
private:


};


#endif //IMPE_CTRL_M2006_H
