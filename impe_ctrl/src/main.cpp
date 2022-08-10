#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "motor_serial/motor_data.h"
#include "motor_serial/motor_ctrl.h"

#include "M2006.h"

ros::Publisher ctrlData_pub;
//M2006 motor(0);
motor_serial::motor_data motor;

#define _SIGN_(X) X > 0 ? 1:-1


double dT = 0;


double pos_ref = -30;
double vel_ref = 0;
double acc_ref = 0;
double eff_ref = 0;
//double eff_ref = 0.0784;

double M_d = 1.5;
double B_d = 0.7;
double K_d = 3.0;

double M_q[1][1] = {{0.0008}};
double C_q[1][1] = {{0.000001}};
double g_q[1][1] = {{0}};



double M2006_KT = 0.602;
double M2006_I_0 = 0.4;

ros::Time now;
ros::Time time_last;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float err[2];

    float output;
    float outputMax;
    float outputMin;
}PID_s;

void PID_Cal(PID_s *pid, float ref, float fdb)
{
    float err_now = ref - fdb;
    pid->output += pid->Kp*(err_now - pid->err[1]) + pid->Ki*err_now + pid->Kd*(err_now - 2*pid->err[0] + pid->err[1]);
    if(pid->output > pid->outputMax)
    {
        pid->output = pid->outputMax;
    }
    if(pid->output < pid->outputMin)
    {
        pid->output = pid->outputMin;
    }

    pid->err[1] = pid->err[0];
    pid->err[0] = err_now;
}

void PID_INIT(PID_s *pid, float kp, float ki, float kd, float outputmax, float outputmin)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->outputMax = outputmax;
    pid->outputMin = outputmin;

    pid->output = 0;
    pid->err[0] = 0;
    pid->err[1] = 0;
}

void impe_controller();

void motorDataCallback(const motor_serial::motor_data::ConstPtr &motor_msg)
{
    static int cnt = 0;
    int N = 3;
    static int torque_last[3] = {0};

    if(motor.id == motor_msg->id)
    {
        motor.angle_fdb = motor_msg->angle_fdb;
        motor.rpm_fdb = motor_msg->rpm_fdb;

        torque_last[0] = motor_msg->torque_fdb;
        motor.torque_fdb = 0;
        for(int i = 0; i < N; i++)
        {
            motor.torque_fdb += torque_last[i];
        }
        motor.torque_fdb /= (float)N;
        for(int i = 0; i < N; i++)
        {
            torque_last[N-i] = torque_last[N-i-1];
        }
    }


    if(cnt <= 30 && motor.torque_fdb!= 0)
    {
        cnt++;
    }

    if(cnt > 30)
    {
        impe_controller();
    }

}


void impe_controller()
{
    now = ros::Time::now();
    dT = now.toSec() - time_last.toSec();
    time_last = now;

    static double angle_last = 0;
    static double rpm_last = 0;
    static double torque_last = 0;

    acc_ref = (motor.rpm_fdb - rpm_last)/dT;

    double torque = 0.5*M_q[0][0]*acc_ref + C_q[0][0]*motor.rpm_fdb + g_q[0][0] + M_q[0][0]*(1/M_d)*(B_d*(motor.rpm_fdb - vel_ref)+K_d*(motor.angle_fdb - pos_ref)) - (1 - M_q[0][0]/M_d)*(torque_last -  motor.torque_fdb*0.001)*0.1;
    torque = torque > 1.6 ? 1.6 : torque;
    torque = torque < -1.6 ? -1.6 : torque;

    //使用位置控制量时，其他两个控制量设为-1, 使用其他控制量同理
    motor_serial::motor_ctrl tempData;
    tempData.id = motor.id;
    tempData.angle_ref = -1;
    tempData.rpm_ref = -1;
    tempData.current_ref = 0;
    if(motor.angle_fdb > -240 && motor.angle_fdb < 1)
    {
        tempData.current_ref = ((eff_ref-torque)/M2006_KT + M2006_I_0 * (float)(_SIGN_((eff_ref-torque)))) * 1000.0;
    }
    else
    {
        tempData.current_ref = 0;
    }

    ctrlData_pub.publish(tempData);

    static int i = 0;
    if(i++ > 5)
    {
        ROS_INFO("ref:%f  angle_fdb: %f  cal:%f  1:%f 2:%f 3:%f\n", tempData.current_ref, motor.angle_fdb, -torque/M2006_KT, -M_q[0][0]*(1/M_d)*B_d*(vel_ref-motor.rpm_fdb), -M_q[0][0]*(1/M_d)*K_d*(pos_ref-motor.angle_fdb), -(1 - M_q[0][0]/M_d)*(torque_last -  motor.torque_fdb*0.001)*0.01);
        i = 0;
    }

    rpm_last = motor.rpm_fdb;
    torque_last = torque;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "impe_ctrl");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);


    ctrlData_pub = nh.advertise<motor_serial::motor_ctrl>("/motor_serial/ctrl_data", 10);
    ros::Subscriber motorData_sub = nh.subscribe("/motor_serial/motor_data", 10, motorDataCallback);

    now = ros::Time::now();
    time_last = now;


    motor.id = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}