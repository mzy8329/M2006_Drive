/**
 * @file main
 * @author mzy (mzy8329@163.com)
 * @brief 实现了阻抗控制电机
 * @version 0.1
 *
 * @copyright Copyright (c) 2022
 *
 */


#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "motor_serial/motor_data.h"
#include "motor_serial/motor_ctrl.h"



ros::Publisher ctrlData_pub;
motor_serial::motor_data motor;

#define _SIGN_(X) X > 0 ? 1:-1


double dT = 0;

double pos_ref = -30;
double vel_ref = 0;
double acc_ref = 0;
double eff_ref = 0.06;

double M_d = 1.5;
double B_d = 0.7;
double K_d = 15.0;

double M_q[1][1] = {{0.0008}};
double C_q[1][1] = {{0.000001}};
double g_q[1][1] = {{0}};

//位置补偿pid参数
float Kp_pos = 0.00001;
float Ki_pos = 0.0001;
float Kd_pos = 0.01;
float outputMax_pos = 0.8;
float outputMin_pos = 0;

double M2006_KT = 0.602;
double M2006_I_0 = 0.5;

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

PID_s pos_pid;



void impe_controller();

/**
 * @brief 读取motor反馈信号，同步控制
 * @param motor_msg
 */
void motorDataCallback(const motor_serial::motor_data::ConstPtr &motor_msg)
{
    //消除刚上电时的误差
    static int cnt = 0;

    //对力矩反馈量进行三次平均滤波
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


/**
 * @brief 进行阻抗控制
 */
void impe_controller()
{
    now = ros::Time::now();
    dT = now.toSec() - time_last.toSec();
    time_last = now;

    static double angle_last = 0;
    static double rpm_last = 0;
    static double torque_last = 0;

    acc_ref = (motor.rpm_fdb - rpm_last)/dT;

    /* 0.5*M_q[0][0]*acc_ref: 0.5 => 减小阻尼，使运动更平滑
     * (1 - M_q[0][0]/M_d)*(eff_ref - torque_last -  motor.torque_fdb*0.001)*0.1:
     * eff_ref - torque_last -  motor.torque_fdb*0.001 => 用上一时刻期望力矩减去电机的实际输出力矩，得到反馈力矩
     * 0.1 => 力矩绕动大，降低权重，减小绕动产生的影响
     * 0.001 => 单位换算
     */
    double torque = 0.5*M_q[0][0]*acc_ref + C_q[0][0]*motor.rpm_fdb + g_q[0][0] + M_q[0][0]*(1/M_d)*(B_d*(motor.rpm_fdb - vel_ref)+K_d*(motor.angle_fdb - pos_ref)) + (1 - M_q[0][0]/M_d)*(eff_ref - torque_last -  motor.torque_fdb*0.001)*0.1;

    // 添加位置补偿量，使其在外力变化后能够在一段时间后回到期望位置
    PID_Cal(&pos_pid, pos_ref, motor.angle_fdb);
    torque -= pos_pid.output;

    // M2006能输出最高力矩为1.6Nm
    torque = torque > 1.6 ? 1.6 : torque;
    torque = torque < -1.6 ? -1.6 : torque;

    //使用位置控制量时，其他两个控制量设为-1, 使用其他控制量同理
    motor_serial::motor_ctrl tempData;
    tempData.id = motor.id;
    tempData.angle_ref = -1;
    tempData.rpm_ref = -1;
    tempData.current_ref = 0;

    // 在工作范围外将电流设为0。 重要，pid补偿量超调严重，需要安全保护
    if(motor.angle_fdb > -240 && motor.angle_fdb < -5)
    {
        //控制量为电流， T = Kt(I - I_0)
        tempData.current_ref = ((eff_ref-torque)/M2006_KT + M2006_I_0 * (float)(_SIGN_((eff_ref-torque)))) * 1000.0;
    }
    else
    {
        tempData.current_ref = 0;
    }

    ctrlData_pub.publish(tempData);

    //debug 用，方便调参
    static int i = 0;
    if(i++ > 5)
    {
        ROS_INFO("ref:%f  angle_fdb: %f  cal:%f  1:%f 2:%f 3:%f pid_out:%f\n", tempData.current_ref, motor.angle_fdb, -torque/M2006_KT, -M_q[0][0]*(1/M_d)*B_d*(motor.rpm_fdb - vel_ref), -M_q[0][0]*(1/M_d)*K_d*(motor.angle_fdb - pos_ref), -(1 - M_q[0][0]/M_d)*(eff_ref - torque_last -  motor.torque_fdb*0.001)*0.1, pos_pid.output);
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

    PID_INIT(&pos_pid, Kp_pos, Ki_pos, Kd_pos, outputMax_pos, outputMin_pos);

    //最高支持4个电机，这里只用到一个
    motor.id = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}