//#include <webots/motor.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include "controller.hpp"
#include "webotsInterface.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <webots/Keyboard.hpp>
#include <webots/DistanceSensor.hpp>

using namespace Eigen;
void update_xz_dot_desire();

void rot_X(Matrix3d& Mat, double angle)
{
    double a = angle * PI / 180.0f;

    Mat(1,1) = cosf(a);
    Mat(1,2) = -sinf(a);
    Mat(2,1) = sinf(a);
    Mat(2,2) = cosf(a);
}

void rot_Y(Matrix3d& Mat, double angle)
{
    double a = angle * PI / 180.0f;

    Mat(0,0) = cosf(a);
    Mat(0,2) = sinf(a);
    Mat(2,0) = -sinf(a);
    Mat(2,2) = cosf(a);
}


void rot_Z(Matrix3d& Mat, double angle)
{
    double a = angle * PI / 180.0f;

    Mat(0,0) = cosf(a);
    Mat(0,1) = -sinf(a);
    Mat(1,0) = sinf(a);
    Mat(1,1) = cosf(a);
}
void calculate_RPY(Matrix3d& out_rot, double roll, double pitch, double yaw){
    Matrix3d rot_x, rot_y, rot_z, temp;
    out_rot.setOnes();

    rot_X(rot_x.setIdentity(), roll);
    rot_Y(rot_y.setIdentity(), yaw);
    rot_Z(rot_z.setIdentity(), pitch);

    temp = rot_z * rot_x;
    out_rot = rot_y * temp;

}



MyRobot::MyRobot() {
    bool temp = true;
}
MyRobot::~MyRobot() noexcept = default;

void MyRobot::update_robot_state() {
    // change coordinate
    // change time




    system_ms += time_step;
    // change x and z
    update_xz_dot_desire();
    // use interface to identify whether foot touch the ground
    is_touching_ground = is_foot_touching(); // todo go ahead
    euler_angle_typedef now_IMU = get_IMU_value();
    euler_angle_typedef now_IMU_dot;

    now_IMU_dot.roll = (now_IMU.roll - euler_angle.roll) / (0.001 * TIME_STEP);
    now_IMU_dot.pitch = (now_IMU.pitch - euler_angle.pitch) / (0.001 * TIME_STEP);
    now_IMU_dot.yaw = (now_IMU.yaw - euler_angle.yaw) / (0.001 * TIME_STEP);
    euler_angle = now_IMU;

    euler_angle_dot.roll = (now_IMU_dot.roll + euler_angle_dot.roll) / 2;
    euler_angle_dot.yaw = (now_IMU_dot.yaw + euler_angle_dot.yaw) / 2;
    euler_angle_dot.pitch = (now_IMU_dot.pitch + euler_angle_dot.pitch) / 2;

    calculate_RPY(R_H_B, euler_angle.roll, euler_angle.pitch, euler_angle.yaw);
    R_B_H = R_H_B.transpose();

    double now_r = get_spring_length();
    double now_r_dot = (now_r - joint_point.r) / (0.001*TIME_STEP);
    joint_point.r = now_r;
    joint_point_dot.r = (now_r_dot + joint_point_dot.r) /2;

    double now_X_motor_angle = get_X_motor_angle();
    double now_X_motor_angle_dot = (now_X_motor_angle - joint_point.X_motor_angle) / (0.001*TIME_STEP);
    joint_point.X_motor_angle = now_X_motor_angle;
    joint_point_dot.X_motor_angle = (now_X_motor_angle_dot + joint_point_dot.X_motor_angle)/2;

    double now_Z_motor_angle = get_Z_motor_angle();
    double now_Z_motor_angle_dot = (now_Z_motor_angle - joint_point.Z_motor_angle) / (0.001*TIME_STEP);
    joint_point.Z_motor_angle = now_Z_motor_angle;
    joint_point_dot.Z_motor_angle = (now_Z_motor_angle_dot + joint_point_dot.Z_motor_angle)/2;

    update_xz_dot();

    update_last_Ts();

    update_robot_state_machine();

}

void MyRobot::robot_control() {
    double dx = spring_normal_length - joint_point.r ; // minus and get the difference
    double F_spring = dx * spring_k;

    if (robot_state == THRUST)
    {
        F_spring += F_thrust; // leave the ground ,push it
    }
    set_spring_force(F_spring);

    if(robot_state == LOADING || robot_state == UNLOADING)
    {
        set_X_torque(0);
        set_Z_torque(0);
    }

    if(robot_state == COMPRESSING || robot_state == THRUST)
    {
        double Tx, Tz;
        Tx = -(-k_p_pose * euler_angle.roll - k_v_pose * euler_angle_dot.roll);
        Tz = -(-k_p_pose * euler_angle.pitch - k_v_pose * euler_angle_dot.pitch);

        set_Z_torque(Tz);
        set_X_torque(Tx);
    }

    if(robot_state == FLIGHT)
    {
        double Tx, Tz;
        double r = joint_point.r;

        double x_f = x_dot * Ts / 2.0 + k_xz_dot * (x_dot - x_dot_desire);
        double z_f = z_dot * Ts / 2.0 + k_xz_dot * (z_dot - z_dot_desire);
        double y_f = -sqrt( r * r - x_f * x_f - z_f * z_f);// todo how ?

        work_point_H_desire[0] = x_f;
        work_point_H_desire[1] = y_f;
        work_point_H_desire[2] = z_f;

        work_point_B_desire = R_B_H * work_point_H_desire;

        double x_f_b = work_point_H_desire[0];
        double x_angle_desire = atan(z_f / y_f) *180 /PI;
        double z_angle_desire = atan(z_f / r) * 180 /PI;

        double x_angle     =joint_point.X_motor_angle;
        double z_angle     =joint_point.Z_motor_angle;
        double x_angle_dot =joint_point_dot.X_motor_angle;
        double z_angle_dot =joint_point_dot.Z_motor_angle;
        Tx = - k_p_leg * (x_angle - x_angle_desire) - k_v_leg * x_angle_dot;
        Tz = - k_p_leg * (z_angle - z_angle_desire) - k_v_leg * z_angle_dot;

        set_X_torque(Tx);
        set_X_torque(Tz);
    }

}

void MyRobot::forward_kinematics(Matrix3d *work_point, Vector3d *joint_point) {

}

void MyRobot::update_xz_dot_desire() {

}

void MyRobot :: update_xz_dot() {
    webots::Keyboard temp ;
    switch(get_keyboard())
    {
        case temp.UP:
        {
             x_dot_desire =  v;
            break;
        }
        case temp.DOWN:
        {
             x_dot_desire = - v;
            break;
        }
        case temp.RIGHT:
        {
             z_dot_desire =  v;
            break;
        }
        case temp.LEFT:
        {
             z_dot_desire = - v;
            break;
        }
        default:
        {
             z_dot_desire = 0;
             x_dot_desire = 0;
            break;
        }

    }

void MyRobot::update_last_Ts() {

}

void MyRobot::robot_init() {

}

void MyRobot::robot_free() {


}

void MyRobot::update_robot_state_machine() {

}

