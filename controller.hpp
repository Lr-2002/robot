#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
#include <Eigen/Dense>
#include "webotsInterface.hpp"
#include <webots/Robot.hpp>
using namespace Eigen;
using namespace webots;
typedef struct
{
    double roll;       //横滚，x轴
    double pitch;      //俯仰，z轴
    double yaw;        //偏航，y轴
}eulerAngleTypeDef;


enum STATE
{
    LOADING = 0x00,
    COMPRESSION = 0x01,
    THRUST = 0x02,
    UNLOADING = 0x03,
    FLIGHT = 0x04
};
typedef struct {
    double r;
    double X_motor_angle;
    double Z_motor_angle;
}joint_space_typedef;
class MyRobot : webots::Robot
{
public:

    // ------------------------- status feature initialize ------------------------------------//



    euler_angle_typedef euler_angle;
    euler_angle_typedef euler_angle_dot;
    Matrix3d R_H_B = Matrix3d :: Identity();
    Matrix3d R_B_H = Matrix3d :: Identity();
    joint_space_typedef joint_point;
    joint_space_typedef joint_point_dot;
    Vector3d work_point_B = Vector3d ::Zero();
    Vector3d work_point_H = Vector3d ::Zero();

    Vector3d work_point_B_desire = Vector3d ::Zero();
    Vector3d work_point_H_desire = Vector3d ::Zero();
    bool is_touching_ground = true;
    double Ts = 0;
    double x_dot = 0;
    double z_dot = 0;
    double x_dot_desire = 0;
    double z_dot_desire = 0;
    bool temp = false;
    int system_ms = 0;
    STATE robot_state = LOADING;

    MyRobot();
    ~MyRobot();
    void robot_control();
    void update_robot_state();
    void forward_kinematics(Vector3d & work_point, joint_space_typedef & joint_point);
    void update_xz_dot();
    void update_last_Ts();

    void run();
    void update_xz_dot_desire();
    void update_robot_state_machine();

    void set_spring_force(double force)
    {
        spring_motor->setForce(-force);
    }

    void set_X_torque(double torque)
    {
        X_motor->setTorque(torque);
    }

    void set_Z_torque(double torque)
    {
        Z_motor->setTorque(torque);
    }

    double get_spring_length()
    {
        double length = spring_position_sensor->getValue();
        return -length + 0.8;
    }

    double get_X_motor_angle()
    {
        double angle = X_motor_pos_sensor->getValue();
        return angle*180.0f/PI;
    }

    double get_Z_motor_angle()
    {
        double angle = Z_motor_pos_sensor->getValue();
        return angle*180.0f/PI;
    }

    bool is_foot_touching()
    {
        return touch_sensor->getValue();
    }

    euler_angle_typedef get_IMU_value()
    {
        const double * data = IMU->getRollPitchYaw();
        // return three angles
        euler_angle_typedef eulerAngle;
        eulerAngle.roll = data[0]*180.0f/PI;
        eulerAngle.pitch = data[1]*180.0f/PI;
        eulerAngle.yaw = data[2]*180.0f/PI;
        return eulerAngle;
    }

    int get_keyboard()
    {
        return keyboard.getKey();
    }

private:
    int time_step{};

    double spring_normal_length = 1.20 ;
    double v = 0.20 ;
    double r_threshold = 0.95;
    int spring_k = 2000;
    double F_thrust =100.0;
    int k_p_leg = 6; // todo specify the function ?
    double k_v_leg = 0.8; // N ~ m/ (rad/s)
    double k_xz_dot = 0.072;
    double k_p_pose = 0.8;
    double k_v_pose = 0.025;

    TouchSensor *touch_sensor;
    InertialUnit *IMU;
    Motor *spring_motor;
    Motor *Z_motor;
    Motor *X_motor;
    PositionSensor *spring_position_sensor;
    PositionSensor *X_motor_pos_sensor;
    PositionSensor *Z_motor_pos_sensor;
    Keyboard keyboard;


};

















#endif