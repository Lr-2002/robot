#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <Eigen/Dense>
#include "webotsInterface.hpp"
#include <webots/Robot.hpp>
using namespace Eigen;
enum STATE
{
    LOADING = 0x00,
    COMPRESSING = 0x01,
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
    void forward_kinematics(Matrix3d * work_point, Vector3d* joint_point);
    void update_xz_dot();
    void update_last_Ts();
    void update_xz_dot_desire();
    void robot_init();
    void robot_free();
    void update_robot_state_machine();
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



};

















#endif