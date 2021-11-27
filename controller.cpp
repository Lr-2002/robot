#include "controller.hpp"
#include <cmath>
#include <Eigen/Dense>

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

    touch_sensor = getTouchSensor("touch sensor");
    IMU = getInertialUnit("inertial unit");
    spring_motor = getMotor("spring motor");
    Z_motor = getMotor("ztational motor");
    X_motor = getMotor("x rotational motor");
    spring_position_sensor =  getPositionSensor("position sensor");
    X_motor_pos_sensor = getPositionSensor("x position sensor");
    Z_motor_pos_sensor = getPositionSensor("z position sensor");
    spring_position_sensor->enable(TIME_STEP);
    X_motor_pos_sensor->enable(TIME_STEP);
    Z_motor_pos_sensor->enable(TIME_STEP);
    touch_sensor->enable(TIME_STEP);
    keyboard.enable(TIME_STEP);
    IMU->enable(TIME_STEP);
}
MyRobot::~MyRobot() noexcept = default;

void MyRobot::run() {
    update_robot_state();
    robot_control();

}
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

    if(robot_state == COMPRESSION || robot_state == THRUST)
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

void MyRobot::forward_kinematics(Vector3d &work_point, joint_space_typedef & joint_point) {

    double Tx = joint_point.X_motor_angle*PI/180.0;
    double Tz = joint_point.Z_motor_angle*PI/180.0;
    double r  = joint_point.r;

    work_point[0] =  r*sin(Tz);
    work_point[1] = -r*cos(Tz)*cos(Tx);
    work_point[2] = -r*cos(Tz)*sin(Tx);
}

void MyRobot::update_xz_dot() {
    forward_kinematics(work_point_B, joint_point);


    //转换到{H}坐标系下
    double pre_x = work_point_H[0];
    double pre_z = work_point_H[2];
    work_point_H = R_H_B * work_point_B;
    double now_x = work_point_H[0];
    double now_z = work_point_H[2];
    //求导
    double now_x_dot = -(now_x - pre_x)/(0.001*TIME_STEP);
    double now_z_dot = -(now_z - pre_z)/(0.001*TIME_STEP);

    //滤波
    static double pre_x_dot = 0;
    static double pre_z_dot = 0;
    now_x_dot = pre_x_dot*0.5 + now_x_dot*0.5;
    now_z_dot = pre_z_dot*0.5 + now_z_dot*0.5;
    pre_x_dot = now_x_dot;
    pre_z_dot = now_z_dot;

    if((robot_state == COMPRESSION)||(robot_state ==  THRUST))
    {
        x_dot = now_x_dot;
        z_dot = now_z_dot;
    }
}

void MyRobot ::update_xz_dot_desire() {
    webots::Keyboard temp;
    switch (get_keyboard()) {
        case temp.UP: {
            x_dot_desire = v;
            break;
        }
        case temp.DOWN: {
            x_dot_desire = -v;
            break;
        }
        case temp.RIGHT: {
            z_dot_desire = v;
            break;
        }
        case temp.LEFT: {
            z_dot_desire = -v;
            break;
        }
        default: {
            z_dot_desire = 0;
            x_dot_desire = 0;
            break;
        }

    }
}

void MyRobot::update_last_Ts() {
    static bool pre_is_foot_touching = false;
    static int stance_start_ms = 0;
    if(pre_is_foot_touching == false && is_touching_ground == true)
    {
        stance_start_ms = system_ms;
        // now is the time to land
    }
    if(pre_is_foot_touching == true && is_touching_ground == false)
    {
        int stance_end_ms = system_ms;
        Ts = 0.001 * (double)(stance_end_ms - stance_start_ms);
    }
    pre_is_foot_touching = is_touching_ground;// change the statues // a little delay
}



void MyRobot::update_robot_state_machine() {

    switch(robot_state)
    {
        case LOADING:
        {
            if(joint_point.r < spring_normal_length * r_threshold)
                robot_state = COMPRESSION;
            break;
        }
        case COMPRESSION:
        {
            if(joint_point_dot.r > 0)
                robot_state = THRUST;
            break;
        }
        case THRUST:
        {
            if(joint_point.r > spring_normal_length * r_threshold)
                robot_state = UNLOADING;
            break;
        }
        case UNLOADING:
        {
            if(is_touching_ground == false)
                robot_state = FLIGHT;
            break;
        }
        case FLIGHT:
        {
            if(is_touching_ground == true)
                robot_state = LOADING;
            break;
        }
        default: break;
    }
}

