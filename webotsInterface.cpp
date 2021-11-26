#include "webotsInterface.hpp"
#include "controller.hpp"
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
using namespace webots;
TouchSensor touch_sensor("touch sensor");
InertialUnit IMU("inertial unit");
Motor spring_motor("spring motor");
Motor Z_motor("ztational motor");
Motor X_motor("x rotational motor");
PositionSensor spring_position_sensor("position sensor");
PositionSensor X_motor_pos_sensor("x position sensor");
PositionSensor Z_motor_pos_sensor("z position sensor");
Keyboard keyboard;
void webots_device_init()
{
    spring_position_sensor.enable(TIME_STEP);
    X_motor_pos_sensor.enable(TIME_STEP);
    Z_motor_pos_sensor.enable(TIME_STEP);
    touch_sensor.enable(TIME_STEP);
    keyboard.enable(TIME_STEP);
    IMU.enable(TIME_STEP);
    // setting time gap to sensor
}

void set_spring_force(double force)
{
    spring_motor.setForce(-force);
}

void set_X_torque(double torque)
{
    X_motor.setTorque(torque);
}

void set_Z_torque(double torque)
{
    Z_motor.setTorque(torque);
}

double get_spring_length()
{
    double length = spring_position_sensor.getValue();
    return -length + 0.8;
}

double get_X_motor_angle()
{
    double angle = X_motor_pos_sensor.getValue();
    return angle*180.0f/PI;
}

double get_Z_motor_angle()
{
    double angle = Z_motor_pos_sensor.getValue();
    return angle*180.0f/PI;
}

bool is_foot_touching()
{
    return touch_sensor.getValue();
}

euler_angle_typedef get_IMU_value()
{
    const double * data = IMU.getRollPitchYaw();
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

