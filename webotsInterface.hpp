#ifndef _WEBOTSINTERFACE_HPP_
#define _WEBOTSINTERFACE_HPP_

#define PI (3.1415926)
#define TIME_STEP 2

typedef struct
{
    double yaw;
    double pitch;
    double roll;
}euler_angle_typedef;

//-----------------------------------------------------------extern
extern void              webots_device_init               ();
extern void              set_spring_force     (double force);
extern void              set_X_torque        (double torque);
extern void              set_Z_torque        (double torque);
extern double            get_spring_length                ();
extern double            get_X_motor_angle                ();
extern double            get_Z_motor_angle                ();
extern bool              is_foot_touching                 ();
extern int               get_keyboard                     ();
extern euler_angle_typedef get_IMU_value                    ();
#endif
