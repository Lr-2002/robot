#include <iostream>
#include <Eigen/Dense>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include "controller.hpp"
using namespace std;
using namespace Eigen;
int main(int argc, char ** argv)
{
    auto * robot = new MyRobot();


    while (robot->step(TIME_STEP) != -1) {
        robot -> run();
    }
    robot->run();

    delete robot;
    return 0;




////
////    wb_robot_init(); // todo to implementation
////    webots_device_init();
////    robot_init();
////
////    while (wb_robot_step(TIME_STEP) != -1)
////    {
////        update_robot_state();
////        robot_control();
////    }
////
////    robot_free();
////    wb_robot_cleanup();
//    return 0;


}
