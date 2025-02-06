/*
    Author: Lorenzo Serafini
*/
#include <ros/ros.h>
#include "ir2425_group_07_assignment2/srvArm.h"
#include "pick_n_place.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0); 
    spinner.start();

    Pick_n_place arm(nh);

    ros::ServiceServer service = nh.advertiseService("arm", &Pick_n_place::pickNplace, &arm);
    ROS_INFO("Ready to move arm");

    ros::waitForShutdown();
    return 0;
}