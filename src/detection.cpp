/*
    Author:Michele Sprocatti
*/

#include <ros/ros.h>
#include "collision_detector.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_node");
    ROS_INFO_STREAM("NODE DETECTION STARTS");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0); 
    spinner.start();
    
    collision_detector srv;

    ROS_INFO("Starting detection");

    ros::Subscriber tag_subscriber = n.subscribe("tag_detections", 1,  &collision_detector::detectionCallback, &srv);

    ros::ServiceServer service_add = n.advertiseService("collision_object_add", &collision_detector::add, &srv);

    ros::ServiceServer service_rem = n.advertiseService("collision_object_rem", &collision_detector::remove, &srv);

    ros::waitForShutdown();
    return 0;
}  
