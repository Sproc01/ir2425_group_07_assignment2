// Author: Michele Sprocatti
#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "ros/ros.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "ir2425_group_07_assignment2/collision_add_service.h"
#include "ir2425_group_07_assignment2/collision_rem_service.h"


class collision_detector
{
    private:
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        std::vector<int> toFind;

        /**
         * @brief Compute the transformation from the tag frame corresponding to the id specified 
         * to the robot frame, the position transformed is the origin that corresponds to the tag position
         * @param id id of the tag of the frame to be transformed
         * @param pos_out result of the transformation
         */
        void transform_tagFrame(int id, geometry_msgs::PoseStamped& pos_out);

    public:
        collision_detector();

        /**
         * @brief function that implements the service collision_service. Add to the planning scene 
         * all the objects corresponding to the tag that the camera see now
         * @param req request of the service
         * @param res response of the service
         * @return true 
         * @return false 
         */
        bool add(ir2425_group_07_assignment2::collision_add_service::Request &req, ir2425_group_07_assignment2::collision_add_service::Response &res);
        

        /**
         * @brief function that implements the service collision_rem_service. Remove from the planning scene 
         * the object specified
         * @param req request of the service
         * @param res response of the service
         * @return true 
         * @return false 
         */
        bool remove(ir2425_group_07_assignment2::collision_rem_service::Request &req, ir2425_group_07_assignment2::collision_rem_service::Response &res);

        /**
         * @brief detection callback for the apriltag detection
         * @param msg message containing the detected tags
         */
        void detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

};
#endif // COLLISION_DETECTOR_H