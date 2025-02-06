/*
    Author: Lorenzo Serafini
*/
#ifndef PICK_N_PLACE_H
#define PICK_N_PLACE_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "ir2425_group_07_assignment2/srvArm.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"
#include "utilities.h"
#include "const_dimension.h"
#include "ir2425_group_07_assignment2/collision_rem_service.h"


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;

class Pick_n_place
{
    private:
        ros::NodeHandle nh;


        /**
         * @brief cunction that rotate the pose of the apriltag in order tranform it into a feasible gripper pose,
         * with the z-axis pointing downwards
         * @param current_pose apriltag pose
         * @return pose with z-axis pointing downwards
         */
        geometry_msgs::PoseStamped align_z_axis_downwards(const geometry_msgs::PoseStamped& current_pose);

        //
        
        /**
         * @brief move the arm in the home configuration, above the robot head to avoid colliding against the walls during navigation
         * @return true if the home configuration was reached
         * @return false if the home configuration could not be reached
         */
        bool move_home();

        /**
         * @brief close the gripper in order to grasp the object
         * @return true if the gripper has been closed
         * @return false if the gripper could not be closed
         */
        bool close_gripper();

        /**
         * @brief open the gripper in order to release the object
         * @return true if the gripper has been opened
         * @return false if the gripper could not be opened
         */
        bool open_gripper();

        /**
         * @brief move the arm to the specified griiper pose
         * @param pose contains the gripper pose to be reached
         * @return true if the pose is reached
         * @return false if the pose could not be reached
         */
        bool move_arm(const geometry_msgs::PoseStamped& pose);


    public:
        Pick_n_place(ros::NodeHandle n);
        /**
         * @brief implements the whole pick/place routine, pick if req.pick_or_place = 1, place if req.pick_or_place = 2, move home if req.pick_or_place = 0
         * @param req the service request
         * @param res the service response
         * @return true if the routine has been performed
         * @return false if the routine could not be perfomermed
         */
        bool pickNplace(ir2425_group_07_assignment2::srvArm::Request &req, ir2425_group_07_assignment2::srvArm::Response &res);
};

#endif