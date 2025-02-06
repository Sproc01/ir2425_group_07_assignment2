/*
Author: Michele Sprocatti
*/

#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "tiago_iaslab_simulation/Coeffs.h"
#include "ir2425_group_07_assignment2/point_line.h"
#include "const_dimension.h"

/**
 * @brief function that moves the joints of the head to the specified positions 
 * with the given velocities, return true if the movement is completed correctly
 * @param pos_joint_1 position of the first joint of the head
 * @param pos_joint_2 position of the second joint of the head
 * @param vel_joint_1 velocity of the movement of the first joint of the head
 * @param vel_joint_2 velocity of the movement of the second joint of the head
 * @return true if the movement is completed correctly
 * @return false if the movement is not completed correctly
 */
bool move_head(double pos_joint_1, double pos_joint_2, double vel_joint_1, double vel_joint_2);

/**
 * @brief Get the coefficients from the service (/straight_line_srv) that provides the coefficients of the line
 * @param n node handle
 * @param coeffs_vec output vector containing the two coefficients of the line
 * @return true if the request is completed correctly
 * @return false if the request is not completed correctly
 */
bool get_coefficients(ros::NodeHandle n, std::vector<float>& coeffs_vec);

/**
 * @brief function that moves the joint of the torso to the specified position 
 * with the given velocity, return true if the movement is completed correctly
 * @param pos_joint position of the joint of the torso
 * @param vel_joint velocity of the movement of the joint of the head
 * @return true if the movement is completed correctly
 * @return false if the movement is not completed correctly
 */
bool move_torso(double pos_joint, double vel_joint);

/**
 * @brief Get a point on the specified line 
 * @param n node handle
 * @param x_max max x value that the point can have
 * @param y_max max y value that the point can have
 * @param m parameter m of the line
 * @param q parameter q of the line
 * @param placing_positions placing positions
 * @return true if request completed correctly
 * @return false if request not completed correctly
 */
bool get_point_line(ros::NodeHandle n, double x_max, double y_max, double m, double q, std::vector<std::pair<double, double>>& placing_positions);


/**
 * @brief compute the transformation from the source frame to the target frame for the specified position
 * @param source_frame source frame of the transformation
 * @param target_frame target frame of the transformation
 * @param pos_in position to be transformed
 * @return geometry_msgs::PoseStamped the transformed position if the transformation is done successfully
 */
geometry_msgs::PoseStamped transformation(std::string source_frame, std::string target_frame, geometry_msgs::PoseStamped pos_in);

/**
 * @brief return the object id for the object of the specified id
 * @param id of the object
 * @return string corresponding to object id
 */
std::string get_object_id(int id);

#endif