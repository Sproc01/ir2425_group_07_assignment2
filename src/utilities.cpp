/*
Author: Michele Sprocatti
*/
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "ir2425_group_07_assignment2/point_line.h"
#include "const_dimension.h"
#include "utilities.h"

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> act_control_client;

using namespace std;

bool move_head(double pos_joint_1, double pos_joint_2, double vel_joint_1, double vel_joint_2)
{
    act_control_client head_client("/head_controller/follow_joint_trajectory");
    head_client.waitForServer();
    control_msgs::FollowJointTrajectoryGoal goalJoint;
    goalJoint.trajectory.joint_names.push_back("head_1_joint");
    goalJoint.trajectory.joint_names.push_back("head_2_joint");
    goalJoint.trajectory.points.resize(1);

    goalJoint.trajectory.points[0].positions.resize(2);
    goalJoint.trajectory.points[0].velocities.resize(2);
    goalJoint.trajectory.points[0].positions[0] = pos_joint_1;
    goalJoint.trajectory.points[0].positions[1] = pos_joint_2;
    goalJoint.trajectory.points[0].velocities[0] = vel_joint_1;
    goalJoint.trajectory.points[0].velocities[1] = vel_joint_2;
    goalJoint.trajectory.points[0].time_from_start = ros::Duration(0.5);
    
    goalJoint.trajectory.header.stamp = ros::Time::now();
    goalJoint.trajectory.header.frame_id = "base_link";
    head_client.sendGoal(goalJoint);
    head_client.waitForResult();

    if(head_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        return false;

    return true;
}

bool get_coefficients(ros::NodeHandle n, vector<float>& coeffs_vec)
{
    ros::ServiceClient client = n.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");
    
    tiago_iaslab_simulation::Coeffs srv;
    srv.request.ready = true;

    if(ros::ok())
    {
        // request for the line coefficients
        srv.request.ready = true;
        if(!client.call(srv))
            return false;
        
        coeffs_vec.push_back(srv.response.coeffs[0]);
        coeffs_vec.push_back(srv.response.coeffs[1]);
        return true;      
    }
    return false;
}

bool move_torso(double pos_joint, double vel_joint)
{
    act_control_client torso_client("/torso_controller/follow_joint_trajectory");
    torso_client.waitForServer();
    control_msgs::FollowJointTrajectoryGoal goalJoint_torso;
    goalJoint_torso.trajectory.joint_names.push_back("torso_lift_joint"); 
    goalJoint_torso.trajectory.points.resize(1);

    goalJoint_torso.trajectory.points[0].positions.resize(1);
    goalJoint_torso.trajectory.points[0].velocities.resize(1);
    goalJoint_torso.trajectory.points[0].positions[0] = pos_joint;
    goalJoint_torso.trajectory.points[0].velocities[0] = vel_joint;
    goalJoint_torso.trajectory.points[0].time_from_start = ros::Duration(1);
    
    goalJoint_torso.trajectory.header.stamp = ros::Time::now();
    goalJoint_torso.trajectory.header.frame_id = "base_link";
    torso_client.sendGoal(goalJoint_torso);
    torso_client.waitForResult();

    if(torso_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        return false;

    return true;
}

bool get_point_line(ros::NodeHandle n, double x_max, double y_max, double m, double q, vector<pair<double, double>>& placing_positions)
{
    ros::ServiceClient client_point = n.serviceClient<ir2425_group_07_assignment2::point_line>("point_line");
    ir2425_group_07_assignment2::point_line srv_point;

    srv_point.request.header.frame_id = "base_link";
    srv_point.request.header.stamp = ros::Time::now();
    srv_point.request.m = m;
    srv_point.request.q = q;
    srv_point.request.y_max = x_max;
    srv_point.request.x_max = y_max;
    if(!client_point.call(srv_point))
        return false;

    
    placing_positions.push_back(make_pair(srv_point.response.point1[0], srv_point.response.point1[1]));
    placing_positions.push_back(make_pair(srv_point.response.point2[0], srv_point.response.point2[1]));
    placing_positions.push_back(make_pair(srv_point.response.point3[0], srv_point.response.point3[1]));
    return true;
}

geometry_msgs::PoseStamped transformation(string source_frame, string target_frame, geometry_msgs::PoseStamped pos_in)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::PoseStamped pos_out;

    while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)))
    {
        ROS_INFO("Waiting for transform");
        ros::Duration(0.5).sleep();
    } 
    geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    
    tf2::doTransform(pos_in, pos_out, transformed);
    
    ROS_INFO("Transform completed");

    return pos_out;
}

string get_object_id(int id)
{
    string res = "";
    switch (id)
    {
        case 1:
            res = ID_1;
            break;
        case 2:
            res = ID_2;
            break;
        case 3:
            res = ID_3;
            break;
        case 4:
            res = ID_4;
            break;
        case 5:
            res = ID_5;
            break;
        case 6:
            res = ID_6;
            break;
        case 7:
            res = ID_7;
            break;
        case 8:
            res = ID_8;
            break;
        case 9:
            res = ID_9;
            break;
    }
    return res;
}