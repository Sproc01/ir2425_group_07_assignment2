// Author: Michele Sprocatti, Lorenzo Serafini

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "tiago_iaslab_simulation/Coeffs.h"
#include "ir2425_group_07_assignment2/waypoint_service.h"
#include "ir2425_group_07_assignment2/collision_add_service.h"
#include "ir2425_group_07_assignment2/point_line.h"
#include "ir2425_group_07_assignment2/srvArm.h"
#include "const_dimension.h"
#include "utilities.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Node_A");
    ros::NodeHandle n;

    // constants
    const double HEAD_INCLINATION = -0.6;
    const double HEAD_VELOCITY = 0.1;
    const double TORSO_POS_DETECT = 0.15;
    const double TORSO_POS_HIGH = 0.35;
    const double TORSO_VELOCITY = 0.1;
    const double HEAD_INCLINATION_TAG_10 = HEAD_INCLINATION - 0.3;
    const bool INTELLIGENT_NAVIGATION = false; // change this to true if you want to use the intelligent navigation

    // variables
    bool move_camera, res_get_coefficients, move_torso_res, get_point_line_res;
    ros::Rate loop_rate(0.25);
    vector<float> coeffs_vec;
    vector<int> ids;
    vector<pair<int, geometry_msgs::PoseStamped>> available_ids;
    vector<geometry_msgs::PoseStamped> placing_map_frame; 
    vector<geometry_msgs::PoseStamped> positions;
    vector<pair<double,double>> placing_positions;
    vector<double> pos_head = {-0.7, 0, 0.7};
    geometry_msgs::PoseStamped pos_placing;
    geometry_msgs::PoseStamped pos_tag_10, pos_tag_10_map;
    double max_x_place = 0.9, max_y_place = 0.9;
    int index, selected_id, number_of_objects = 3; // number of objects to be picked and placed

    // variables for services
    ros::ServiceClient client_navigation = n.serviceClient<ir2425_group_07_assignment2::waypoint_service>("waypoint_service");
    ir2425_group_07_assignment2::waypoint_service srv_nav;

    ros::ServiceClient client_collision = n.serviceClient<ir2425_group_07_assignment2::collision_add_service>("collision_object_add");
    ir2425_group_07_assignment2::collision_add_service srv_collision;

    ros::ServiceClient client_arm = n.serviceClient<ir2425_group_07_assignment2::srvArm>("arm");
    ir2425_group_07_assignment2::srvArm srv_arm;

    // home position
    ROS_INFO("Request to reach home position");
    srv_arm.request.header.frame_id = "base_link";
    srv_arm.request.header.stamp = ros::Time::now();
    srv_arm.request.pick_or_place = 0;
    if(!client_arm.call(srv_arm))
    {
        ROS_ERROR("Error reaching position home");
        return 1;
    }
    ROS_INFO("Reached home position");

    // move the torso down
    move_torso_res = move_torso(TORSO_POS_DETECT, TORSO_VELOCITY);
    if(!move_torso_res)
    {
        ROS_ERROR("Error occur while trying to move the torso");
        return 1;
    }    
    ROS_INFO("Moving torso down");

    // move the camera down
    move_camera = move_head(0, HEAD_INCLINATION, 0, HEAD_VELOCITY);
    if(!move_camera)
    {
        ROS_ERROR("Error occur while trying to move the camera");
        return 1;
    }
    ROS_INFO("Moving camera down");


    // get the coefficients of the line
    res_get_coefficients = get_coefficients(n, coeffs_vec);
    if(!res_get_coefficients)
    {
        ROS_ERROR("Error occur while trying to get the coefficients");
        return 1;
    }
    ROS_INFO("Coefficients: m = %f, q = %f", coeffs_vec[0], coeffs_vec[1]);


    loop_rate.sleep();

    // request for the navigation
    if(INTELLIGENT_NAVIGATION)
    {
        ROS_INFO("Requested Intelligent Navigation");
        srv_nav.request.waypoint_num = 0;
        srv_nav.request.header.frame_id = "base_link";
        srv_nav.request.header.stamp = ros::Time::now();
        if (!client_navigation.call(srv_nav))
        {
            ROS_ERROR("Failed to call service waypoint_navigation");
            return 1;
        }
        ROS_INFO("Response navigation: %s", srv_nav.response.status.c_str());
    }


    // navigate around the pick table
    for(int i = 1; i < 4; i++)
    {
        loop_rate.sleep();
        ROS_INFO("Request to navigation");

        // request for the navigation
        srv_nav.request.waypoint_num = i;
        srv_nav.request.header.frame_id = "base_link";
        srv_nav.request.header.stamp = ros::Time::now();
        if (!client_navigation.call(srv_nav))
        {
            ROS_ERROR("Failed to call service waypoint_navigation");
            return 1;
        }
        ROS_INFO("Response navigation: %s", srv_nav.response.status.c_str());


        for(int j = 0; j < pos_head.size(); j++)
        {
            loop_rate.sleep();

            // move the camera to inspect the surroundings
            move_camera = move_head(pos_head[j], HEAD_INCLINATION, HEAD_VELOCITY, 0);
            if(!move_camera)
            {
                ROS_ERROR("Error occur while trying to move the camera");
                return 1;
            }
            ROS_INFO("Moving camera to inspect");

            loop_rate.sleep();

            // request for the service that add the collision objects
            srv_collision.request.rq = 1;
            srv_collision.request.header.stamp = ros::Time::now();
            srv_collision.request.header.frame_id = "base_link";
            if(!client_collision.call(srv_collision))
            {
                ROS_ERROR("Failed to call service collision");
                return 1;
            }
            ids = srv_collision.response.ids_added;
            positions = srv_collision.response.positions_objects_added;


            for(int j = 0; j < ids.size(); j++)
                available_ids.push_back(make_pair(ids[j], positions[j]));

            if(srv_collision.response.added)
            {
                ROS_INFO("Response collision -> ids added: ");
                for(int i = 0; i < ids.size(); i++)
                    ROS_INFO_STREAM("ID: " << to_string(ids[i]));
            }
        
        }  
        loop_rate.sleep();
    }

    // print the ids found and their positions
    for(int i = 0; i < available_ids.size(); i++)
    {
        ROS_INFO_STREAM("ID: " << to_string(available_ids[i].first) << " with position: "
            << "(X: " << available_ids[i].second.pose.position.x << ", " 
            << "Y: " << available_ids[i].second.pose.position.y << ", "
            << "Z: " << available_ids[i].second.pose.position.z << ")");           
    }
    
    // move the torso up
    move_torso_res = move_torso(TORSO_POS_HIGH, TORSO_VELOCITY);
    if(!move_torso_res)
    {
        ROS_ERROR("Error occur while trying to move the torso");
        return 1;
    }    
    ROS_INFO("Moving torso up");

    // select random id and the position in which it can be picked up and placed
    for(int i = 0; i < number_of_objects; i++)
    {

        // sample the id
        index = rand() % available_ids.size();
        selected_id = available_ids[index].first;
        ROS_INFO_STREAM("Selected id: " << selected_id);

        loop_rate.sleep();

        // determine the picking position
        double pick_x = available_ids[index].second.pose.position.x;
        double pick_y = available_ids[index].second.pose.position.y;
        
        if(pick_x < TABLE_PICK_X && pick_y < TABLE_PICK_Y)
            srv_nav.request.waypoint_num = 2;
        else if (pick_x < TABLE_PICK_X && pick_y > TABLE_PICK_Y)
            srv_nav.request.waypoint_num = 1;
        else
            srv_nav.request.waypoint_num = 3;

        ROS_INFO_STREAM("Going to waypoint " << srv_nav.request.waypoint_num << " to pick the object");
        
        // reach the picking position
        ROS_INFO("Call navigation");
        srv_nav.request.header.frame_id = "base_link";
        srv_nav.request.header.stamp = ros::Time::now();
        if (!client_navigation.call(srv_nav))
        {
            ROS_ERROR("Failed to call service waypoint_navigation");
            return 1;
        }
        ROS_INFO("Response navigation: %s", srv_nav.response.status.c_str());

        loop_rate.sleep();

        // pick the object
        srv_arm.request.object_id = get_object_id(selected_id);
        srv_arm.request.header.frame_id = "base_link";
        srv_arm.request.header.stamp = ros::Time::now();
        srv_arm.request.target_pose = available_ids[index].second;
        srv_arm.request.target_pose.header.frame_id = "map";
        srv_arm.request.target_pose.header.stamp = ros::Time::now();
        srv_arm.request.pick_or_place = 1;
        ROS_INFO("Picking..");
        if(!client_arm.call(srv_arm))
        {
            ROS_ERROR("Error during pick");
            return 1;
        }   
        ROS_INFO("Executed");
        remove(available_ids.begin(), available_ids.end(), available_ids[index]);
        
        loop_rate.sleep();        
        
        srv_nav.request.waypoint_num = 4;

        // reach the position to see tag 10
        ROS_INFO("Call navigation");
        srv_nav.request.header.frame_id = "base_link";
        srv_nav.request.header.stamp = ros::Time::now();
        if (!client_navigation.call(srv_nav))
        {
            ROS_ERROR("Failed to call service waypoint_navigation");
            return 1;
        }
        ROS_INFO("Response navigation: %s", srv_nav.response.status.c_str());

        loop_rate.sleep();

        // get the point on the line where to place
        if(placing_positions.size() == 0)
        {
            // move camera in order to see the tag 10
            move_camera = move_head(pos_head[1], HEAD_INCLINATION_TAG_10, HEAD_VELOCITY, HEAD_VELOCITY);
            if(!move_camera)
            {
                ROS_ERROR("Error occur while trying to move the camera");
                return 1;
            }
            ROS_INFO("Moving camera to see the tag 10");

            loop_rate.sleep();

            pos_tag_10.pose.position.x = 0;
            pos_tag_10.pose.position.y = 0;
            pos_tag_10.pose.position.z = 0;
            pos_tag_10.pose.orientation.w = 0;
            pos_tag_10.pose.orientation.x = 0;
            pos_tag_10.pose.orientation.y = 0;
            pos_tag_10.pose.orientation.z = 0;
            pos_tag_10_map = transformation("tag_10", "map", pos_tag_10);

            max_x_place = abs(TABLE_PLACE_Y - pos_tag_10_map.pose.position.y);
            max_y_place = abs(TABLE_PLACE_X - 0.05 - pos_tag_10_map.pose.position.x);

            ROS_INFO_STREAM("Max values: x: " << max_x_place << ", y: " << max_y_place);
            get_point_line_res = get_point_line(n, max_x_place, max_y_place, coeffs_vec[0], coeffs_vec[1], placing_positions);
            if(!get_point_line_res)
            {
                ROS_ERROR("Error occur while trying to get the point on the line");
                return 1;
            }
            
            for(int j = 0; j < number_of_objects; j++)
            {
                ROS_INFO_STREAM("Placing position in tag 10 frame: (" << placing_positions[j].first << ", " << placing_positions[j].second << ")");
                
                // computing the placing position in map frame
                pos_placing.pose.position.x = placing_positions[j].first;
                pos_placing.pose.position.y = placing_positions[j].second;
                pos_placing.pose.position.z = 0;
                pos_placing.pose.orientation.w = 1;
                pos_placing.pose.orientation.x = 0;
                pos_placing.pose.orientation.y = 0;
                pos_placing.pose.orientation.z = 0;
                placing_map_frame.push_back(transformation("tag_10", "map", pos_placing));
            }
        }
        
        ROS_INFO_STREAM("Placing position in map: (X: " << placing_map_frame[i].pose.position.x << 
                                    ", Y: " << placing_map_frame[i].pose.position.y << 
                                    ", Z: " << placing_map_frame[i].pose.position.z << ")");

        loop_rate.sleep();

        ROS_INFO("Placing...");

        // place the object
        placing_map_frame[i].pose.position.z = 0.85;
        srv_arm.request.object_id = get_object_id(selected_id);
        srv_arm.request.target_pose.pose.position = placing_map_frame[i].pose.position;
        srv_arm.request.target_pose.pose.orientation = placing_map_frame[i].pose.orientation;
        srv_arm.request.target_pose.header.frame_id = "map";
        srv_arm.request.target_pose.header.stamp = ros::Time::now();
        srv_arm.request.header.frame_id = "base_link";
        srv_arm.request.header.stamp = ros::Time::now();
        srv_arm.request.pick_or_place = 2;
        if(!client_arm.call(srv_arm))
        {
            ROS_ERROR("Error during place");
            return 1;
        }   
        ROS_INFO("Executed");

        srv_arm.request.pick_or_place = 0;
        if(!client_arm.call(srv_arm))
        {
            ROS_ERROR("Error while returning to home position");
            return 1;
        }   
        ROS_INFO("Returned to home position");
    }

    return 0;
}