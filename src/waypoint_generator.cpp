/*
    Author: Zlatko Kovachev
*/

// ####################### INCLUDES ####################### 

#include "ros/ros.h"
#include "ir2425_group_07_assignment2/waypoint_service.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <random>
#include <cmath>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "waypoint_generator.h"


// ####################### WAIPOINT GENERATOR'S CONSTRUCTOR ####################### 

waypoint_generator::waypoint_generator()
{
        // Waypoints
    waypoints_pos_0 = {
        {7.00, -0.5, -M_PI/2},
        {7.50, -1.0, -M_PI/2},
        {8.75, -0.5, -2*M_PI/3},
        {8.50,  0.5, -2*M_PI/3},
        {7.00,  0.5, -M_PI/2}
    };

    waypoints_tables = {
        {7.00, -2.7, 0.0},
        {7.85, -3.9, M_PI/2},
        {8.65, -3.0, M_PI},
        {8.65, -2.0, M_PI},
        {7.05, -1.9, 0.0}
    };


    waypoints_corners = {
        {7.0, -3.9},
        {8.7, -3.9},
        {8.7, -1.0},
        {7.0, -1.0}
    };

        // Subscriptions
    ac = new MoveBaseClient("move_base", true);
        //wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))){ROS_INFO("Waiting for the move_base action server to come up");}
    sub_started = false;

        // Path planning
    last_waypoint_used = -1;
    rand_gen();

        // Costs
    DIST_TABLES = 1.1;
    DIST_MAX = 0.25;
    Y_ERR = 0.2;
    X_OFFSET = 0.8;
    Y_OFFSET = 0.85;
    OFFSET_CORRECTION_PLACING = 0.2;
    OFFSET_CORRECTION_PICKING = 0.3;
}

waypoint_generator::waypoint_generator(ros::NodeHandle& nh) : waypoint_generator()
{
    start_scan_sub(nh);
    sub_started = true;
}


// ####################### WAIPOINT GENERATOR'S DESTRUCTOR ####################### 

waypoint_generator::~waypoint_generator(void) {};


// ####################### WAIPOINT GENERATOR'S HELPFULL FUNCTIONS #######################

void waypoint_generator::start_scan_sub(ros::NodeHandle& nh)
{
    if(!sub_started)
        scan_subscriber = nh.subscribe("scan", 1000, &waypoint_generator::callbackLaserScan, this);
}

void waypoint_generator::rand_gen()
{
    std::random_device rd;
    std::mt19937 gen(rd()); 
    std::uniform_int_distribution<> dist(0, 4);
    
    rand_int = dist(gen);
}

void waypoint_generator::callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg.header = msg->header;

    scan_msg.angle_min = msg->angle_min;
    scan_msg.angle_max = msg->angle_max;
    scan_msg.angle_increment = msg->angle_increment;

    scan_msg.ranges = msg->ranges;
}

geometry_msgs::PoseStamped waypoint_generator::tf_source_to_target( geometry_msgs::PoseStamped &pos_in, std::string source_frame, std::string target_frame )
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)))
        ros::Duration(0.5).sleep();
    geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::PoseStamped pos_out;
    tf2::doTransform(pos_in, pos_out, transformed);

    return pos_out;
}


// ####################### WAIPOINT GENERATOR'S PATH PLANNING FUNCTIONS #######################

void waypoint_generator::generate_waypoints()
{
    ROS_INFO("\n=------------------------------------=\n\n\t-> Start generate waypoint procedure\n");

//--------------------------------------------------------------------- Get last 'scan' message

    sensor_msgs::LaserScan::ConstPtr latest_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", ros::Duration(5));
    scan_subscriber.shutdown();
    scan_msg = *latest_scan;

//--------------------------------------------------------------------- Create clusters

    std::vector<std::vector<std::array<float, 2>>> clusters;
    std::vector<std::array<float, 2>> group;
    group.push_back({scan_msg.ranges[0], scan_msg.angle_min});

    for(short i = 1; i < scan_msg.ranges.size(); i++)
    {
        float scan = scan_msg.ranges[i];
        float theta = scan_msg.angle_min + i * scan_msg.angle_increment;

        if(abs(scan - group.back()[0]) <= DIST_MAX)
            group.push_back({scan, theta});
        
        else
        {
            clusters.push_back(group);
            group.clear();
            group.push_back({scan, theta});
        }
    }
    
    ROS_INFO("-> Number of clusters: %li", clusters.size());

//--------------------------------------------------------------------- Remove big and small clusters

    std::vector<std::vector<std::array<float, 2>>> legs;
    for(int i = 0; i < clusters.size(); i++)
    {
        ROS_INFO("\t->Dimention of cluster #%i: %li", i, clusters[i].size());
        if(clusters[i].size() >= 2 && clusters[i].size() <= 15)
            legs.push_back(clusters[i]);
    }

//--------------------------------------------------------------------- Evaluate the clusters positions
    
    ROS_INFO("\n");

    std::vector<std::array<float, 4>> tables;
    for(int i = 0; i < legs.size(); i++)
    {
        std::array<float, 2> first = legs[i].front();
        std::array<float, 2> last = legs[i].back();

        float mid_dist = (first[0] + last[0]) / 2;
        float mid_theta = (first[1] + last[1]) / 2;

        float x = mid_dist * cos(mid_theta);
        float y = mid_dist * sin(mid_theta);
        
        if(mid_theta > -2*M_PI/6 && mid_theta < 2*M_PI/6)
        {
            tables.push_back({mid_dist, mid_theta, x, y});
            ROS_INFO("-> Leg number #%i -> x: %.4f, y: %.4f, DIST: %.4f, THETA: %.4f", i + 1, x, y, mid_dist, mid_theta);
        }
    }

//--------------------------------------------------------------------- Selecting the two tables

    float min_dist_diff = 5.0;
    short index_1 = 0;
    short index_2 = 1;

    if(tables.size() > 2)
    {
        for(short i = 0; i < tables.size() - 1; i++)
        {
            for(short j = i + 1; j < tables.size(); j++)
            {   
                float x = tables[j][2] - tables[i][2];
                float y = tables[j][3] - tables[i][3];
                float dist = sqrt(std::pow(x, 2) + std::pow(y, 2));
                float dist_diff = abs(dist - DIST_TABLES);

                    // Want the distances to be similar to DIST_TABLES
                if(dist_diff < min_dist_diff)
                {
                    min_dist_diff = dist_diff;
                    index_1 = i;
                    index_2 = j;
                }
            }
        }
    }

    if(tables[index_1][0] < tables[index_2][0])
    {
        short temp = index_1;
        index_1 = index_2;
        index_2 = temp;
    }

//--------------------------------------------------------------------- Tf from "base_link" to "map"

    geometry_msgs::PoseStamped map_table_pick;
    geometry_msgs::PoseStamped map_table_place;
    geometry_msgs::PoseStamped robot_table_pick;
    geometry_msgs::PoseStamped robot_table_place;

    robot_table_pick.pose.position.x = tables[index_1][2];
    robot_table_pick.pose.position.y = tables[index_1][3];
    robot_table_pick.pose.position.z = 0;

    robot_table_place.pose.position.x = tables[index_2][2];
    robot_table_place.pose.position.y = tables[index_2][3];
    robot_table_place.pose.position.z = 0;

    map_table_pick = tf_source_to_target(robot_table_pick, "base_link", "map");
    map_table_place = tf_source_to_target(robot_table_place, "base_link", "map");

    map_table_pick.pose.position.y -= Y_ERR;
    map_table_place.pose.position.y -= Y_ERR;

    ROS_INFO("Pick table -> x: %.4f, y: %.4f", map_table_pick.pose.position.x, map_table_pick.pose.position.y);
    ROS_INFO("Place table -> x: %.4f, y: %.4f\n", map_table_place.pose.position.x, map_table_place.pose.position.y);

//--------------------------------------------------------------------- Evaluate the new waypoints

    waypoints_tables = {
        {(float)map_table_pick.pose.position.x - X_OFFSET,     (float)map_table_pick.pose.position.y + OFFSET_CORRECTION_PICKING,             0.0},
        {(float)map_table_pick.pose.position.x,                (float)map_table_pick.pose.position.y - Y_OFFSET,  M_PI/2},
        {(float)map_table_pick.pose.position.x + X_OFFSET,     (float)map_table_pick.pose.position.y,             M_PI},
        {(float)map_table_place.pose.position.x + X_OFFSET,    (float)map_table_place.pose.position.y - OFFSET_CORRECTION_PLACING,            M_PI},
        {(float)map_table_place.pose.position.x - X_OFFSET,    (float)map_table_place.pose.position.y,            0.0}
    };

    for(int i = 0; i < waypoints_tables.size(); i++)
        ROS_INFO("Waypoint #%i -> x: %.4f, y: %.4f", i, waypoints_tables[i][0], waypoints_tables[i][1]);

    ROS_INFO("\n");

    waypoints_corners = {
        {waypoints_tables[0][0], waypoints_tables[1][1]},
        {waypoints_tables[2][0], waypoints_tables[1][1]},
        {waypoints_tables[3][0], waypoints_tables[3][1] + Y_OFFSET},
        {waypoints_tables[4][0], waypoints_tables[4][1] + Y_OFFSET}
    };

    for(int i = 0; i < waypoints_corners.size(); i++)
        ROS_INFO("Corner #%i -> x: %.4f, y: %.4f", i, waypoints_corners[i][0], waypoints_corners[i][1]);
}

void waypoint_generator::start_path_none(const int new_waypoint)
{
    path.push_back(waypoints_pos_0[rand_int]);
}

void waypoint_generator::start_path_1(const int new_waypoint)
{
    if(new_waypoint == 2 || new_waypoint == 3 || new_waypoint == 4)
    {
        path.push_back({waypoints_tables[0][0], waypoints_tables[0][1], 3*M_PI/2});
        path.push_back({waypoints_corners[0][0], waypoints_corners[0][1], waypoints_tables[0][2]});
    }
    
    if(new_waypoint == 3 || new_waypoint == 4) 
        path.push_back({waypoints_corners[1][0], waypoints_corners[1][1], waypoints_tables[1][2]});

    if(new_waypoint == 5)
        path.push_back({waypoints_tables[0][0], waypoints_tables[0][1], waypoints_tables[1][2]});

    path.push_back(waypoints_tables[new_waypoint - 1]);

}

void waypoint_generator::start_path_2(const int new_waypoint)
{
    if(new_waypoint == 1 || new_waypoint == 5)
    {
        path.push_back({waypoints_tables[1][0], waypoints_tables[1][1], waypoints_tables[2][2]});
        path.push_back({waypoints_corners[0][0], waypoints_corners[0][1], waypoints_tables[1][2]});
    }
    
    if(new_waypoint == 3 || new_waypoint == 4)
    {
        path.push_back({waypoints_tables[1][0], waypoints_tables[1][1], waypoints_tables[0][2]});
        path.push_back({waypoints_corners[1][0], waypoints_corners[1][1], waypoints_tables[1][2]});
    }
    
    path.push_back(waypoints_tables[new_waypoint - 1]);

}

void waypoint_generator::start_path_3(const int new_waypoint)
{
    if(new_waypoint == 1 || new_waypoint == 2 || new_waypoint == 5)
        path.push_back({waypoints_corners[1][0], waypoints_corners[1][1], waypoints_tables[2][2]});
    
    if(new_waypoint == 1 || new_waypoint == 5)
        path.push_back({waypoints_corners[0][0], waypoints_corners[0][1], waypoints_tables[1][2]});
    
    path.push_back(waypoints_tables[new_waypoint - 1]);

}

void waypoint_generator::start_path_4(const int new_waypoint)
{
    if(new_waypoint == 1 || new_waypoint == 5)
    {
        path.push_back({waypoints_corners[2][0], waypoints_corners[2][1], waypoints_tables[3][2]});
        path.push_back({waypoints_corners[3][0], waypoints_corners[3][1], 3*M_PI/2});
    }
    
    if(new_waypoint == 2)
    {
        path.push_back({waypoints_tables[3][0], waypoints_tables[3][1], 3*M_PI/2});
        path.push_back({waypoints_corners[1][0], waypoints_corners[1][1], waypoints_tables[2][2]});
    }
    
    path.push_back(waypoints_tables[new_waypoint - 1]);

}

void waypoint_generator::start_path_5(const int new_waypoint)
{
    if(new_waypoint == 3 || new_waypoint == 4)
    {
        path.push_back({waypoints_tables[4][0], waypoints_tables[4][1], M_PI/2});
        path.push_back({waypoints_corners[3][0], waypoints_corners[3][1], waypoints_tables[4][2]});
        path.push_back({waypoints_corners[2][0], waypoints_corners[2][1], 3*M_PI/2});
    }
    
    if(new_waypoint == 2)
    {
        path.push_back({waypoints_tables[4][0], waypoints_tables[4][1], 3*M_PI/2});
        path.push_back({waypoints_corners[0][0], waypoints_corners[0][1], waypoints_tables[0][2]});
    }
    
    path.push_back(waypoints_tables[new_waypoint - 1]);

}

void waypoint_generator::calculate_path(const int past_waypoint, const int new_waypoint)
{
    path.clear();
    
    if(new_waypoint == 0)       start_path_none(new_waypoint);
    else if(past_waypoint == 1) start_path_1(new_waypoint);
    else if(past_waypoint == 2) start_path_2(new_waypoint);
    else if(past_waypoint == 3) start_path_3(new_waypoint);
    else if(past_waypoint == 4) start_path_4(new_waypoint);
    else if(past_waypoint == 5) start_path_5(new_waypoint);
    else                        path.push_back({waypoints_tables[new_waypoint - 1]});
}


// ####################### WAIPOINT GENERATOR'S MOVE FUNCTIONS #######################

void waypoint_generator::move_tiago(std::array<float, 3> waypoint)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoint[0];
    goal.target_pose.pose.position.y = waypoint[1];
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint[2]);

    ac->sendGoal(goal);
    ac->waitForResult();
    
    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("-> Tiago moved to the correct waypoint (%.2f, %.2f)", waypoint[0], waypoint[1]);
    else
        ROS_INFO("\n\t!!! TIAGO FAILED TO MOVE !!!");
}

bool waypoint_generator::move(ir2425_group_07_assignment2::waypoint_service::Request &req,ir2425_group_07_assignment2::waypoint_service::Response &res)
{
    res.header.frame_id = "map";
    res.header.stamp = ros::Time::now();

    int req_waypoint = req.waypoint_num;
    ROS_INFO("\n=------------------------------------=\n\n\t-> Move Tiago from waypoint #%i to waypoint #%i\n", last_waypoint_used, req_waypoint);

    if(req_waypoint == 0 && last_waypoint_used != -1)
    {
        ROS_INFO("\n\t-> Tiago could not go to waypoint 0 if it is not the first movement selected.\n");
        res.status = "NOT MOVED";
        return true;
    }
    
        // Calculate path
    calculate_path(last_waypoint_used, req_waypoint);
    if(req_waypoint != 0)
        last_waypoint_used = req_waypoint;

        // Move Tiago to each waypoint
    std::array<float, 3> selected_waypoint;
    for(short i = 0; i < path.size(); i++)
    {
        selected_waypoint = path[i];
        move_tiago(selected_waypoint);
        ros::Duration(0.1).sleep();
    }

        // Generation of waypoint if first motion is to waypoint 0
    if(req_waypoint == 0 && last_waypoint_used == -1)
    {
        generate_waypoints();
        last_waypoint_used = req_waypoint;
        move_tiago({waypoints_corners[3][0], waypoints_corners[3][1], -M_PI/2});
    }

    ROS_INFO("\t-> Tiago moved successfully to the final desination");
    res.status = "MOVED";

    return true;
}

