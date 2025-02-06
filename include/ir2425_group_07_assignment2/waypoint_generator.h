/*
    Author: Zlatko Kovachev
*/

#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H


// ####################### INCLUDES ####################### 

#include "ros/ros.h"
#include "ir2425_group_07_assignment2/waypoint_service.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// ######################################################## 


/**
 * @brief Class that stores all the variables and functions to make Tiago navigate using waypoints. 
 */
class waypoint_generator
{
    private:
    // ####################### VARIABLES ####################### 
        
            // Waypoints
        std::vector<std::array<float, 3>> waypoints_pos_0;
        std::vector<std::array<float, 3>> waypoints_tables;
        std::vector<std::array<float, 2>> waypoints_corners;

            // Subscriptions
        MoveBaseClient* ac;
        ros::Subscriber scan_subscriber;
        sensor_msgs::LaserScan scan_msg;
        bool sub_started;

            // Path planning
        std::vector<std::array<float, 3>> path;
        int last_waypoint_used;
        int rand_int;

            // Costs
        float DIST_MAX;
        float DIST_TABLES;
        float Y_ERR;
        float X_OFFSET;
        float Y_OFFSET;
        float OFFSET_CORRECTION_PLACING;
        float OFFSET_CORRECTION_PICKING;

    public:

    // ####################### CONSTRUCTORS & DESTRUCTOR ####################### 

        waypoint_generator();

        /**
         * @brief Constructor that needs a NodeHandle to start the 'scan' subscription
         * 
         * @param nh NodeHandle of the node that created the instance of this class
         */
        waypoint_generator(ros::NodeHandle& nh);

        ~waypoint_generator(void);


    // ####################### HELPFULL FUNCTIONS #######################

        /**
         * @brief Given a NodeHandle starts a subscription to the topic 'scan'
         * 
         * @param nh NodeHandle of the node that created the instance of this class
         */
        void start_scan_sub(ros::NodeHandle& nh);

        /**
         * @brief Generates a random integer between 0 and 4 and save the value into rand_int
         */
        void rand_gen();

        /**
         * @brief Callback function for the 'scan' subscription. Saves the topic's message in scan_msg
         * 
         * @param msg Scan message 
         */
        void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);

        /**
         * @brief Evaluates the transfromation between two reference frames
         * 
         * @param pos_in The pose we want to transform 
         * @param source_frame The name of the source reference frame
         * @param target_frame The name of the target reference frame
         * 
         * @return geometry_msgs::PoseStamped - The pose transformed in target_frame
         */
        geometry_msgs::PoseStamped tf_source_to_target( geometry_msgs::PoseStamped &pos_in, std::string source_frame, std::string target_frame );


    // ####################### PATH PLANNING FUNCTIONS #######################

        /**
         * @brief Given the 'scan' message at waipoint 0, evaluates the waipoints of waypoints_tables and
         * waypoints_corners, using such informations
         */
        void generate_waypoints();

        /**
         * @brief Sets the waypoints for the path that Tiago needs to follow to reach the waypoint 0
         * 
         * @param new_waypoint Value of the waypoint that we want to reach
         */
        void start_path_none(const int new_waypoint);
        
        /**
         * @brief Sets the waypoints for the path that Tiago needs to follow starting from waypoin 1
         * 
         * @param new_waypoint Value of the waypoint that we want to reach
         */
        void start_path_1(const int new_waypoint);

        /**
         * @brief Sets the waypoints for the path that Tiago needs to follow starting from waypoin 2
         * 
         * @param new_waypoint Value of the waypoint that we want to reach
         */
        void start_path_2(const int new_waypoint);

        /**
         * @brief Sets the waypoints for the path that Tiago needs to follow starting from waypoin 3
         * 
         * @param new_waypoint Value of the waypoint that we want to reach
         */
        void start_path_3(const int new_waypoint);

        /**
         * @brief Sets the waypoints for the path that Tiago needs to follow starting from waypoin 4
         * 
         * @param new_waypoint Value of the waypoint that we want to reach
         */
        void start_path_4(const int new_waypoint);

        /**
         * @brief Sets the waypoints for the path that Tiago needs to follow starting from waypoin 5
         * 
         * @param new_waypoint Value of the waypoint that we want to reach
         */
        void start_path_5(const int new_waypoint);

        /**
         * @brief Main functions that manage all the Tiago path's  
         * 
         * @param past_waypoint Value of the waypoint that Tiago is currently
         * @param new_waypoint Value of the waypoint that we want to reach
         */
        void calculate_path(const int past_waypoint, const int new_waypoint);


    // ####################### MOVE FUNCTIONS #######################

        /**
         * @brief Makes Tiago move to a waypoint using MoveBaseClient
         * 
         * @param waypoint Pose in 'map' frame where we want to move Tiago 
         */
        void move_tiago(std::array<float, 3> waypoint);

        /**
         * @brief Main function used to create the waypoint service
         */
        bool move(ir2425_group_07_assignment2::waypoint_service::Request &req,ir2425_group_07_assignment2::waypoint_service::Response &res);
};

#endif