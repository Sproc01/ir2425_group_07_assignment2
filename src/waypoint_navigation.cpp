/*
    Author: Zlatko Kovachev

        -> WAYPOINT LEGEND


    [B]           [2]           [A]

            ---------------
            |             |
            |     PICK    |
    [3]     |             |     [1]
            |    TABLE    |
            |             |
            ---------------

            ---------------
            |             |
            |    PLACE    |
    [4]     |             |     [5]
            |    TABLE    |
            |             |
            ---------------

    [C]                         [D]


                  [0]

    It is possible to move only in the numbered waypoints {0,1,2,3,4,5}.

*/

// ####################### INCLUDES ####################### 

#include "ros/ros.h"
#include "ir2425_group_07_assignment2/waypoint_service.h"
#include "waypoint_generator.h"


// ####################### MAIN #######################

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_server");
    ros::NodeHandle n;
    
    waypoint_generator waypoint_node;
    waypoint_node.start_scan_sub(n);
    
        // Creates the service using the instance waypoint_node 
    ros::ServiceServer server = n.advertiseService(
        "waypoint_service", 
        &waypoint_generator::move, 
        &waypoint_node
    );

    ROS_INFO("\n\t--> Navigation server ready for motion.\n");

    ros::spin();
    return 0;
}