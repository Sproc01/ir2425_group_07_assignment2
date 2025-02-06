//  Author: Michele Sprocatti

#include <ros/ros.h>
#include <ir2425_group_07_assignment2/point_line.h>

using namespace std;

bool get_point_line(ir2425_group_07_assignment2::point_line::Request &req, ir2425_group_07_assignment2::point_line::Response &res)
{
    ROS_INFO_STREAM("m and q received (m: " << req.m << ", q: "<< req.q << ")");
    res.header.frame_id = "base_link";
    double y = req.q;
    double x = 0;
    double dy = (req.y_max - req.q) / 2;
    ROS_INFO_STREAM("Increment y: " << dy);
    for(int i = 0; i < 3; i++)
    {
        x = (y - req.q)/ req.m;
        switch (i)
        {
            case 0:
                res.point1.push_back(x);
                res.point1.push_back(y);
                break;
            case 1:
                res.point2.push_back(x);
                res.point2.push_back(y);
                break;
            case 2:
                res.point3.push_back(x);
                res.point3.push_back(y);
                break;
        }
        y = y + dy;
    }
    ROS_INFO_STREAM("Point generated: (X: " << res.point1[0] << ", Y: "<< res.point1[1] << ")");
    ROS_INFO_STREAM("Point generated: (X: " << res.point2[0] << ", Y: "<< res.point2[1] << ")");
    ROS_INFO_STREAM("Point generated: (X: " << res.point3[0] << ", Y: "<< res.point3[1] << ")");
    res.header.stamp = ros::Time::now();
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_line_node");
    ROS_INFO_STREAM("Node line starts");
    ros::NodeHandle n;

    ROS_INFO("Ready to receive m and q");

    ros::ServiceServer service = n.advertiseService("point_line", get_point_line);

    ros::spin();
    return 0;
}  