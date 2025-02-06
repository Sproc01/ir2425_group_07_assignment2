#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "ir2425_group_07_assignment2/collision_add_service.h"
#include "ir2425_group_07_assignment2/collision_rem_service.h"
#include "collision_detector.h"
#include "const_dimension.h"
#include "utilities.h"

using namespace std;

collision_detector::collision_detector()
{
    toFind = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    vector<moveit_msgs::CollisionObject> tables;
    shape_msgs::SolidPrimitive primitive;
    moveit_msgs::CollisionObject table;
    geometry_msgs::Pose box_pose;

    // collision object table h: 0.9, l: 0.9
    // Table 1
    table.header.frame_id = "map";
    table.header.stamp = ros::Time::now();
    table.id = "table_pick";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.9;
    primitive.dimensions[primitive.BOX_Y] = 0.9;
    primitive.dimensions[primitive.BOX_Z] = 0.9;
    box_pose.orientation.w = 1;
    box_pose.position.x = TABLE_PICK_X;
    box_pose.position.y = TABLE_PICK_Y;
    box_pose.position.z = TABLE_Z;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(box_pose);
    table.operation = table.ADD;
    tables.push_back(table);

    table.primitives.clear();
    table.primitive_poses.clear();
    
    // Table 2
    table.header.frame_id = "map";
    table.header.stamp = ros::Time::now();
    table.id = "table_place";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = TABLE_SIDE;
    primitive.dimensions[primitive.BOX_Y] = TABLE_SIDE;
    primitive.dimensions[primitive.BOX_Z] = TABLE_SIDE;
    box_pose.orientation.w = 1;
    box_pose.position.x = TABLE_PLACE_X;
    box_pose.position.y = TABLE_PLACE_Y;
    box_pose.position.z = TABLE_Z;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(box_pose);
    table.operation = table.ADD;
    tables.push_back(table);

    ROS_INFO("Collision objects tables added");
    planning_scene_interface.applyCollisionObjects(tables);
}

bool collision_detector::remove(ir2425_group_07_assignment2::collision_rem_service::Request &req, ir2425_group_07_assignment2::collision_rem_service::Response &res)
{
    ROS_INFO("Removing collision object");
    planning_scene_interface.removeCollisionObjects({req.object_id});
    res.success = true;
    return true;
}

void collision_detector::transform_tagFrame(int id, geometry_msgs::PoseStamped& pos_out)
{
    string target_frame = "map";
    string source_frame = "tag_"+to_string(id);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::PoseStamped pos_in;

    int max_it = 0; // max 3 iterations if the tag is not seen in the camera no transform is available
    while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)) && max_it < 3)
    {
        ROS_INFO("Waiting for transform");
        ROS_INFO_STREAM("Frame " << source_frame);
        ros::Duration(0.5).sleep();
        max_it++;
    }

    if(max_it != 3)
    {
        geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        pos_in.header.frame_id = source_frame;
        pos_in.pose.position.x = 0;
        pos_in.pose.position.y = 0;
        pos_in.pose.position.z = 0;
        pos_in.pose.orientation.x = 0;
        pos_in.pose.orientation.y = 0;
        pos_in.pose.orientation.z = 0;
        pos_in.pose.orientation.w = 1;
        tf2::doTransform(pos_in, pos_out, transformed);
        ROS_INFO("Transform frame tag completed");
    }
    else
    {
        pos_out.header.frame_id = "-1"; // error code
    }
}

bool collision_detector::add(ir2425_group_07_assignment2::collision_add_service::Request &req, ir2425_group_07_assignment2::collision_add_service::Response &res)
{
    res.added = false;
    geometry_msgs::PoseStamped pos;
    moveit_msgs::CollisionObject collision_object;
    shape_msgs::SolidPrimitive primitive;
    geometry_msgs::Pose box_pose;

    for(int i = 0; i < toFind.size() - 1; i++)
    {
        collision_object.header.frame_id = "map";
        collision_object.header.stamp = ros::Time::now();
        if(toFind[i] != -1 && toFind[i] != 0) // if the tag is found
        {
            transform_tagFrame(toFind[i], pos);
            collision_object.primitives.clear();
            collision_object.primitive_poses.clear();
            if (pos.header.frame_id != "-1") // no trasform available so we cannot see the tag now
            {
                collision_object.id = get_object_id(toFind[i]);
                if(toFind[i] == 1 || toFind[i] == 2 || toFind[i] == 3)
                {
                    // hexagonal prism h: 0.1, r: 0.05 m -> cylinder
                    primitive.type = primitive.CYLINDER;
                    primitive.dimensions.resize(2);
                    primitive.dimensions[primitive.CYLINDER_HEIGHT] = HEXAGONAL_HEIGHT;
                    primitive.dimensions[primitive.CYLINDER_RADIUS] = HEXAGONAL_RADIUS;
                    // z based on trials
                    box_pose.position.z = 0.825;
                }
                else if(toFind[i] == 4 || toFind[i] == 5 || toFind[i] == 6)
                {
                    // cube l: 0.05 m -> BOX
                    primitive.type = primitive.BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[primitive.BOX_X] = CUBE_SIDE;
                    primitive.dimensions[primitive.BOX_Y] = CUBE_SIDE;
                    primitive.dimensions[primitive.BOX_Z] = CUBE_SIDE;
                    // z based on trials
                    box_pose.position.z = 0.8;
                }
                else if(toFind[i] == 7 || toFind[i] == 8 || toFind[i] == 9)
                {
                    // triangular prism  b: 0.07, h: 0.035, L: 0.05 m -> BOX
                    primitive.type = primitive.BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[primitive.BOX_X] = TRIANGULAR_BASE;
                    primitive.dimensions[primitive.BOX_Y] = TRIANGULAR_SIDE;
                    primitive.dimensions[primitive.BOX_Z] = TRIANGULAR_HEIGHT;
                    // z based on trials
                    box_pose.position.z = 0.8;
                }
                toFind[i] = 0;
                box_pose.orientation.x = pos.pose.orientation.x;
                box_pose.orientation.y = pos.pose.orientation.y;
                box_pose.orientation.z = pos.pose.orientation.z;
                box_pose.orientation.w = pos.pose.orientation.w;
                box_pose.position.x = pos.pose.position.x;
                box_pose.position.y = pos.pose.position.y;
                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(box_pose);
                collision_object.operation = collision_object.ADD;
                collision_objects.push_back(collision_object);
                res.ids_added.push_back(i+1);
                res.positions_objects_added.push_back(pos);
                res.added = true;
            }
        }
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);
    collision_objects.clear();
    ROS_INFO("Collision Object added to the planning scene");
    res.header.stamp = ros::Time::now();
    res.header.frame_id = "base_link";
    return true;
}


void collision_detector::detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
    if(msg->detections.size() > 0) // if no detection do nothing
    {
        string target_frame = "base_link";
        string source_frame = msg->header.frame_id;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)));
            ros::Duration(0.5).sleep();

        geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        
        //Transform available
        geometry_msgs::PoseStamped pos_in;
        geometry_msgs::PoseStamped pos_out;
        for (int i = 0; i < msg->detections.size(); ++i)
        {
            pos_in.header.frame_id = msg->detections.at(i).pose.header.frame_id;
            pos_in.pose.position.x = msg->detections.at(i).pose.pose.pose.position.x;
            pos_in.pose.position.y = msg->detections.at(i).pose.pose.pose.position.y;
            pos_in.pose.position.z = msg->detections.at(i).pose.pose.pose.position.z;
            pos_in.pose.orientation.x = msg->detections.at(i).pose.pose.pose.orientation.x;
            pos_in.pose.orientation.y = msg->detections.at(i).pose.pose.pose.orientation.y;
            pos_in.pose.orientation.z = msg->detections.at(i).pose.pose.pose.orientation.z;
            pos_in.pose.orientation.w = msg->detections.at(i).pose.pose.pose.orientation.w;

            tf2::doTransform(pos_in, pos_out, transformed);

            if(toFind[msg->detections.at(i).id[0] - 1] == -1)
            {
                ROS_INFO_STREAM("Found apriltag with id: " << to_string(msg->detections.at(i).id[0]) 
                << " with position: "
                << "( X: " << pos_out.pose.position.x << ", " 
                << "Y: " << pos_out.pose.position.y << ", "
                << "Z: " << pos_out.pose.position.z << " )");
                toFind[msg->detections.at(i).id[0] - 1] = msg->detections.at(i).id[0];
            }
        }
    }
}