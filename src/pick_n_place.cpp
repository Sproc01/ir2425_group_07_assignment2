/*
    Author: Lorenzo Serafini
*/
#include "pick_n_place.h"

Pick_n_place::Pick_n_place(ros::NodeHandle n)
{  nh = n; }

geometry_msgs::PoseStamped Pick_n_place::align_z_axis_downwards(const geometry_msgs::PoseStamped& current_pose)
{
    geometry_msgs::PoseStamped final_pose;
    final_pose.header.frame_id = current_pose.header.frame_id;
    final_pose.pose.position = current_pose.pose.position;      //we maintain the same position, we only change the orientation

    tf2::Quaternion current_orientation;
    tf2::fromMsg(current_pose.pose.orientation, current_orientation);
    

    tf2::Matrix3x3 rotation_matrix(current_orientation);
    tf2::Vector3 current_z_axis = rotation_matrix.getColumn(2);

    tf2::Vector3 target_z_axis(0, 0, -1);

    tf2::Quaternion alignment_rotation;
    tf2::Quaternion updated_orientation;

    if(current_z_axis == target_z_axis)
        return current_pose;
    else if(current_z_axis == tf2::Vector3(0,0,1))      //otherwise (0,0,1) x (0,0,-1) = (0,0,0)
    {
        alignment_rotation.setRPY(0, M_PI, 0); 
        alignment_rotation.normalize();
        updated_orientation = current_orientation * alignment_rotation;
    }
    else
    {
        tf2::Vector3 rotation_axis = current_z_axis.cross(target_z_axis);
        double rotation_angle = current_z_axis.angle(target_z_axis);
        alignment_rotation.setRotation(rotation_axis, rotation_angle);
        alignment_rotation.normalize();

        updated_orientation = alignment_rotation * current_orientation;
        updated_orientation.normalize();
        
        tf2::Quaternion rotation_quaternion;    //useful for the triangular shape object, to grasp it along its vertical sides
        rotation_quaternion.setRPY(0, 0, M_PI/2);
        updated_orientation = updated_orientation * rotation_quaternion;

    }
    
    updated_orientation.normalize();

    final_pose.pose.orientation = tf2::toMsg(updated_orientation);

    return final_pose;

}

bool Pick_n_place::move_home()
{
    moveit::planning_interface::MoveGroupInterface arm("arm_torso");
    std::vector<double> target_joint_values = 
    {
        TORSO_HOME_VALUE,   // Joint torso value (torso_lift_joint)
        0.07,   // Joint 1 value
        1.02,   // Joint 2 value
        -3.46,  // Joint 3 value
        0.75,   // Joint 4 value
        -1.87,  // Joint 5 value
        1.39,   // Joint 6 value
        0.0     // Joint 7 value
    };

    arm.setJointValueTarget(target_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto success = arm.plan(my_plan);

    if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Home reached");
        arm.execute(my_plan);
        return true;
    }
    else
    {
        ROS_ERROR("Failed home planning");
        return false;
    }

}

bool Pick_n_place::close_gripper()
{
    arm_control_client gripper_control_client("/gripper_controller/follow_joint_trajectory", true);
    gripper_control_client.waitForServer();
    
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].effort.resize(2);
    goal.trajectory.points[0].positions[0] = 0.01; 
    goal.trajectory.points[0].positions[1] = 0.01;
    goal.trajectory.points[0].effort[0] = 60.0;
    goal.trajectory.points[0].effort[1] = 60.0;

    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

    gripper_control_client.sendGoal(goal);
    gripper_control_client.waitForResult();

    if(gripper_control_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Gripper closed");
        return true;
    }
    else
    {
        ROS_INFO("Failed to close gripper");
        return false;
    }
}

bool Pick_n_place::open_gripper()
{
    arm_control_client gripper_control_client("/gripper_controller/follow_joint_trajectory", true);
    gripper_control_client.waitForServer();
    
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].velocities.resize(2);
    goal.trajectory.points[0].positions[0] = 0.04; 
    goal.trajectory.points[0].positions[1] = 0.04;
    goal.trajectory.points[0].velocities[0] = 0;
    goal.trajectory.points[0].velocities[1] = 0;
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

    gripper_control_client.sendGoal(goal);
    gripper_control_client.waitForResult();

    bool finished_before_timeout = gripper_control_client.waitForResult(ros::Duration(5.0));

    if(gripper_control_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Gripper opened");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to open gripper");
        return false;
    }
}

bool Pick_n_place::move_arm(const geometry_msgs::PoseStamped& pose)
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    arm.setPlanningTime(120.0);
    arm.setNumPlanningAttempts(60);
    arm.setEndEffectorLink("gripper_grasping_frame_Z");
    arm.setGoalOrientationTolerance(M_PI/18);
    
    arm.setPoseReferenceFrame(pose.header.frame_id);
    arm.setPoseTarget(pose.pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto success = arm.move();

    if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Planning succeeded, executing...");
        return true;
    }
    else
    {
        ROS_INFO("Failed planning: %i",success.val);
        return false;
    }
}

bool Pick_n_place::pickNplace(ir2425_group_07_assignment2::srvArm::Request &req, ir2425_group_07_assignment2::srvArm::Response &res)
{
    if(req.pick_or_place == 0)  //moving to home configuration
        return move_home();

    geometry_msgs::PoseStamped appro_target = align_z_axis_downwards(req.target_pose);

    appro_target.pose.position.z += 0.15;   //to approach the object from above

    if(move_arm(appro_target))
    {
        if(move_torso(TORSO_HOME_VALUE - TORSO_OFFSET_VALUE, 0))    
        {
            bool success = false;
            if(req.pick_or_place == 1)  //picking
                success = close_gripper();
            else                        //placing
                success = open_gripper();

            if(success)
            {
                if(req.pick_or_place == 1)//attahc if picking
                {
                    ros::ServiceClient gazebo_link_attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
                    gazebo_ros_link_attacher::Attach attacher;
                    attacher.request.model_name_1 = "tiago";
                    attacher.request.link_name_1 = "arm_7_link";
                    attacher.request.model_name_2 = req.object_id;
                    attacher.request.link_name_2 = req.object_id+ "_link";

                    if(gazebo_link_attach_client.call(attacher) && attacher.response.ok)
                    {
                        ROS_INFO("Object attached");
                        //arm.attachObject(req.object_id);
                        ros::ServiceClient client_collision = nh.serviceClient<ir2425_group_07_assignment2::collision_rem_service>("collision_object_rem");
                        ir2425_group_07_assignment2::collision_rem_service srv_collision;
                        srv_collision.request.object_id = req.object_id;
                        if(!client_collision.call(srv_collision))
                        {
                            ROS_ERROR("Error removing collision object");
                            return 1;
                        }
                        ROS_INFO("Removed the collision object correctly");
                    }
                } 
                else    //deteach if placing
                {
                    ros::ServiceClient gazebo_link_detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
                    gazebo_ros_link_attacher::Attach attacher;
                    attacher.request.model_name_1 = "tiago";
                    attacher.request.link_name_1 = "arm_7_link";
                    attacher.request.model_name_2 = req.object_id;
                    attacher.request.link_name_2 = req.object_id+ "_link";

                    if(gazebo_link_detach_client.call(attacher) && attacher.response.ok)
                    {
                        ROS_INFO("Object detached");
                        //arm.detachObject(req.object_id);
                    }
                }
                
                if(move_torso(TORSO_HOME_VALUE, 0)) //departing
                {
                    if(move_home())     //reaching home configuration
                    {
                        res.done = true;
                        res.header.frame_id = "base_link";
                        res.header.stamp = ros::Time::now();
                        return true;
                    }
                }
            }
        }
    }
    
    res.done = false;
    res.header.frame_id = "base_link";
    res.header.stamp = ros::Time::now();
    return false;
}