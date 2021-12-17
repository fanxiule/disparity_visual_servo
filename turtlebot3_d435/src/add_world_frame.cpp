#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

void poseCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    static tf::TransformBroadcaster br;
    geometry_msgs::Pose robot_pose = msg->pose[2];
    double pos_x = robot_pose.position.x;
    double pos_y = robot_pose.position.y;
    double pos_z = robot_pose.position.z;
    double orient_x = robot_pose.orientation.x;
    double orient_y = robot_pose.orientation.y;
    double orient_z = robot_pose.orientation.z;
    double orient_w = robot_pose.orientation.w;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos_x, pos_y, pos_z));
    transform.setRotation(tf::Quaternion(orient_x, orient_y, orient_z, orient_w)); // transform from origin to robot
    // needs to convert transform to from robot to origin
    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "base_footprint", "world"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_world_frame");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("/gazebo/model_states", 10, &poseCallback);

    ros::spin();
    return 0;
}