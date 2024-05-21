#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

// Globale variabelen om de huidige positie op te slaan
geometry_msgs::Point current_position_flycrane, current_position_flycrane1;
bool flycrane_hover_initiated = false;
bool flycrane1_hover_initiated = false;

void odometryCallbackflycrane(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position_flycrane = msg->pose.pose.position;
}

void odometryCallbackflycrane1(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position_flycrane1 = msg->pose.pose.position;
}

bool isPositionReached(const geometry_msgs::Point& current, const geometry_msgs::Point& target) {
    double tolerance = 0.1; // Stel een positie tolerantie in
    double distance = sqrt(pow(current.x - target.x, 2) +
                           pow(current.y - target.y, 2) +
                           pow(current.z - target.z, 2));
    return distance < tolerance;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "position_commander");
    ros::NodeHandle nh;
    std::this_thread::sleep_for(std::chrono::seconds(3)); // Wacht 3 seconden voor takeoff
    std::string drone_id1, drone_id2;
    nh.getParam("/BEP/Position_move_node/drone_id1", drone_id1);
    nh.getParam("/BEP/Position_move_node/drone_id2", drone_id2);

    ros::Publisher pose_pub_flycrane = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id1 + "/agiros_pilot/go_to_pose", 10);
    ros::Publisher pose_pub_flycrane1 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id2 + "/agiros_pilot/go_to_pose", 10);
    ros::Publisher hover_pub_flycrane = nh.advertise<std_msgs::Empty>("/" + drone_id1 + "/agiros_pilot/force_hover", 10, true);
    ros::Publisher hover_pub_flycrane1 = nh.advertise<std_msgs::Empty>("/" + drone_id2 + "/agiros_pilot/force_hover", 10, true);

    ros::Subscriber odom_sub_flycrane = nh.subscribe("/" + drone_id1 + "/agiros_pilot/odometry", 10, odometryCallbackflycrane);
    ros::Subscriber odom_sub_flycrane1 = nh.subscribe("/" + drone_id2 + "/agiros_pilot/odometry", 10, odometryCallbackflycrane1);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        geometry_msgs::Point target_position, target_position1;
        nh.getParam("/BEP/Position_move_node/target_x", target_position.x);
        nh.getParam("/BEP/Position_move_node/target_y", target_position.y);
        nh.getParam("/BEP/Position_move_node/target_z", target_position.z);
        nh.getParam("/BEP/Position_move_node/target_x1", target_position1.x);
        nh.getParam("/BEP/Position_move_node/target_y1", target_position1.y);
        nh.getParam("/BEP/Position_move_node/target_z1", target_position1.z);

        geometry_msgs::PoseStamped pose_flycrane, pose_flycrane1;
        pose_flycrane.header.stamp = ros::Time::now();
        pose_flycrane.header.frame_id = "world";
        pose_flycrane.pose.position = target_position;
        pose_flycrane.pose.orientation.w = sin(0.25*M_PI);
        pose_flycrane.pose.orientation.z = sin(0.25*M_PI);
        pose_flycrane1.header.stamp = ros::Time::now();
        pose_flycrane1.header.frame_id = "world";
        pose_flycrane1.pose.position = target_position1;
        pose_flycrane1.pose.orientation.w = sin(0.25*M_PI);
        pose_flycrane1.pose.orientation.z = sin(0.25*M_PI);

        pose_pub_flycrane.publish(pose_flycrane);
        pose_pub_flycrane1.publish(pose_flycrane1);

        if (isPositionReached(current_position_flycrane, target_position) && !flycrane_hover_initiated) {
            ROS_INFO("flycrane reaching target position, initiating hover.");
            hover_pub_flycrane.publish(std_msgs::Empty());
            flycrane_hover_initiated = true;
        }
        if (isPositionReached(current_position_flycrane1, target_position1) && !flycrane1_hover_initiated) {
            ROS_INFO("flycrane1 reaching target position, initiating hover.");
            hover_pub_flycrane1.publish(std_msgs::Empty());
            flycrane1_hover_initiated = true;
        }

        if (flycrane_hover_initiated && flycrane1_hover_initiated) {
            ROS_INFO("Both drones are in hover mode, exiting.");
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
