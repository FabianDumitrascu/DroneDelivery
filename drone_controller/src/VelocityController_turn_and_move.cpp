#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <cmath>
#include <sstream>
#include <thread>
#include <chrono>

class PIDController {
private:
    double Kp, Ki, Kd;
    double prev_error_x, prev_error_y, prev_error_z;
    double integral_x, integral_y, integral_z;

public:
    PIDController(double p, double i, double d) :
        Kp(p), Ki(i), Kd(d), prev_error_x(0), prev_error_y(0), prev_error_z(0),
        integral_x(0), integral_y(0), integral_z(0) {}

    geometry_msgs::Twist computeControl(const geometry_msgs::Point& target, const geometry_msgs::Point& current, double dt) {
        double error_x = target.x - current.x;
        double error_y = target.y - current.y;
        double error_z = target.z - current.z;
        integral_x += error_x * dt;
        integral_y += error_y * dt;
        integral_z += error_z * dt;
        double derivative_x = (error_x - prev_error_x) / dt;
        double derivative_y = (error_y - prev_error_y) / dt;
        double derivative_z = (error_z - prev_error_z) / dt;
        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_z = error_z;
        geometry_msgs::Twist twist;
        twist.linear.x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
        twist.linear.y = Kp * error_y + Ki * integral_y + Kd * derivative_y;
        twist.linear.z = Kp * error_z + Ki * integral_z + Kd * derivative_z;
        return twist;
    }

    double computeYawControl(double target_yaw, double current_yaw, double dt) {
        double error_yaw = normalizeAngle(target_yaw - current_yaw);
        double yaw_rate = Kp * error_yaw; // Simplified for demonstration
        return yaw_rate;
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

double getYawFromQuaternion(double x, double y, double z, double w) {
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_turn_and_move_node");
    ros::NodeHandle nh("~");  // Private namespace NodeHandle
    std::this_thread::sleep_for(std::chrono::seconds(4)); // Wait 4 seconds before takeoff

    // Parameters for target positions, PID coefficients, and yaw angles
    std::string drone_id1, drone_id2;
    double target_x, target_y, target_z, kp, ki, kd;
    double target_x1, target_y1, target_z1;
    double target_yaw, target_yaw1;

    // Retrieve parameters using the private namespace
    nh.param<std::string>("drone_id1", drone_id1, "falcon");
    nh.param<std::string>("drone_id2", drone_id2, "falcon1");
    nh.param<double>("target_x", target_x, 2.0);
    nh.param<double>("target_y", target_y, 1.0);
    nh.param<double>("target_z", target_z, 1.0);
    nh.param<double>("target_x1", target_x1, 2.0);
    nh.param<double>("target_y1", target_y1, -1.0);
    nh.param<double>("target_z1", target_z1, 1.0);
    nh.param<double>("kp", kp, 1.0);
    nh.param<double>("ki", ki, 0.01);
    nh.param<double>("kd", kd, 0.05);
    nh.param<double>("target_yaw", target_yaw, 0.0);
    nh.param<double>("target_yaw1", target_yaw1, 0.0);

    // Logging
    ROS_INFO("Drone ID 1: %s", drone_id1.c_str());
    ROS_INFO("Drone ID 2: %s", drone_id2.c_str());
    ROS_INFO("Target position: (%f, %f, %f)", target_x, target_y, target_z);
    ROS_INFO("Target2 position: (%f, %f, %f)", target_x1, target_y1, target_z1);
    ROS_INFO("Target yaw: %f, Target yaw1: %f", target_yaw, target_yaw1);
    ROS_INFO("PID Coefficients: Kp=%f, Ki=%f, Kd=%f", kp, ki, kd);

    // Topics
    std::ostringstream velocity_topic1, velocity_topic2, odometry_topic1, odometry_topic2;
    velocity_topic1 << "/" << drone_id1 << "/agiros_pilot" << "/velocity_command";
    odometry_topic1 << "/" << drone_id1 << "/agiros_pilot" << "/odometry";
    velocity_topic2 << "/" << drone_id2 << "/agiros_pilot" << "/velocity_command";
    odometry_topic2 << "/" << drone_id2 << "/agiros_pilot" << "/odometry";

    // PID Controllers
    PIDController pid1(kp, ki, kd), pid2(kp, ki, kd);

    // Publishers
    ros::Publisher vel_pub1 = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic1.str(), 10);
    ros::Publisher vel_pub2 = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic2.str(), 10);
    ROS_INFO("Publishing to %s and %s", velocity_topic1.str().c_str(), velocity_topic2.str().c_str());

    // Rate
    double loop_rate_hz = 50.0;
    ros::Rate loop_rate(loop_rate_hz);

    // Target positions
    geometry_msgs::Point target_position;
    target_position.x = target_x;
    target_position.y = target_y;
    target_position.z = target_z;

    geometry_msgs::Point target_position1;
    target_position1.x = target_x1;
    target_position1.y = target_y1;
    target_position1.z = target_z1;

    // Callbacks for each drone
    auto odometryCallback1 = [&vel_pub1, &pid1, &target_position, target_yaw, loop_rate_hz](const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TwistStamped velocity_command;
        velocity_command.header.stamp = ros::Time::now();
        velocity_command.twist = pid1.computeControl(target_position, msg->pose.pose.position, 1.0 / loop_rate_hz);
        double current_yaw = getYawFromQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        velocity_command.twist.angular.z = pid1.computeYawControl(target_yaw, current_yaw, 1.0 / loop_rate_hz);
        vel_pub1.publish(velocity_command);
    };

    auto odometryCallback2 = [&vel_pub2, &pid2, &target_position1, target_yaw1, loop_rate_hz](const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TwistStamped velocity_command;
        velocity_command.header.stamp = ros::Time::now();
        velocity_command.twist = pid2.computeControl(target_position1, msg->pose.pose.position, 1.0 / loop_rate_hz);
        double current_yaw = getYawFromQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        velocity_command.twist.angular.z = pid2.computeYawControl(target_yaw1, current_yaw, 1.0 / loop_rate_hz);
        vel_pub2.publish(velocity_command);
    };

    // Subscribers for each drone
    ros::Subscriber odom_sub1 = nh.subscribe<nav_msgs::Odometry>(odometry_topic1.str(), 10, odometryCallback1);
    ros::Subscriber odom_sub2 = nh.subscribe<nav_msgs::Odometry>(odometry_topic2.str(), 10, odometryCallback2);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
