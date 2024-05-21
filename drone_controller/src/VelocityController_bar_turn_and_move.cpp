#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <cmath>
#include <sstream>
#include <thread>
#include <chrono>
#include <tf/tf.h>

class PIDController {
private:
    double Kp, Ki, Kd;
    double prev_error_x, prev_error_y, prev_error_z;
    double integral_x, integral_y, integral_z;
    double integral_limit;

public:
    PIDController(double p, double i, double d, double limit) :
        Kp(p), Ki(i), Kd(d), prev_error_x(0), prev_error_y(0), prev_error_z(0),
        integral_x(0), integral_y(0), integral_z(0), integral_limit(limit) {}

    geometry_msgs::Twist computeControl(const geometry_msgs::Point& target, const geometry_msgs::Point& current, double dt) {
        double error_x = target.x - current.x;
        double error_y = target.y - current.y;
        double error_z = target.z - current.z;
        integral_x += error_x * dt;
        integral_y += error_y * dt;
        integral_z += error_z * dt;
        
        // Apply integral windup protection
        integral_x = std::max(-integral_limit, std::min(integral_x, integral_limit));
        integral_y = std::max(-integral_limit, std::min(integral_y, integral_limit));
        integral_z = std::max(-integral_limit, std::min(integral_z, integral_limit));
        
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
        double yaw_rate = Kp * error_yaw;
        return yaw_rate;
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

class DroneController {
private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Publisher vel_pub1;
    ros::Subscriber odom_sub;
    PIDController pid;
    PIDController pid1;
    geometry_msgs::Point target_position;
    geometry_msgs::Point target_position1;
    std::string drone_id1;
    std::string drone_id2;
    std::string bar_odometry_topic;
    double target_yaw;
    double target_yaw1;
    double target_x, target_y, target_z;
    double target_yaw_increment;
    double initial_yaw;
    double initial_yaw_degrees;
    double target_yaw_end_normalized_degrees;
    double target_yaw_end;
    double radius = 1.0;
    ros::Timer update_timer, shutdown_timer;
    geometry_msgs::Point initial_position_bar;

public:
    DroneController() : nh("~"), pid(1.0, 0.01, 0.05, 10.0), pid1(1.0, 0.01, 0.05, 10.0) {
        nh.param<std::string>("drone_id1", drone_id1, "falcon");
        nh.param<std::string>("drone_id2", drone_id2, "falcon1");
        nh.param<std::string>("bar_odometry_topic", bar_odometry_topic, "/bar/agiros_pilot/odometry");
        nh.param<double>("target_x", target_x, 2.0);
        nh.param<double>("target_y", target_y, 1.0);
        nh.param<double>("target_z", target_z, 1.0);
        nh.param<double>("target_yaw_end", target_yaw_end, 360.0); 
        target_yaw_end_normalized_degrees = normalizeAngle(target_yaw_end * M_PI / 180.0) *180 /M_PI;
        setupTimer();
        setupCommunication();
        fetchInitialYaw();
    }
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    void fetchInitialYaw() {
        bool initialYawFound = false;
        while (!initialYawFound) {
            auto msg = ros::topic::waitForMessage<nav_msgs::Odometry>(bar_odometry_topic, nh, ros::Duration(5));
            if (msg) {
                tf::Quaternion q(msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z,
                                msg->pose.pose.orientation.w);
                double roll, pitch, yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                initial_yaw = yaw;  // Use the yaw extracted from quaternion
                initial_yaw_degrees = normalizeAngle(initial_yaw) * 180.0 / M_PI;
                target_yaw = initial_yaw_degrees;
                if (target_yaw_end_normalized_degrees > initial_yaw_degrees && target_yaw_end_normalized_degrees - initial_yaw_degrees > 180.0) {
                    target_yaw_increment = -30.0;  // e.g., from -90 to 180 degrees
                    target_yaw_end_normalized_degrees -= 360.0;
                } else if (target_yaw_end_normalized_degrees > initial_yaw_degrees && target_yaw_end_normalized_degrees - initial_yaw_degrees <= 180.0) {
                    target_yaw_increment = 30.0;    // e.g., from 0 to 90 degrees
                } else if (target_yaw_end_normalized_degrees < initial_yaw_degrees && initial_yaw_degrees - target_yaw_end_normalized_degrees > 180.0) {
                    target_yaw_increment = 30.0;    // e.g., from 180 to -90 degrees
                    target_yaw_end_normalized_degrees += 360.0;
                } else if (target_yaw_end_normalized_degrees < initial_yaw_degrees && initial_yaw_degrees - target_yaw_end_normalized_degrees <= 180.0) {
                    target_yaw_increment = -30.0;   // e.g., from 90 to 0 degrees
                }
                ROS_INFO("Initial Yaw: %f degrees", initial_yaw_degrees);
                ROS_INFO("End Yaw: %f degrees", target_yaw_end_normalized_degrees);
                ROS_INFO("target_yaw_increment: %f", target_yaw_increment);
                initialYawFound = true;
            } else {
                ROS_WARN("No odometry message received. Retrying in 1 second...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    
    void setupCommunication() {
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/falcon/agiros_pilot/velocity_command", 10);
        vel_pub1 = nh.advertise<geometry_msgs::TwistStamped>("/falcon1/agiros_pilot/velocity_command", 10);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(bar_odometry_topic, 10, &DroneController::odometryCallbackBar, this);
    }

    void setupTimer() {
        update_timer = nh.createTimer(ros::Duration(0.3), &DroneController::updateCallback, this);
    }

    void updateCallback(const ros::TimerEvent&) {
        if (initial_yaw_degrees < target_yaw_end_normalized_degrees) {
            if (target_yaw >= target_yaw_end_normalized_degrees) {
                target_yaw = target_yaw_end_normalized_degrees;
                updatePositions(target_yaw_end_normalized_degrees * M_PI / 180.0);
                ROS_INFO("Reached final yaw: %f", target_yaw_end_normalized_degrees);
                ROS_INFO("last target position: (%f, %f, %f, %f)", target_position.x, target_position.y, target_position.z, target_yaw * 180 / M_PI);
                ROS_INFO("last target position1: (%f, %f, %f, %f)", target_position1.x, target_position1.y, target_position1.z, target_yaw1 * 180 / M_PI);
                shutdown_timer = nh.createTimer(ros::Duration(5), &DroneController::shutdownCallback, this, true);
                update_timer.stop();
            } else {
                target_yaw += target_yaw_increment;
                ROS_INFO("Current yaw: %f", target_yaw);
                double target_yaw_radians = target_yaw * M_PI / 180.0;
                updatePositions(target_yaw_radians);
            }
        } else {
            if (target_yaw <= target_yaw_end_normalized_degrees) {
                target_yaw = target_yaw_end_normalized_degrees;
                updatePositions(target_yaw_end_normalized_degrees * M_PI / 180.0);
                ROS_INFO("Reached final yaw: %f", target_yaw_end_normalized_degrees);
                ROS_INFO("last target position: (%f, %f, %f, %f)", target_position.x, target_position.y, target_position.z, target_yaw * 180 / M_PI);
                ROS_INFO("last target position1: (%f, %f, %f, %f)", target_position1.x, target_position1.y, target_position1.z, target_yaw1 * 180 / M_PI);
                shutdown_timer = nh.createTimer(ros::Duration(5), &DroneController::shutdownCallback, this, true);
                update_timer.stop();
            } else {
                target_yaw += target_yaw_increment;
                ROS_INFO("Current yaw: %f", target_yaw);
                double target_yaw_radians = target_yaw * M_PI / 180.0;
                updatePositions(target_yaw_radians);
            }
        }
    }
    void shutdownCallback(const ros::TimerEvent&) {
        ROS_INFO("Shutdown after holding position.");
        ros::shutdown();
    }
    void updatePositions(double yaw_radians) {
        target_position.x = target_x + radius * cos(yaw_radians);
        target_position.y = target_y + radius * sin(yaw_radians);
        target_position.z = target_z;
        target_position1.x = target_x + radius * cos(yaw_radians + M_PI);
        target_position1.y = target_y + radius * sin(yaw_radians + M_PI);
        target_position1.z = target_z;
        target_yaw = yaw_radians;
        target_yaw1 = yaw_radians;
    }

    void odometryCallbackBar(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TwistStamped velocity_command;
        geometry_msgs::TwistStamped velocity_command1;
        velocity_command.header.stamp = ros::Time::now();
        velocity_command1.header.stamp = ros::Time::now();
        
        // Compute linear control commands
        velocity_command.twist = pid.computeControl(target_position, msg->pose.pose.position, 1.0 / 50.0);
        velocity_command1.twist = pid1.computeControl(target_position1, msg->pose.pose.position, 1.0 / 50.0);
        
        // Convert quaternion to yaw
        tf::Quaternion q(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // Use the yaw angle for angular control
        velocity_command.twist.angular.z = pid.computeYawControl(target_yaw, yaw, 1.0 / 50.0);
        velocity_command1.twist.angular.z = pid1.computeYawControl(target_yaw1, yaw, 1.0 / 50.0);
        
        // Publish the velocity commands
        vel_pub.publish(velocity_command);
        vel_pub1.publish(velocity_command1);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_turn_and_move_node");
    std::this_thread::sleep_for(std::chrono::seconds(4)); // Wacht 4 seconden voor takeoff
    DroneController controller;
    ros::spin();
    return 0;
}

