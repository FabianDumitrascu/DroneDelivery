#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <cmath>
#include <tf/tf.h>
#include <chrono>
#include <thread>

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
        //ROS_INFO("Error: x = %f, y = %f, z = %f", error_x, error_y, error_z);

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
        //ROS_INFO("Error: x = %f, y = %f, z = %f", error_x, error_y, error_z);
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
    std::string bar_odometry_topic;
    double target_x, target_y, target_z, target_yaw;
    double target_yaw_radians;
    double distance_between_drones;

public:
    DroneController() : nh("~"), pid(0.5, 0.0, 0.01, 10.0), pid1(0.5, 0.0, 0.01, 10.0) {
        nh.param<std::string>("bar_odometry_topic", bar_odometry_topic, "/bar/odometry_sensor1/odometry");
        nh.param<double>("target_x", target_x, 2.0);
        nh.param<double>("target_y", target_y, 1.0);
        nh.param<double>("target_z", target_z, 1.0);
        nh.param<double>("target_yaw", target_yaw, 0.0);
        nh.param<double>("distance_between_drones", distance_between_drones, 1.0); // Example value
        target_yaw_radians = angleToRadians(target_yaw); // Initialize target_yaw_radians
        ROS_INFO("target_yaw_radians: %f", target_yaw_radians);
        setupCommunication();
    }

    void setupCommunication() {
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/falcon/agiros_pilot/velocity_command", 10);
        vel_pub1 = nh.advertise<geometry_msgs::TwistStamped>("/falcon1/agiros_pilot/velocity_command", 10);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(bar_odometry_topic, 10, &DroneController::odometryCallbackBar, this);
    }

    double angleToRadians(double angle) {
        angle = fmod(angle, 360.0); // Ensure angle is within 0 to 360 degrees
        if (angle > 180.0) {
            angle -= 360.0; // Convert angle to range -180 to 180 degrees
        }
        return angle * M_PI / 180.0; // Convert angle to radians
    }

    void odometryCallbackBar(const nav_msgs::Odometry::ConstPtr& msg) {
        // Calculate target positions for the drones to generate the desired torque
        target_position.x = target_x;
        target_position.y = target_y;
        target_position.z = target_z;

        geometry_msgs::TwistStamped velocity_command;
        geometry_msgs::TwistStamped velocity_command1;
        velocity_command.header.stamp = ros::Time::now();
        velocity_command1.header.stamp = ros::Time::now();

        // Compute control commands for each drone
        velocity_command.twist = pid.computeControl(target_position, msg->pose.pose.position, 1.0 / 50.0);
        velocity_command1.twist = pid1.computeControl(target_position, msg->pose.pose.position, 1.0 / 50.0);

        // Publish the velocity commands
        vel_pub.publish(velocity_command);
        vel_pub1.publish(velocity_command1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_turn_and_move_node");
    std::this_thread::sleep_for(std::chrono::seconds(4)); // Wait 4 seconds for takeoff
    DroneController controller;
    ros::spin();
    return 0;
}
