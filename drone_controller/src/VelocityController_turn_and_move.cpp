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

class DroneController {
private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub1;
    ros::Publisher vel_pub2;
    ros::Subscriber odom_sub1;
    ros::Subscriber odom_sub2;
    PIDController pid1;
    PIDController pid2;
    geometry_msgs::Point target_position;
    geometry_msgs::Point target_position1;
    double target_yaw;
    double target_yaw1;

public:
    DroneController() : nh("~"), pid1(1.0, 0.01, 0.05), pid2(1.0, 0.01, 0.05) {
        std::string drone_id1, drone_id2;
        double target_x, target_y, target_z, kp, ki, kd, target_x1, target_y1, target_z1, Mid_x, Mid_y, Mid_z, Mid_yaw;
        double radius = 1.0;

        nh.param<std::string>("drone_id1", drone_id1, "falcon");
        nh.param<std::string>("drone_id2", drone_id2, "falcon1");
        nh.param<double>("Mid_x", Mid_x, 2.0);
        nh.param<double>("Mid_y", Mid_y, 1.0);
        nh.param<double>("Mid_z", Mid_z, 1.0);
        nh.param<double>("kp", kp, 1.0);
        nh.param<double>("ki", ki, 0.01);
        nh.param<double>("kd", kd, 0.05);
        nh.param<double>("Mid_yaw", Mid_yaw, 0.0);
        double Mid_yaw_radians = Mid_yaw * M_PI / 180.0;
        target_x = Mid_x + radius * cos(Mid_yaw_radians);
        target_y = Mid_y + radius * sin(Mid_yaw_radians);
        target_z = Mid_z;
        target_x1 = Mid_x + radius * cos(Mid_yaw_radians + M_PI);
        target_y1 = Mid_y + radius * sin(Mid_yaw_radians + M_PI);
        target_z1 = Mid_z;
        target_yaw = Mid_yaw_radians;
        target_yaw1 = Mid_yaw_radians;

        std::ostringstream velocity_topic1, velocity_topic2, odometry_topic1, odometry_topic2;
        velocity_topic1 << "/" << drone_id1 << "/agiros_pilot" << "/velocity_command";
        odometry_topic1 << "/" << drone_id1 << "/agiros_pilot" << "/odometry";
        velocity_topic2 << "/" << drone_id2 << "/agiros_pilot" << "/velocity_command";
        odometry_topic2 << "/" << drone_id2 << "/agiros_pilot" << "/odometry";

        vel_pub1 = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic1.str(), 10);
        vel_pub2 = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic2.str(), 10);

        ros::Rate loop_rate(50.0);

        target_position.x = target_x;
        target_position.y = target_y;
        target_position.z = target_z;

        target_position1.x = target_x1;
        target_position1.y = target_y1;
        target_position1.z = target_z1;

        auto odometryCallback1 = [this](const nav_msgs::Odometry::ConstPtr& msg) {
            geometry_msgs::TwistStamped velocity_command;
            velocity_command.header.stamp = ros::Time::now();
            velocity_command.twist = pid1.computeControl(target_position, msg->pose.pose.position, 1.0 / 50.0);
            double current_yaw = getYawFromQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            velocity_command.twist.angular.z = pid1.computeYawControl(target_yaw, current_yaw, 1.0 / 50.0);
            vel_pub1.publish(velocity_command);
        };

        auto odometryCallback2 = [this](const nav_msgs::Odometry::ConstPtr& msg) {
            geometry_msgs::TwistStamped velocity_command;
            velocity_command.header.stamp = ros::Time::now();
            velocity_command.twist = pid2.computeControl(target_position1, msg->pose.pose.position, 1.0 / 50.0);
            double current_yaw = getYawFromQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            velocity_command.twist.angular.z = pid2.computeYawControl(target_yaw1, current_yaw, 1.0 / 50.0);
            vel_pub2.publish(velocity_command);
        };

        odom_sub1 = nh.subscribe<nav_msgs::Odometry>(odometry_topic1.str(), 10, odometryCallback1);
        odom_sub2 = nh.subscribe<nav_msgs::Odometry>(odometry_topic2.str(), 10, odometryCallback2);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_turn_and_move_node");
    std::this_thread::sleep_for(std::chrono::seconds(4));

    DroneController droneController;

    return 0;
}
