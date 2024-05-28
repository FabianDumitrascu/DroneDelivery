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
    std::string drone_id1;
    std::string drone_id2;
    double target_yaw;
    double target_yaw1;
    double Mid_x, Mid_y, Mid_z;
    double Mid_yaw;
    double Mid_yaw_increment;
    double initial_yaw;
    double initial_yaw_degrees;
    double Mid_yaw_end_normalized_degrees;
    double Mid_yaw_end;
    double radius = 1.0;
    ros::Timer update_timer, shutdown_timer;

public:
    DroneController() : nh("~"), pid1(1.0, 0.01, 0.05, 10.0), pid2(1.0, 0.01, 0.05, 10.0) {
        nh.param<std::string>("drone_id1", drone_id1, "flycrane");
        nh.param<std::string>("drone_id2", drone_id2, "flycrane1");
        nh.param<double>("Mid_x", Mid_x, 2.0);
        nh.param<double>("Mid_y", Mid_y, 1.0);
        nh.param<double>("Mid_z", Mid_z, 1.0);
        nh.param<double>("Mid_yaw_end", Mid_yaw_end, 360.0); 
        Mid_yaw_end_normalized_degrees = normalizeAngle(Mid_yaw_end * M_PI / 180.0) *180 /M_PI;
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
            auto msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/mass1/agiros_pilot/odometry", nh, ros::Duration(5));
            if (msg) {
                double initial_yaw = getYawFromQuaternion(msg->pose.pose.orientation.x, 
                                                        msg->pose.pose.orientation.y,
                                                        msg->pose.pose.orientation.z,
                                                        msg->pose.pose.orientation.w);
                double initial_yaw_degrees = normalizeAngle(initial_yaw) * 180.0 / M_PI;
                
                Mid_yaw = initial_yaw_degrees;
                if (Mid_yaw_end_normalized_degrees > initial_yaw_degrees and Mid_yaw_end_normalized_degrees - initial_yaw_degrees > 180.0) {
                    Mid_yaw_increment = -30.0;  //bvb van -90 naar 180 graden
                    Mid_yaw_end_normalized_degrees += -360.0;
                } else if (Mid_yaw_end_normalized_degrees > initial_yaw_degrees and Mid_yaw_end_normalized_degrees - initial_yaw_degrees <= 180.0) {
                    Mid_yaw_increment = 30.0;    // bvb can 0 naar 90 graden
                } else if (Mid_yaw_end_normalized_degrees < initial_yaw_degrees and initial_yaw_degrees - Mid_yaw_end_normalized_degrees > 180.0) {
                    Mid_yaw_increment = 30.0;    // bvb van 180 naar -90 graden
                    Mid_yaw_end_normalized_degrees += 360.0;
                } else if (Mid_yaw_end_normalized_degrees < initial_yaw_degrees and initial_yaw_degrees - Mid_yaw_end_normalized_degrees <= 180.0) {
                    Mid_yaw_increment = -30.0;   // bvb van 90 naar 0 graden
                }
                ROS_INFO("Initial Yaw: %f degrees", initial_yaw_degrees);
                ROS_INFO("End Yaw: %f degrees", Mid_yaw_end_normalized_degrees);
                ROS_INFO("Mid_yaw_increment: %f", Mid_yaw_increment);
                initialYawFound = true;
            } else {
                ROS_WARN("No odometry message received. Retrying in 1 second...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    

    void setupCommunication() {
        vel_pub1 = nh.advertise<geometry_msgs::TwistStamped>("/flycrane/agiros_pilot/velocity_command", 10);
        vel_pub2 = nh.advertise<geometry_msgs::TwistStamped>("/flycrane1/agiros_pilot/velocity_command", 10);
        odom_sub1 = nh.subscribe<nav_msgs::Odometry>("/mass1/agiros_pilot/odometry", 10, &DroneController::odometryCallback1, this);
        odom_sub2 = nh.subscribe<nav_msgs::Odometry>("/mass2/agiros_pilot/odometry", 10, &DroneController::odometryCallback2, this);
    }


    void setupTimer() {
        update_timer = nh.createTimer(ros::Duration(0.3), &DroneController::updateCallback, this);
    }

    void updateCallback(const ros::TimerEvent&) {
        
        if (initial_yaw_degrees < Mid_yaw_end_normalized_degrees) {
            if (Mid_yaw >= Mid_yaw_end_normalized_degrees) {
                Mid_yaw = Mid_yaw_end_normalized_degrees;
                updatePositions(Mid_yaw_end_normalized_degrees * M_PI / 180.0);
                ROS_INFO("Reached final yaw: %f", Mid_yaw_end_normalized_degrees);
                ROS_INFO("last target position: (%f, %f, %f, %f)", target_position.x, target_position.y, target_position.z, target_yaw* 180 / M_PI);
                ROS_INFO("last target position1: (%f, %f, %f, %f)", target_position1.x, target_position1.y, target_position1.z, target_yaw1* 180 / M_PI);   
                shutdown_timer = nh.createTimer(ros::Duration(5), &DroneController::shutdownCallback, this, true);
                update_timer.stop();
            }
            else {
                Mid_yaw += Mid_yaw_increment;
                ROS_INFO("Current yaw: %f", Mid_yaw);
                double Mid_yaw_radians = Mid_yaw * M_PI / 180.0;
                updatePositions(Mid_yaw_radians);
            }
        } 
        else {
            if (Mid_yaw <= Mid_yaw_end_normalized_degrees) {
                Mid_yaw = Mid_yaw_end_normalized_degrees;
                updatePositions(Mid_yaw_end_normalized_degrees * M_PI / 180.0);
                ROS_INFO("Reached final yaw: %f", Mid_yaw_end_normalized_degrees);
                ROS_INFO("last target position: (%f, %f, %f, %f)", target_position.x, target_position.y, target_position.z, target_yaw* 180 / M_PI);
                ROS_INFO("last target position1: (%f, %f, %f, %f)", target_position1.x, target_position1.y, target_position1.z, target_yaw1* 180 / M_PI);   
                shutdown_timer = nh.createTimer(ros::Duration(5), &DroneController::shutdownCallback, this, true);
                update_timer.stop();
            }
            else {
                Mid_yaw += Mid_yaw_increment;
                ROS_INFO("Current yaw: %f", Mid_yaw);
                double Mid_yaw_radians = Mid_yaw * M_PI / 180.0;
                updatePositions(Mid_yaw_radians);
            }
        }
    }
    void shutdownCallback(const ros::TimerEvent&) {
        ROS_INFO("Shutdown after holding position.");
        ros::shutdown();
    }
    void updatePositions(double yaw_radians) {
        target_position.x = Mid_x + radius * cos(yaw_radians);
        target_position.y = Mid_y + radius * sin(yaw_radians);
        target_position.z = Mid_z;
        target_position1.x = Mid_x + radius * cos(yaw_radians + M_PI);
        target_position1.y = Mid_y + radius * sin(yaw_radians + M_PI);
        target_position1.z = Mid_z;
        target_yaw = yaw_radians;
        target_yaw1 = yaw_radians;
    }

    void odometryCallback1(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TwistStamped velocity_command;
        velocity_command.header.stamp = ros::Time::now();
        velocity_command.twist = pid1.computeControl(target_position, msg->pose.pose.position, 1.0 / 50.0);
        double current_yaw = getYawFromQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        velocity_command.twist.angular.z = pid1.computeYawControl(target_yaw, current_yaw, 1.0 / 50.0);
        vel_pub1.publish(velocity_command);
    }

    void odometryCallback2(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TwistStamped velocity_command;
        velocity_command.header.stamp = ros::Time::now();
        velocity_command.twist = pid2.computeControl(target_position1, msg->pose.pose.position, 1.0 / 50.0);
        double current_yaw = getYawFromQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        velocity_command.twist.angular.z = pid2.computeYawControl(target_yaw1, current_yaw, 1.0 / 50.0);
        vel_pub2.publish(velocity_command);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_turn_and_move_node");
    std::this_thread::sleep_for(std::chrono::seconds(4)); // Wacht 4 seconden voor takeoff
    DroneController controller;
    ros::spin();
    return 0;
}

