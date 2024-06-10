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

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

class DroneController {
private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub1;
    ros::Publisher vel_pub2;
    ros::Subscriber odom_sub1;
    ros::Subscriber odom_sub2;
    ros::Subscriber odom_sub_bar;
    PIDController pid1;
    PIDController pid2;
    geometry_msgs::Point target_position;
    geometry_msgs::Point target_position1;
    std::string drone_id1;
    std::string drone_id2;
    std::ostringstream velocity_topic1, velocity_topic2;  
    std::ostringstream odometry_topic1, odometry_topic2; 
    double target_yaw;
    double target_yaw1;
    double Mid_x, Mid_y, Mid_z;
    double Mid_yaw;
    double Mid_yaw_increment;
    double initial_yaw_degrees;
    double Mid_yaw_end_normalized_degrees;
    double Mid_yaw_end;
    double radius = 0.55;
    double Kp, Ki, Kd;
    double startX, startY;
    double centerX, centerY;
    double currentYawBar;
    double etha_yaw, etha_translation;
    bool initialYawFound;
    bool simulation;
    bool Landing;
    bool stop_movement;
    double regenboogpaard;
    geometry_msgs::Point initial_position_falcon1, initial_position_falcon2, current_position_bar;
    geometry_msgs::Pose currentPoseBar;  // Use geometry_msgs::Pose for currentPoseBar
    geometry_msgs::Point current_position_falcon1, current_position_falcon2;
    ros::Timer update_timer, shutdown_timer;

public:
    DroneController() : nh("~"), pid1(0.0, 0.0, 0.0, 10.0), pid2(0.0, 0.0, 0.0, 10.0), stop_movement(false), initialYawFound(false), Landing(false), etha_yaw(0.0), etha_translation(0.0), regenboogpaard(0.0), simulation(false) {
        nh.param<std::string>("drone_id1", drone_id1, "flycrane");
        nh.param<std::string>("drone_id2", drone_id2, "flycrane1");
        nh.getParam("Mid_x", Mid_x);
        nh.getParam("Mid_y", Mid_y);
        nh.getParam("Mid_z", Mid_z);
        nh.getParam("kp", Kp);
        nh.getParam("ki", Ki);
        nh.getParam("kd", Kd);
        nh.getParam("etha_yaw", etha_yaw);
        nh.getParam("etha_translation", etha_translation);
        nh.getParam("begin_yaw", initial_yaw_degrees);
        nh.param<double>("Mid_yaw_end", Mid_yaw_end, 360.0); 
        nh.getParam("simulation", simulation);
        Mid_yaw_end_normalized_degrees = normalizeAngle(Mid_yaw_end * M_PI / 180.0) * 180 / M_PI;
        setupTimer();
        setupCommunication();
        pid1 = PIDController(Kp, Ki, Kd, 10.0);
        pid2 = PIDController(Kp, Ki, Kd, 10.0);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    void fetchInitialYaw() {
        if (initial_position_falcon1.x != 0.0000000 && initial_position_falcon2.x != 0.0000000) {
            // auto msg = ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic1.str(), nh, ros::Duration(5));
            // startX = (initial_position_falcon1.x + initial_position_falcon2.x) / 2;
            // startY = (initial_position_falcon1.y + initial_position_falcon2.y) / 2;
            // double dx = initial_position_falcon1.x - startX;
            // double dy = initial_position_falcon1.y - startY;
            // centerX = startX;
            // centerY = startY;
            // initial_yaw_degrees = atan2(dy, dx) * 180.0 / M_PI;
            Mid_yaw = initial_yaw_degrees;
            if (Mid_yaw_end_normalized_degrees > initial_yaw_degrees) {
                Mid_yaw_increment = 30.0;
            } else if (Mid_yaw_end_normalized_degrees < initial_yaw_degrees) {
                Mid_yaw_increment = -30.0;
            } else {
                Mid_yaw_increment = 0.0;
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
    

    void setupCommunication() {
        velocity_topic1 << "/" << drone_id1 << "/agiros_pilot" << "/velocity_command";
        velocity_topic2 << "/" << drone_id2 << "/agiros_pilot" << "/velocity_command";

        // odometry_topic1 << "/mocap/" << drone_id1 << "/agiros_pilot" << "/odometry";
        // odometry_topic2 << "/mocap/" << drone_id2 << "/agiros_pilot" << "/odometry";

        odometry_topic1 << "/" << drone_id1 << "/agiros_pilot" << "/odometry";
        odometry_topic2 << "/" << drone_id2 << "/agiros_pilot" << "/odometry";
        if (simulation){
            odom_sub_bar = nh.subscribe("/bar/odometry_sensor1/odometry", 10, &DroneController::barCallbacksimulation, this);
        }
        if (!simulation){
            odom_sub_bar = nh.subscribe("/mocap/bar_large/pose", 10, &DroneController::barCallback, this);
        }
        vel_pub1 = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic1.str(), 10);
        vel_pub2 = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic2.str(), 10);
        
        odom_sub1 = nh.subscribe<nav_msgs::Odometry>(odometry_topic1.str(), 10, &DroneController::odometryCallback1, this);
        odom_sub2 = nh.subscribe<nav_msgs::Odometry>(odometry_topic2.str(), 10, &DroneController::odometryCallback2, this);
    }


    void setupTimer() {
        update_timer = nh.createTimer(ros::Duration(1), &DroneController::updateCallback, this);
    }
        double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
    void updateCallback(const ros::TimerEvent&) {

        if (stop_movement) {
            ROS_WARN("Stopping the node due to proximity alert.");
            ros::shutdown();
            return;
        }        
        if (initialYawFound == true and Landing == false) {
            if (initial_yaw_degrees < Mid_yaw_end_normalized_degrees) {
                if (Mid_yaw >= Mid_yaw_end_normalized_degrees) {
                    Mid_yaw = Mid_yaw_end_normalized_degrees;
                    updatePositions(Mid_yaw_end_normalized_degrees * M_PI / 180.0);
                    ROS_INFO("Reached final yaw: %f", Mid_yaw_end_normalized_degrees);
                    ROS_INFO("last target position: (%f, %f, %f, %f)", target_position.x, target_position.y, target_position.z, target_yaw* 180 / M_PI);
                    ROS_INFO("last target position1: (%f, %f, %f, %f)", target_position1.x, target_position1.y, target_position1.z, target_yaw1* 180 / M_PI);
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
        else {
            fetchInitialYaw();
        }
    }
    double QuaternionToYaw(geometry_msgs::Quaternion q) {
        double roll, pitch, yaw;
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    void updatePositions(double yaw_radians) {
        target_position.x = Mid_x + radius * cos(yaw_radians);
        target_position.y = Mid_y + radius * sin(yaw_radians);
        target_position.z = Mid_z+1;
        target_position1.x = Mid_x + radius * cos(yaw_radians + M_PI);
        target_position1.y = Mid_y + radius * sin(yaw_radians + M_PI);
        target_position1.z = Mid_z+1;
        target_yaw = yaw_radians;
        target_yaw1 = yaw_radians;
    }

    void odometryCallback1(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::Point next_position = msg->pose.pose.position;
        
        double distance_to_other_drone = sqrt(pow(next_position.x - current_position_falcon2.x, 2) +
                                            pow(next_position.y - current_position_falcon2.y, 2) +
                                            pow(next_position.z - current_position_falcon2.z, 2));
        if (distance_to_other_drone < 0.5) { // Example threshold distance
            ROS_WARN("Drones too close! Stopping movement. Distance: %f", distance_to_other_drone);
            stop_movement = true;
        }
        
        if (!stop_movement) {
            geometry_msgs::TwistStamped velocity_command;
            velocity_command.header.stamp = ros::Time::now();
            velocity_command.twist = pid1.computeControl(target_position, next_position, 1.0 / 50.0);
            if (!initialYawFound) {
                initial_position_falcon1 = msg->pose.pose.position;
                fetchInitialYaw();
            } else {
                vel_pub1.publish(velocity_command);
                current_position_falcon1 = msg->pose.pose.position;
            }
        }
    }

    void odometryCallback2(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::Point next_position = msg->pose.pose.position;
        
        double distance_to_other_drone = sqrt(pow(next_position.x - current_position_falcon1.x, 2) +
                                            pow(next_position.y - current_position_falcon1.y, 2) +
                                            pow(next_position.z - current_position_falcon1.z, 2));
        if (distance_to_other_drone < 0.5) { // Example threshold distance
            ROS_WARN("Drones too close! Stopping movement. Distance: %f", distance_to_other_drone);
            stop_movement = true;
        }
        
        if (!stop_movement) {
            geometry_msgs::TwistStamped velocity_command;
            velocity_command.header.stamp = ros::Time::now();
            velocity_command.twist = pid2.computeControl(target_position1, next_position, 1.0 / 50.0);
            if (!initialYawFound) {
                initial_position_falcon2 = msg->pose.pose.position;
            } else {
                vel_pub2.publish(velocity_command);
                current_position_falcon2 = msg->pose.pose.position;
            }
        }
    }

    
    void barCallbacksimulation(const nav_msgs::Odometry::ConstPtr& msg) {
        currentPoseBar = msg->pose.pose; 
        geometry_msgs::Quaternion q = msg->pose.pose.orientation;
        currentYawBar = QuaternionToYaw(q) * 180.0 / M_PI;
        double error_yaw, error_translation;
        error_yaw = currentYawBar - Mid_yaw_end;
        error_translation = sqrt(pow(currentPoseBar.position.x - Mid_x, 2) + pow(currentPoseBar.position.y - Mid_y, 2));  // Access position members
        // ROS_INFO("error_yaw: %f, error_translation: %f", error_yaw, error_translation);
        if (fabs(error_yaw) < etha_yaw && error_translation < etha_translation) { 
            regenboogpaard += 0.1;
            ROS_INFO("error_yaw: %f, error_translation: %f, regenboogpaard: %f", error_yaw, error_translation, regenboogpaard);
            if (regenboogpaard >= 10.0) {  // Use >= to ensure the timer condition is met
                Landscript();
                Landing = true;
            }
        } else {
            regenboogpaard = 0;
        }
    }

    void barCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        currentPoseBar = msg->pose; 
        geometry_msgs::Quaternion q = msg->pose.orientation;
        currentYawBar = QuaternionToYaw(q) * 180.0 / M_PI;
        double error_yaw, error_translation;
        error_yaw = 90 + currentYawBar - Mid_yaw_end;
        error_translation = sqrt(pow(currentPoseBar.position.x - Mid_x, 2) + pow(currentPoseBar.position.y - Mid_y, 2));  // Access position members
        // ROS_INFO("error_yaw: %f, error_translation: %f", error_yaw, error_translation);
        if (fabs(error_yaw) < etha_yaw && error_translation < etha_translation) { 
            regenboogpaard += 0.01;
            ROS_INFO("error_yaw: %f, error_translation: %f, regenboogpaard: %f", error_yaw, error_translation, regenboogpaard);
            if (regenboogpaard >= 5.0) {  // Use >= to ensure the timer condition is met
                Landscript();
                Landing = true;
            }
        } else {
            regenboogpaard = 0;
        }
    }

    void Landscript() {
        target_position.x = Mid_x + radius * cos(Mid_yaw_end * M_PI / 180.0);
        target_position.y = Mid_y + radius * sin(Mid_yaw_end * M_PI / 180.0);
        target_position.z = 0.5;
        target_position1.x = Mid_x + radius * cos(Mid_yaw_end * M_PI / 180.0 + M_PI);
        target_position1.y = Mid_y + radius * sin(Mid_yaw_end * M_PI / 180.0 + M_PI);
        target_position1.z = 0.5;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_turn_and_move_node");
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Wacht 4 seconden voor takeoff
    DroneController controller;
    ros::spin();
    return 0;
}

