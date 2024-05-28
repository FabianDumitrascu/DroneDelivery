#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <cmath>
#include <tf/tf.h>
#include <chrono>
#include <thread>

enum ControlPhase {
    ROTATION,
    TRANSLATION
};

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
};

class DroneController {
private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Publisher vel_pub1;
    ros::Subscriber odom_sub_bar;
    ros::Subscriber odom_sub1;
    ros::Subscriber odom_sub2;
    PIDController pid;
    PIDController pid_turn;
    std::string bar_odometry_topic;
    std::string drone_id1, drone_id2;
    double target_x, target_y, target_z, target_yaw_end;
    double mid_x, mid_y;
    double new_target_yaw_radians;
    double target_yaw_radians;
    double distance_between_drones;
    geometry_msgs::Point current_position1;
    geometry_msgs::Point current_position2;
    geometry_msgs::Point current_position_bar;
    geometry_msgs::Point target_position1;
    geometry_msgs::Point target_position2;
    ControlPhase phase;
    double increment_yaw;
    bool initial_positions_set;
    bool odometry_received1;
    bool odometry_received2;
    double current_yaw; // Track current yaw
    ros::Timer update_timer;
    ros::Timer transition_timer;

public:
    DroneController()
        : nh("~"),
          pid(0.0, 0.0, 0.0, 10.0), // Initializing with default values
          pid_turn(0.9, 0.0, 0.0, 10.0), // Initializing with default values
          
          phase(ROTATION),
          initial_positions_set(false),
          odometry_received1(false),
          odometry_received2(false) {
        
        nh.param<std::string>("bar_odometry_topic", bar_odometry_topic, "/bar/odometry_sensor1/odometry");
        nh.param<std::string>("drone_id1", drone_id1, "falcon1");
        nh.param<std::string>("drone_id2", drone_id2, "falcon2");
        nh.param<double>("target_x", target_x, 2.0);
        nh.param<double>("target_y", target_y, 1.0);
        nh.param<double>("target_z", target_z, 1.0);
        nh.param<double>("target_yaw_end", target_yaw_end, 0.0);
        nh.param<double>("distance_between_drones", distance_between_drones, 0.5); // Example value

        double kp, ki, kd;
        nh.param<double>("kp", kp, 0.0);
        nh.param<double>("ki", ki, 0.0);
        nh.param<double>("kd", kd, 0.0);

        // Initialize PID controllers with retrieved parameters
        pid = PIDController(kp, ki, kd, 10.0);

        target_yaw_radians = normalizeAngle(target_yaw_end * M_PI / 180.0); // Initialize target_yaw_radians
        ROS_INFO("target_yaw_radians: %f", target_yaw_radians);
        setupCommunication();
        setupTimer();
    }

    void setupTimer() {
        update_timer = nh.createTimer(ros::Duration(1), &DroneController::updatePositions, this);
    }
    void stopTimer() {
        update_timer.stop();
    }

    void setupCommunication() {
        std::ostringstream odometry_topic1, odometry_topic2;
        odometry_topic1 << "/" << drone_id1 << "/agiros_pilot" << "/odometry";
        odometry_topic2 << "/" << drone_id2 << "/agiros_pilot" << "/odometry";
        odom_sub1 = nh.subscribe<nav_msgs::Odometry>("/falcon1/agiros_pilot/odometry", 10, &DroneController::odometryCallback1, this);
        odom_sub2 = nh.subscribe<nav_msgs::Odometry>(odometry_topic2.str(), 10, &DroneController::odometryCallback2, this);
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/falcon1/agiros_pilot/velocity_command", 10);
        vel_pub1 = nh.advertise<geometry_msgs::TwistStamped>("/falcon2/agiros_pilot/velocity_command", 10);
        odom_sub_bar = nh.subscribe<nav_msgs::Odometry>(bar_odometry_topic, 10, &DroneController::odometryCallbackBar, this);
    }

    double normalizeAngle(double angle) {
        angle = fmod(angle, 2 * M_PI); // Ensure angle is within 0 to 2*pi radians
        if (angle < 0.0) {
            angle += 2 * M_PI; // Convert negative angle to positive
        }
        if (angle > M_PI) {
            angle = 2 * M_PI - angle; // Convert angle to range 0 to pi radians
        }
        return angle;
    }

    void odometryCallback1(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position1 = msg->pose.pose.position;
        if (current_position1.z != 0.0) {
            odometry_received1 = true;
        }
        if (phase == ROTATION and initial_positions_set == true) {
            geometry_msgs::TwistStamped velocity_command1;
            velocity_command1.header.stamp = ros::Time::now();
            velocity_command1.twist = pid_turn.computeControl(target_position1, msg->pose.pose.position, 1.0 / 50.0);
            vel_pub.publish(velocity_command1);
        }
    }

    void odometryCallback2(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position2 = msg->pose.pose.position;
        if (current_position2.z != 0.0) {
            odometry_received2 = true;
        }
        if (phase == ROTATION and initial_positions_set == true) {
            geometry_msgs::TwistStamped velocity_command2;
            velocity_command2.header.stamp = ros::Time::now();
            velocity_command2.twist = pid_turn.computeControl(target_position2, msg->pose.pose.position, 1.0 / 50.0);
            vel_pub1.publish(velocity_command2);
        }
    }

    void odometryCallbackBar(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_bar = msg->pose.pose.position;
        if (phase == TRANSLATION and initial_positions_set == true) {
            geometry_msgs::TwistStamped velocity_command1;
            geometry_msgs::TwistStamped velocity_command2;
            velocity_command1.header.stamp = ros::Time::now();
            velocity_command2.header.stamp = ros::Time::now();
            velocity_command1.twist = pid.computeControl(target_position1, current_position_bar, 1.0 / 50.0);
            velocity_command2.twist = pid.computeControl(target_position2, current_position_bar, 1.0 / 50.0);
            vel_pub.publish(velocity_command1);
            vel_pub1.publish(velocity_command2);
        }
        double roll, pitch;
        tf::Quaternion q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw);
        current_yaw = normalizeAngle(current_yaw);
    }

    void updatePositions(const ros::TimerEvent&) {
        if (!odometry_received1 || !odometry_received2) {
            ROS_INFO("Waiting for odometry data from drones");
            return;
        }
        if (!initial_positions_set) {
            mid_x = (current_position1.x + current_position2.x) / 2.0;
            mid_y = (current_position1.y + current_position2.y) / 2.0;

            if (target_yaw_radians > current_yaw) {
                increment_yaw = 0.25*M_PI; 
            } else if (target_yaw_radians < current_yaw){
                increment_yaw = -0.25*M_PI;
            }
            new_target_yaw_radians = current_yaw;
            ROS_INFO("Start Yaw: %f, target yaw: %f, increment: %f", current_yaw, target_yaw_radians, increment_yaw);
            initial_positions_set = true;
        }

        if (phase == ROTATION) {
            double angle_diff = target_yaw_radians - new_target_yaw_radians;
            ROS_INFO("angle_diff: %f", angle_diff);
            if (fabs(angle_diff) < 0.25*M_PI) {
                new_target_yaw_radians = target_yaw_radians;
                ROS_INFO("sending final yaw");
                if (fabs(target_yaw_radians - current_yaw) < 0.1) {
                    ROS_INFO("Switching to TRANSLATION phase");
                    phase = TRANSLATION;
                }
            } else {
                new_target_yaw_radians += increment_yaw;
                new_target_yaw_radians = normalizeAngle(new_target_yaw_radians); // Normalize the angle
            }
            target_position1.x = mid_x + distance_between_drones * cos(new_target_yaw_radians);
            target_position1.y = mid_y + distance_between_drones * sin(new_target_yaw_radians);
            target_position1.z = target_z + 1;

            target_position2.x = mid_x - distance_between_drones * cos(new_target_yaw_radians);
            target_position2.y = mid_y - distance_between_drones * sin(new_target_yaw_radians);
            target_position2.z = target_z + 1;
            ROS_INFO("1: x: %f, y: %f, z: %f", target_position1.x, target_position1.y, target_position1.z);
            ROS_INFO("2: x: %f, y: %f, z: %f", target_position2.x, target_position2.y, target_position2.z);
        } else if (phase == TRANSLATION) {
            // Gebruik de huidige positie van de bar voor de translatie fase
            target_position1.x = target_x;
            target_position1.y = target_y;
            target_position1.z = target_z;

            target_position2.x = target_x;
            target_position2.y = target_y;
            target_position2.z = target_z;
            ROS_INFO("target_bar: x: %f, y: %f, z: %f", target_position1.x, target_position1.y, target_position1.z);
            stopTimer();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_turn_and_move_node");
    std::this_thread::sleep_for(std::chrono::seconds(6)); // Wait 4 seconds for takeoff
    DroneController controller;
    ros::spin();
    return 0;
}