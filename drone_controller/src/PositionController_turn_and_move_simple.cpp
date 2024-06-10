#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include <cmath>
#include <thread>
#include <chrono>
#include <tf/tf.h>
#include <mutex>

class CirclePathController {
public:
    CirclePathController() : initial_positions_set(false), centerX(0.0), centerY(0.0), centerZ(1.0), startX(0.0), startY(0.0),
                             target_x(0.0), target_y(0.0), target_z(1.0), degree_increment(30.0), etha_yaw(0.0), etha_translation(0.0), regenboogpaard(0.0), simulation(false), Landing(false) {
        nh = ros::NodeHandle("~");
        std::string drone_id1, drone_id2;
        nh.getParam("drone_id1", drone_id1);
        nh.getParam("drone_id2", drone_id2);
        nh.getParam("target_x", target_x);
        nh.getParam("target_y", target_y);
        nh.getParam("target_z", target_z);
        nh.getParam("end_angle_degrees", end_angle_degrees);
        nh.getParam("increment_time", increment_time);
        nh.getParam("increment_degrees", increment_degrees);
        nh.getParam("etha_yaw", etha_yaw);
        nh.getParam("etha_translation", etha_translation);
        nh.getParam("simulation", simulation);

        odom_sub_falcon1 = nh.subscribe("/" + drone_id1 + "/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon1, this);
        odom_sub_falcon2 = nh.subscribe("/" + drone_id2 + "/agiros_pilot/odometry", 10, &CirclePathController::odometryCallbackFalcon2, this);
        if (simulation){
            ROS_INFO("Subscribing to bar topic");
            odom_sub_bar = nh.subscribe("/bar/odometry_sensor1/odometry", 10, &CirclePathController::barCallbacksimulation, this);
        }
        if (!simulation){
            ROS_INFO("Subscribing to mocap topic");
            odom_sub_bar = nh.subscribe("/mocap/bar_large/pose", 10, &CirclePathController::barCallback, this);
        }

        pose_pub_falcon1 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id1 + "/agiros_pilot/go_to_pose", 2);
        pose_pub_falcon2 = nh.advertise<geometry_msgs::PoseStamped>("/" + drone_id2 + "/agiros_pilot/go_to_pose", 2);
        timer = nh.createTimer(ros::Duration(increment_time), &CirclePathController::updateCallback, this, false, false);
    }

private:
    ros::NodeHandle nh;
    ros::Timer error_timer;
    bool error_timer_started = false;
    bool simulation;
    double currentYawBar;
    double end_angle_degrees;
    double etha_yaw;
    ros::Publisher pose_pub_falcon1, pose_pub_falcon2;
    ros::Publisher land_pub_1, land_pub_2;  // Add these as member variables
    ros::Subscriber odom_sub_falcon1, odom_sub_falcon2, odom_sub_bar;
    ros::Timer timer, initial_position_timer, shutdown_timer;
    bool initial_positions_set;
    double centerX, centerY, centerZ;
    double startX, startY;
    double target_x, target_y, target_z;
    double angle_degrees, start_angle_degrees;
    double increment_time, increment_degrees;
    double degree_increment, increment_x, increment_y;
    double roll, pitch, current_yaw;
    double regenboogpaard;
    double etha_translation;  // Declare this as a member variable
    bool Landing;
    geometry_msgs::Point initial_position_falcon1, initial_position_falcon2, current_position_bar;
    geometry_msgs::Pose currentPoseBar;  // Use geometry_msgs::Pose for currentPoseBar
    geometry_msgs::Point current_position_falcon1, current_position_falcon2;

    std::mutex data_mutex; // Mutex for synchronizing data access

    double normalizeEndAngleDegrees(double angle) {
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    void odometryCallbackFalcon1(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initial_positions_set) {
            initial_position_falcon1 = msg->pose.pose.position;
            checkInitialPositionsSet();
        } else { 
            current_position_falcon1 = msg->pose.pose.position;
        }
    }

    void odometryCallbackFalcon2(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initial_positions_set) {
            initial_position_falcon2 = msg->pose.pose.position;
            checkInitialPositionsSet();
        } else { 
            current_position_falcon2 = msg->pose.pose.position;
        }
    }
    void barCallbacksimulation(const nav_msgs::Odometry::ConstPtr& msg) {
        currentPoseBar = msg->pose.pose; 
        geometry_msgs::Quaternion q = msg->pose.pose.orientation;
        currentYawBar = QuaternionToYaw(q) * 180.0 / M_PI;
        double error_yaw, error_translation;
        error_yaw = currentYawBar - end_angle_degrees;
        error_translation = sqrt(pow(currentPoseBar.position.x - target_x, 2) + pow(currentPoseBar.position.y - target_y, 2));  // Access position members
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
        currentPoseBar = msg->pose;  // Use msg->pose for assignment
        geometry_msgs::Quaternion q = msg->pose.orientation;
        currentYawBar = QuaternionToYaw(q) * 180.0 / M_PI;
        double error_yaw, error_translation;
        error_yaw = 90 + currentYawBar - end_angle_degrees;
        error_translation = sqrt(pow(currentPoseBar.position.x - target_x, 2) + pow(currentPoseBar.position.y - target_y, 2));  // Access position members
        // ROS_INFO("error_yaw: %f, error_translation: %f, regenboogpaard: %f", error_yaw, error_translation, regenboogpaard);
        if (fabs(error_yaw) < etha_yaw && error_translation < etha_translation) { 
            regenboogpaard += 0.01;
            ROS_INFO("error_yaw: %f, error_translation: %f, regenboogpaard: %f", error_yaw, error_translation, regenboogpaard);
            if (regenboogpaard >= 5.0) {  // Use >= to ensure the timer condition is met
                Landscript();
            }
        } else {
            regenboogpaard = 0;
        }
    }

    void Landscript() {
        geometry_msgs::PoseStamped new_pose_falcon1, new_pose_falcon2;
        new_pose_falcon1.pose.position = current_position_falcon1;  // Assign position directly
        new_pose_falcon2.pose.position = current_position_falcon2;  // Assign position directly
        new_pose_falcon1.pose.position.z = 0.5;
        new_pose_falcon2.pose.position.z = 0.5;
        ROS_INFO("going down to point %f, %f, %f", new_pose_falcon1.pose.position.x, new_pose_falcon1.pose.position.y, new_pose_falcon1.pose.position.z);
        ROS_INFO("going down to point %f, %f, %f", new_pose_falcon2.pose.position.x, new_pose_falcon2.pose.position.y, new_pose_falcon2.pose.position.z);
        pose_pub_falcon1.publish(new_pose_falcon1);
        pose_pub_falcon2.publish(new_pose_falcon2);
        ros::Duration(3).sleep(); 
        ros::shutdown();
    }
    
    void checkInitialPositionsSet() {
        if (initial_position_falcon1.x != 0.0000000 && initial_position_falcon2.x != 0.0000000) {
            startX = (initial_position_falcon1.x + initial_position_falcon2.x) / 2;
            startY = (initial_position_falcon1.y + initial_position_falcon2.y) / 2;
            double dx = initial_position_falcon1.x - startX;
            double dy = initial_position_falcon1.y - startY;
            centerX = startX;
            centerY = startY;
            end_angle_degrees = normalizeEndAngleDegrees(end_angle_degrees);
            start_angle_degrees = atan2(dy, dx) * 180.0 / M_PI; // Deze is sowieso tussen -180 en 180 
            angle_degrees = start_angle_degrees;
            if (end_angle_degrees > start_angle_degrees) {
                degree_increment = increment_degrees; 
            } else if (end_angle_degrees < start_angle_degrees) {
                degree_increment = -increment_degrees; 
            } else {
                degree_increment = 0.0;
            }
            if (end_angle_degrees - start_angle_degrees == 0.0) {
                increment_x = target_x - startX;
                increment_y = target_y - startY;
            } else {
                increment_x = (target_x - startX) / std::abs((end_angle_degrees - start_angle_degrees) / degree_increment);
                increment_y = (target_y - startY) / std::abs((end_angle_degrees - start_angle_degrees) / degree_increment);
            }
            initial_positions_set = true;
            ROS_INFO("Initial info: startX=%f, startY=%f, start angle=%f, start degree increment=%f, start increment x=%f, start increment y=%f", startX, startY, angle_degrees, degree_increment, increment_x, increment_y);
            // Start a one-time timer of 1 second before starting the regular updateCallback
            initial_position_timer = nh.createTimer(ros::Duration(0.1), &CirclePathController::startRegularTimerCallback, this, true);
        }
    }

    void startRegularTimerCallback(const ros::TimerEvent&) {
        timer.start();
    }

    void updateCallback(const ros::TimerEvent&) {
        double next_angle = normalizeEndAngleDegrees(angle_degrees + degree_increment);

        if (end_angle_degrees > start_angle_degrees) {
            if (next_angle >= end_angle_degrees) {
                angle_degrees = end_angle_degrees;
                ROS_INFO("Reached end angle: %f degrees", angle_degrees);
                timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
                updateDronesPosition();
                
            } else {
                angle_degrees = next_angle;
            }
        } else {
            if (next_angle <= end_angle_degrees) {
                angle_degrees = end_angle_degrees;
                ROS_INFO("Reached end angle: %f degrees", angle_degrees);
                timer.stop(); // Stop the main timer to prevent further update and set a one-time timer to shut down after 10 seconds
                updateDronesPosition();
            } else {
                angle_degrees = next_angle;
            }
        }
        updateDronesPosition();
    }

    double QuaternionToYaw(geometry_msgs::Quaternion q) {
        double roll, pitch, yaw;
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void updateDronesPosition() {
        double radius = 0.55;
        double angle_rad = degreesToRadians(angle_degrees);
        centerX += increment_x;
        centerY += increment_y;
        if (startX <= target_x && centerX >= target_x) {
            centerX = target_x;
        } 
        if (startX >= target_x && centerX <= target_x) {
            centerX = target_x;
        }
        if (startY <= target_y && centerY >= target_y) {
            centerY = target_y;
        }
        if (startY >= target_y && centerY <= target_y) {
            centerY = target_y;
        }
        
        centerZ = target_z;

        geometry_msgs::PoseStamped new_pose_falcon1, new_pose_falcon2;
        new_pose_falcon1.pose.position.x = centerX + radius * cos(angle_rad);
        new_pose_falcon1.pose.position.y = centerY + radius * sin(angle_rad);
        new_pose_falcon1.pose.position.z = centerZ + 1.0;
        new_pose_falcon2.pose.position.x = centerX + radius * cos(angle_rad + M_PI);
        new_pose_falcon2.pose.position.y = centerY + radius * sin(angle_rad + M_PI);
        new_pose_falcon2.pose.position.z = centerZ + 1.0;

        pose_pub_falcon1.publish(new_pose_falcon1);
        pose_pub_falcon2.publish(new_pose_falcon2);

        ROS_INFO("Updated angle: %f", angle_degrees);
        ROS_INFO("Center position: x=%f, y=%f, z=%f", centerX, centerY, centerZ);
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char **argv) {
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second
    ros::init(argc, argv, "circle_path_controller");
    CirclePathController controller;
    ros::spin();
    return 0;
}
