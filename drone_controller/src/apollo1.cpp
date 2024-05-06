#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <cmath>
#include <sstream>

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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apollo1_node");
    ros::NodeHandle nh("apollo1");
    ROS_INFO("Current namespace: %s", nh.getNamespace().c_str());


    // Parameters for target position and PID coefficients
    std::string drone_id;
    double target_x, target_y, target_z, kp, ki, kd;

    nh.getParam("drone_id", drone_id);
    nh.getParam("target_x", target_x);
    nh.getParam("target_y", target_y);
    nh.getParam("target_z", target_z);
    nh.getParam("kp", kp);
    nh.getParam("ki", ki);
    nh.getParam("kd", kd);
    
    ROS_INFO("Retrieved 'drone_id': %s", drone_id.c_str());
    ROS_INFO("Retrieved 'target_x': %f", target_x);
    ROS_INFO("Retrieved 'target_y': %f", target_y);
    ROS_INFO("Retrieved 'target_z': %f", target_z);
    ROS_INFO("Retrieved 'kp': %f", kp);
    ROS_INFO("Retrieved 'ki': %f", ki);
    ROS_INFO("Retrieved 'kd': %f", kd);

    std::ostringstream velocity_topic;
    std::ostringstream odometry_topic;

    velocity_topic << "/" << drone_id << "/agiros_pilot/velocity_command";
    odometry_topic << "/" << drone_id << "/agiros_pilot/odometry";

    PIDController pid(kp, ki, kd);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic.str(), 10);
    double loop_rate_hz = 50.0;
    ros::Rate loop_rate(loop_rate_hz);

    geometry_msgs::Point target_position;
    target_position.x = target_x;
    target_position.y = target_y;
    target_position.z = target_z;

    auto odometryCallback = [&vel_pub, &pid, &target_position, &loop_rate_hz](const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TwistStamped velocity_command;
        velocity_command.header.stamp = ros::Time::now();
        velocity_command.twist = pid.computeControl(target_position, msg->pose.pose.position, 1.0 / loop_rate_hz);
        vel_pub.publish(velocity_command);
    };

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odometry_topic.str(), 10, odometryCallback);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
