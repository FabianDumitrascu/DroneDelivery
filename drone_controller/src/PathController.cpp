#include "ros/ros.h"
#include "agiros_msgs/Reference.h"
#include "agiros_msgs/Setpoint.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

class PathController {
public:
    PathController(ros::NodeHandle& nh) : nh_(nh) {
        // Read parameters from the parameter server
        nh_.getParam("target_x", target_x_);
        nh_.getParam("target_y", target_y_);
        nh_.getParam("target_z", target_z_);

        ROS_INFO("PathController initialized with target (x=%f, y=%f, z=%f)", 
                 target_x_, target_y_, target_z_);

        // Initialize the trajectory publisher
        trajectory_pub_ = nh_.advertise<agiros_msgs::Reference>("/flycrane/agiros_pilot/trajectory", 1);

        // Publish the target point
        publishTargetPoint();
    }

private:
    ros::NodeHandle& nh_;
    ros::Publisher trajectory_pub_;
    double target_x_, target_y_, target_z_;

    void publishTargetPoint() {
        agiros_msgs::Reference reference_msg;
        agiros_msgs::Setpoint setpoint;

        // Fill the header
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "world";  // Assuming 'world' frame, change as appropriate
        reference_msg.header = header;
        setpoint.state.header = header;

        // Fill the setpoint with the target point
        setpoint.state.pose.position.x = target_x_;
        setpoint.state.pose.position.y = target_y_;
        setpoint.state.pose.position.z = target_z_;
        setpoint.state.pose.orientation.w = 1.0; // Neutral orientation
        
        // Set a moderate velocity and reduced acceleration
        setpoint.state.velocity.linear.x = 0.2;
        setpoint.state.velocity.linear.y = 0.2;
        setpoint.state.velocity.linear.z = 0.1;
        setpoint.state.acceleration.linear.x = 0.1;
        setpoint.state.acceleration.linear.y = 0.1;
        setpoint.state.acceleration.linear.z = 0.1;
        
        // Set the time to reach the target point
        setpoint.state.t = 10.0; // 10 seconds

        // Add the setpoint to the reference message
        reference_msg.points.push_back(setpoint);

        // Publish the trajectory
        trajectory_pub_.publish(reference_msg);
        ROS_INFO("Target point published at (x=%f, y=%f, z=%f)", 
                 setpoint.state.pose.position.x, setpoint.state.pose.position.y, setpoint.state.pose.position.z);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "PathController_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting PathController_node...");
    PathController controller(nh);

    ros::spin();
    return 0;
}
