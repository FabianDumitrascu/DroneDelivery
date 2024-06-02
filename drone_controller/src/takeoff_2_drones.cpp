#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "takeoff_2_drones");
    ros::NodeHandle nh("takeoff");
    ROS_INFO("Current namespace: %s", nh.getNamespace().c_str());

    // Retrieve drone names from the parameter server or set to default
    std::string drone_name_1, drone_name_2;

    nh.getParam("drone_name_1", drone_name_1);
    nh.getParam("drone_name_2", drone_name_2);

    ROS_INFO("Retrieved 'drone_name1': %s", drone_name_1.c_str());
    ROS_INFO("Retrieved 'drone_name2': %s", drone_name_2.c_str());

    // Set up the publishers for the drone commands
    ros::Publisher enable_pub_1 = nh.advertise<std_msgs::Bool>("/" + drone_name_1 + "/agiros_pilot/enable", 1, true);
    ros::Publisher force_hover_pub_1 = nh.advertise<std_msgs::Empty>("/" + drone_name_1 + "/agiros_pilot/force_hover", 1, true);
    ros::Publisher start_pub_1 = nh.advertise<std_msgs::Empty>("/" + drone_name_1 + "/agiros_pilot/start", 1, true);

    ros::Publisher enable_pub_2 = nh.advertise<std_msgs::Bool>("/" + drone_name_2 + "/agiros_pilot/enable", 1, true);
    ros::Publisher force_hover_pub_2 = nh.advertise<std_msgs::Empty>("/" + drone_name_2 + "/agiros_pilot/force_hover", 1, true);
    ros::Publisher start_pub_2 = nh.advertise<std_msgs::Empty>("/" + drone_name_2 + "/agiros_pilot/start", 1, true);

    // Ensure that the publishers are valid and the master has been notified of the advertisement
    if (!enable_pub_1 || !force_hover_pub_1 || !start_pub_1 || !enable_pub_2 || !force_hover_pub_2 || !start_pub_2) {
        ROS_ERROR("Failed to advertise on the necessary topics!");
        return -1;
    }

    // Sleep to give ROS time to set up the connection
    ros::Duration(1).sleep();

    std_msgs::Bool enable_msg;
    enable_msg.data = true;

    std_msgs::Empty empty_msg;

    // Publish the enable command
    enable_pub_1.publish(enable_msg);
    enable_pub_2.publish(enable_msg);
    ROS_INFO("Enable command sent to both drones.");
    
    // Sleep to ensure the messages have time to be sent
    ros::Duration(1).sleep();

    // Publish the force hover command
    force_hover_pub_1.publish(empty_msg);
    force_hover_pub_2.publish(empty_msg);
    ROS_INFO("Force hover command sent to both drones.");
    
    // Sleep to ensure the messages have time to be sent
    ros::Duration(1).sleep();

    // Publish the start command
    start_pub_1.publish(empty_msg);
    start_pub_2.publish(empty_msg);
    ROS_INFO("Start command sent to both drones.");

    // Sleep to ensure the messages have time to be sent
    ros::Duration(1).sleep();

    ROS_INFO("TAKEOFF SUCCEEDED");
    return 0;
}
