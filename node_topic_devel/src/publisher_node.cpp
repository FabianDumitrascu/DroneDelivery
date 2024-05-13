#include "ros/ros.h"
#include "node_topic_devel/person_data.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Publisher");
    ros::NodeHandle nh;

    ros::Publisher topic_pub = nh.advertise<node_topic_devel::person_data>("tutorial", 1000); 
    ros::Rate loop_rate(1);

    while(ros::ok()) {
        node_topic_devel::person_data person_data;
        person_data.name = "John";
        person_data.age = 25;
        person_data.color = "blue";

        topic_pub.publish(person_data);
        ros::spinOnce();
        loop_rate.sleep();

    }
    
    return 0;

}

