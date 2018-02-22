/**
 * \state_publisher.cpp
 * \brief A node to scale and republish the Image from Miro's left eye camera. 
 * \authors John Thomas,Parag Khanna 
 * \version 0.1
 * \date  11 February 2018
 * 
 * \param[in] ...
 * 
 *   
 * This node updates the joint state of MiRo robot model.
 */


#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(11);
        joint_state.position.resize(11);
        
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
