/*
功能：读取各topic的内容显示出来
*/


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>

// void state_cb(const mavros_msgs::State::ConstPtr& msg)
// { // state callback
//     current_state = *msg;
// }

// void local_pos_subCallback(const geometry_msgs::PoseStamped::ConstPtr& curr_p) // local position callback
// {
// 	current_pose = *curr_p;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_state");
    ros::NodeHandle nh;

    // ros::Subscriber state_sub = nh.subscribe("mavros/state", 1, state_subCallback);
    // ros::Subscriber local_position_sub = nh.subscribe("mavros/local_position/pose", 1, local_pos_subCallback);

    
}